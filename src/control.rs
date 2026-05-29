use core::{
    ops::Add,
    sync::atomic::{AtomicU32, Ordering},
};

use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker};
use futures::join;
use nalgebra::{UnitQuaternion, Vector3};
use num_traits::Float;

use crate::{
    RawMutex,
    config::{Config, MotorMixerConfig, RateControllerAxisConfig, ServoMixerConfig},
    drivers::motor::Motor,
    fusion, imu,
    utils::{GuardedRefCell, NalgebraVec3Ext},
};

const VALID_COMMAND_RATE: Duration = Duration::from_millis(100);
const FAILSAFE_MONITOR_PERIOD: Duration = Duration::from_millis(300);
const FAILSAFE_RECOVERY_MIN_COMMAND_RATE: Duration = Duration::from_hz(10);
const FAILSAFE_RECOVERY_PERIOD: Duration = Duration::from_millis(200);

pub static RC_COMMAND: Signal<RawMutex, Command> = Signal::new();

pub enum Command {
    Disarmed,
    Manual {
        throttle: u16,
        throw: Vector3<f32>,
    },
    Rate {
        throttle: u16,
        rate: Vector3<f32>,
    },
    Angle {
        throttle: u16,
        roll_angle: f32,
        pitch_angle: f32,
    },
}

impl Command {
    fn is_disarmed(&self) -> bool {
        matches!(self, Command::Disarmed)
    }
}

#[derive(Clone, Copy)]
pub struct Outputs<const MOTORS: usize, const SERVOS: usize, M: Motor, S: Motor> {
    pub motors: [M; MOTORS],
    pub servos: [S; SERVOS],
}

pub struct Control<const MOTORS: usize, const SERVOS: usize> {
    state: GuardedRefCell<State<MOTORS, SERVOS>>,
}

struct State<const MOTORS: usize, const SERVOS: usize> {
    armed: bool,

    command: Command,

    slow_loop_target_rate: Option<RateControl>,

    rate_controller: RateController,

    motor_mixer_config: [MotorMixerConfig; MOTORS],
    servo_mixer_config: [ServoMixerConfig; SERVOS],
}

impl<const MOTORS: usize, const SERVOS: usize> Control<MOTORS, SERVOS> {
    pub fn new(cfg: &'static Config) -> Self {
        Self {
            state: GuardedRefCell::new(State {
                armed: false,

                command: Command::Disarmed,

                slow_loop_target_rate: None,

                rate_controller: RateController {
                    roll: AxisRateController::new(cfg.rate_controller_config.roll),
                    pitch: AxisRateController::new(cfg.rate_controller_config.pitch),
                    yaw: AxisRateController::new(cfg.rate_controller_config.yaw),
                },

                motor_mixer_config: core::array::from_fn(|i| cfg.motors_config[i]),
                servo_mixer_config: core::array::from_fn(|i| cfg.servos_config[i]),
            }),
        }
    }

    pub async fn run<M: Motor, S: Motor>(&self, outputs: Outputs<MOTORS, SERVOS, M, S>) {
        join!(
            self.command_guardian(),
            self.slow_loop(),
            self.fast_loop(outputs),
            // self.calibration_loop()
        );
    }

    async fn command_guardian(&self) {
        'failsafe_start: loop {
            // Disarm on failsafe
            self.state.guarded_mut(|state| {
                state.command = Command::Disarmed;
            });

            // In failsafe
            let mut maybe_good_signal_timeout_start: Option<Instant> = None;
            loop {
                match embassy_time::with_timeout(
                    FAILSAFE_RECOVERY_MIN_COMMAND_RATE,
                    RC_COMMAND.wait(),
                )
                .await
                {
                    Ok(_) => {
                        if let Some(good_signal_timeout_start) = maybe_good_signal_timeout_start
                            && good_signal_timeout_start - Instant::now()
                                >= FAILSAFE_RECOVERY_PERIOD
                        {
                            // We can exit failsafe now
                            break;
                        } else {
                            maybe_good_signal_timeout_start = Some(Instant::now());
                        }
                    }
                    Err(_) => maybe_good_signal_timeout_start = None,
                }
            }

            loop {
                // Receive command loop. No failsafe in sight
                while let Ok(command) =
                    embassy_time::with_timeout(VALID_COMMAND_RATE, RC_COMMAND.wait()).await
                {
                    self.state.guarded_mut(|state| {
                        state.command = command;
                    });
                }

                // Maybe we are in failsafe, do we receive any more packets in this recovery period?
                let recovery_deadline = Instant::now().add(FAILSAFE_MONITOR_PERIOD);
                if embassy_time::with_deadline(recovery_deadline, RC_COMMAND.wait())
                    .await
                    .is_err()
                {
                    // No command received in the failsafe monitor period, so we enter failsafe again
                    continue 'failsafe_start;
                }
            }
        }
    }

    async fn fast_loop<M: Motor, S: Motor>(&self, mut outputs: Outputs<MOTORS, SERVOS, M, S>) {
        const FREQ_HZ: u64 = 500;
        const DT: f32 = 1.0 / FREQ_HZ as f32;

        let mut ticker = Ticker::every(Duration::from_hz(FREQ_HZ));

        loop {
            ticker.next().await;

            self.state.guarded_mut(|state| {
                // Arm state transitions
                if !state.armed
                    && !state.command.is_disarmed()
                    && let Ok(()) = ArmBlocker::arm()
                {
                    state.armed = true;
                } else if state.armed && state.command.is_disarmed() {
                    ArmBlocker::disarm();
                    state.armed = false;
                }

                // Will be `None` only if the passthrough rate_control from the slow loop is late
                let maybe_manual_control = if state.armed {
                    match state.command {
                        Command::Disarmed => Some(ManualControl::IDLE),
                        Command::Manual { throttle, throw } => {
                            Some(ManualControl { throttle, throw })
                        }
                        _ => {
                            let maybe_rate_control =
                                if let Command::Rate { throttle, rate } = state.command {
                                    Some(RateControl { throttle, rate })
                                } else {
                                    state.slow_loop_target_rate.clone()
                                };

                            maybe_rate_control.map(|rate_control| {
                                let gyro = imu::GYRO.get();
                                rate_control.target_manual(&mut state.rate_controller, &gyro, DT)
                            })
                        }
                    }
                } else {
                    Some(ManualControl::IDLE)
                };

                if let Some(manual_control) = maybe_manual_control {
                    manual_control
                        .mix(&state.motor_mixer_config, &state.servo_mixer_config)
                        .apply(&mut outputs);
                }
            });
        }
    }

    async fn slow_loop(&self) {
        const FREQ_HZ: u64 = 100;
        // const DT: f32 = 1.0 / FREQ_HZ as f32;

        let mut ticker = Ticker::every(Duration::from_hz(FREQ_HZ));

        loop {
            ticker.next().await;

            self.state.guarded_mut(|state| {
                state.slow_loop_target_rate = if state.armed {
                    match state.command {
                        Command::Angle {
                            throttle,
                            roll_angle,
                            pitch_angle,
                        } => {
                            let attitude = fusion::ATTITUDE.get();
                            Some(
                                AngleControl {
                                    throttle,
                                    roll_angle,
                                    pitch_angle,
                                }
                                .target_rate(&attitude),
                            )
                        }
                        _ => None,
                    }
                } else {
                    None
                };
            });
        }
    }

    // async fn calibration_loop(&self) {}
}

static ARM_BLOCK: AtomicU32 = AtomicU32::new(0);
const ARMED_FLAG_MASK: u32 = 1 << 31;

#[derive(Debug)]
pub struct AlreadyArmed;

pub struct ArmBlocker(());

impl ArmBlocker {
    pub fn new() -> Result<Self, AlreadyArmed> {
        let state = ARM_BLOCK.fetch_add(1, Ordering::Relaxed);
        assert!(state & !ARMED_FLAG_MASK < 0xFFF_FFFF);
        if state & ARMED_FLAG_MASK != 0 {
            Err(AlreadyArmed)
        } else {
            Ok(Self(()))
        }
    }

    fn arm() -> Result<(), u32> {
        loop {
            let state = ARM_BLOCK.load(Ordering::Relaxed);
            // If already armed
            if state & ARMED_FLAG_MASK != 0 {
                break Ok(());
            }
            // If there are any arm blockers
            if state & !ARMED_FLAG_MASK != 0 {
                break Err(state & !ARMED_FLAG_MASK);
            }
            // This should never fail on a single core
            if ARM_BLOCK
                .compare_exchange_weak(
                    state,
                    state | ARMED_FLAG_MASK,
                    Ordering::Relaxed,
                    Ordering::Relaxed,
                )
                .is_ok()
            {
                break Ok(());
            }
        }
    }

    fn disarm() {
        let state = ARM_BLOCK.load(Ordering::Relaxed);
        // If armed, reset the field
        if state & ARMED_FLAG_MASK != 0 {
            ARM_BLOCK.store(0, Ordering::Relaxed);
        }
    }
}

impl Drop for ArmBlocker {
    fn drop(&mut self) {
        assert!(ARM_BLOCK.fetch_sub(1, Ordering::Release) != 0);
    }
}

struct AngleControl {
    throttle: u16,
    roll_angle: f32,
    pitch_angle: f32,
}

impl AngleControl {
    fn target_rate(self, attitude: &UnitQuaternion<f32>) -> RateControl {
        const ANGLE_ROLL_KP: f32 = 1.0;
        const ANGLE_PITCH_KP: f32 = 1.0;

        let world_up = -Vector3::z();
        let body_up = attitude.inverse_transform_vector(&world_up);
        let target_up = UnitQuaternion::from_euler_angles(self.roll_angle, self.pitch_angle, 0.0)
            .inverse_transform_vector(&world_up);

        let angle_error = {
            let cross = body_up.cross(&target_up);
            let dot = body_up.dot(&target_up).clamp(-1.0, 1.0);

            let angle = dot.atan2(cross.norm());

            cross
                .try_normalize(1e-6)
                .map(|cross_normalized| cross_normalized * angle)
                .unwrap_or(Vector3::zeros())
        };

        RateControl {
            throttle: self.throttle,
            rate: Vector3::new(
                angle_error.x * ANGLE_ROLL_KP,
                angle_error.y * ANGLE_PITCH_KP,
                0.0,
            ),
        }
    }
}

#[derive(Clone)]
struct RateControl {
    throttle: u16,
    rate: Vector3<f32>,
}

impl RateControl {
    fn target_manual(
        self,
        rate_controller: &mut RateController,
        gyro: &Vector3<f32>,
        dt: f32,
    ) -> ManualControl {
        let Self { throttle, rate } = self;
        ManualControl {
            throttle,
            throw: rate_controller.update(&rate, gyro, dt),
        }
    }
}

struct ManualControl {
    throttle: u16,
    throw: Vector3<f32>,
}

impl ManualControl {
    const IDLE: Self = Self {
        throttle: 1000,
        throw: Vector3::ZEROS,
    };

    fn mix<const MOTORS: usize, const SERVOS: usize>(
        self,
        motors_cfg: &[MotorMixerConfig; MOTORS],
        servos_cfg: &[ServoMixerConfig; SERVOS],
    ) -> ActuatorOutput<MOTORS, SERVOS> {
        let Self { throttle, throw } = self;

        ActuatorOutput {
            motors: core::array::from_fn(|i| {
                let value = throttle as i16 + throw.dot(&motors_cfg[i].mixer_gain) as i16;
                value.clamp(motors_cfg[i].trim_min_us, motors_cfg[i].trim_max_us) as u16
            }),
            servos: core::array::from_fn(|i| {
                let value =
                    servos_cfg[i].mid_point_us + throw.dot(&servos_cfg[i].mixer_gain) as i16;
                value.clamp(servos_cfg[i].trim_min_us, servos_cfg[i].trim_max_us) as u16
            }),
        }
    }
}

struct ActuatorOutput<const MOTORS: usize, const SERVOS: usize> {
    motors: [u16; MOTORS],
    servos: [u16; SERVOS],
}

impl<const MOTORS: usize, const SERVOS: usize> ActuatorOutput<MOTORS, SERVOS> {
    fn apply<M: Motor, S: Motor>(self, outputs: &mut Outputs<MOTORS, SERVOS, M, S>) {
        for (motor, value) in outputs.motors.iter_mut().zip(self.motors) {
            motor.write(value);
        }
        for (servo, value) in outputs.servos.iter_mut().zip(self.servos) {
            servo.write(value);
        }
    }
}

struct AxisRateController {
    cfg: RateControllerAxisConfig,
    i_term: f32,
}

impl AxisRateController {
    fn new(cfg: RateControllerAxisConfig) -> Self {
        Self { cfg, i_term: 0.0 }
    }

    fn update(&mut self, target_rate: f32, current_rate: f32, dt: f32) -> f32 {
        let error = target_rate - current_rate;

        let ff_term = self.cfg.kff * target_rate;
        let p_term = self.cfg.kp * error;

        const I_TERM_MAX_ERROR: f32 = 165.0;
        if error.abs() <= I_TERM_MAX_ERROR {
            self.i_term += self.cfg.ki * error * dt;
        }

        ff_term + p_term + self.i_term
    }
}

struct RateController {
    roll: AxisRateController,
    pitch: AxisRateController,
    yaw: AxisRateController,
}

impl RateController {
    fn i_term(&self) -> Vector3<f32> {
        Vector3::new(self.roll.i_term, self.pitch.i_term, self.yaw.i_term)
    }

    fn reset_i_term(&mut self) {
        self.roll.i_term = 0.0;
        self.pitch.i_term = 0.0;
        self.yaw.i_term = 0.0;
    }

    fn update(
        &mut self,
        target_rate: &Vector3<f32>,
        current_rate: &Vector3<f32>,
        dt: f32,
    ) -> Vector3<f32> {
        Vector3::new(
            self.roll.update(target_rate.x, current_rate.x, dt),
            self.pitch.update(target_rate.y, current_rate.y, dt),
            self.yaw.update(target_rate.z, current_rate.z, dt),
        )
    }
}
