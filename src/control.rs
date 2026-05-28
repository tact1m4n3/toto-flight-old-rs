use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker};
use futures::join;
use nalgebra::Vector3;

use crate::{
    RawMutex,
    config::{Config, MotorMixerConfig, RateControllerAxisConfig, ServoMixerConfig},
    drivers::motor::Motor,
    imu,
    utils::{GuardedRefCell, NalgebraVec3Ext},
};

const FAILSAFE_INTERVAL: Duration = Duration::from_millis(500);

static COMMAND: Signal<RawMutex, Command> = Signal::new();

pub enum Command {
    Disarmed,
    Manual { throttle: u16, throw: Vector3<f32> },
    Rate { throttle: u16, rate: Vector3<f32> },
    Angle { throttle: u16, angle: Vector3<f32> },
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
                command: Command::Disarmed,

                slow_loop_target_rate: None,

                rate_controller: RateController {
                    roll: AxisRateController {
                        cfg: cfg.rate_controller_config.roll,
                        i_term: 0.0,
                    },
                    pitch: AxisRateController {
                        cfg: cfg.rate_controller_config.pitch,
                        i_term: 0.0,
                    },
                    yaw: AxisRateController {
                        cfg: cfg.rate_controller_config.yaw,
                        i_term: 0.0,
                    },
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
        let mut armed = false;

        loop {
            if let Ok(command) = embassy_time::with_timeout(FAILSAFE_INTERVAL, COMMAND.wait()).await
            {
                if !armed
                    && !command.is_disarmed()
                    && let Ok(()) = ArmBlocker::arm()
                {
                    armed = true;
                } else if armed && command.is_disarmed() {
                    ArmBlocker::disarm();
                    armed = false;
                }

                self.state.guarded_mut(|state| {
                    state.command = if armed { command } else { Command::Disarmed }
                });
                continue;
            }

            todo!("handle failsafe logic")
        }
    }

    async fn fast_loop<M: Motor, S: Motor>(&self, mut outputs: Outputs<MOTORS, SERVOS, M, S>) {
        const FREQ_HZ: u64 = 500;
        const DT: f32 = 1.0 / FREQ_HZ as f32;

        let mut ticker = Ticker::every(Duration::from_hz(FREQ_HZ));

        loop {
            ticker.next().await;

            self.state.guarded_mut(|state| {
                // Will be `None` only if the passthrough rate_control from the slow loop is late.
                let maybe_manual_control = match state.command {
                    Command::Disarmed => Some(ManualControl::IDLE),
                    Command::Manual { throttle, throw } => Some(ManualControl { throttle, throw }),
                    _ => {
                        let maybe_rate_control =
                            if let Command::Rate { throttle, rate } = state.command {
                                Some(RateControl { throttle, rate })
                            } else {
                                state.slow_loop_target_rate.clone()
                            };

                        maybe_rate_control.map(|rate_control| {
                            let gyro = imu::GYRO.get();
                            rate_control.run(&mut state.rate_controller, gyro, DT)
                        })
                    }
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
        const DT: f32 = 1.0 / FREQ_HZ as f32;

        let mut ticker = Ticker::every(Duration::from_hz(FREQ_HZ));

        loop {
            ticker.next().await;

            self.state.guarded_mut(|state| {
                state.slow_loop_target_rate = match state.command {
                    Command::Angle { throttle, angle } => todo!("angle mode not supported yet"),
                    _ => None,
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

#[derive(Clone)]
struct RateControl {
    throttle: u16,
    rate: Vector3<f32>,
}

impl RateControl {
    fn run(
        self,
        rate_controller: &mut RateController,
        gyro: Vector3<f32>,
        dt: f32,
    ) -> ManualControl {
        let Self { throttle, rate } = self;
        ManualControl {
            throttle,
            throw: rate_controller.update(rate, gyro, dt),
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
        target_rate: Vector3<f32>,
        current_rate: Vector3<f32>,
        dt: f32,
    ) -> Vector3<f32> {
        Vector3::new(
            self.roll.update(target_rate.x, current_rate.x, dt),
            self.pitch.update(target_rate.y, current_rate.y, dt),
            self.yaw.update(target_rate.z, current_rate.z, dt),
        )
    }
}
