use core::cell::RefCell;

use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::Mutex, signal::Signal};
use embassy_time::{Duration, Instant, Ticker};
use futures::join;
use nalgebra::{UnitQuaternion, Vector3};
use num_traits::Float;

use crate::{
    RawMutex,
    config::{self, MotorMixerConfig, RateControllerAxisConfig, ServoMixerConfig},
    drivers::motor::Motor,
    receiver, sensors,
    utils::{DurationExt, GuardedRefCell, NalgebraVec3Ext},
};

// TODO: Maybe move command stuff to another module

pub static HEARTBEAT_RECEIVER_COMMAND: Signal<RawMutex, ()> = Signal::new();
pub static AUTO_TUNE: Signal<RawMutex, bool> = Signal::new();

pub static SHARED_STATE: Mutex<RawMutex, RefCell<SharedState>> =
    Mutex::new(RefCell::new(SharedState {
        arm_state: ArmState::ArmFailed,
        arm_blockers: 0,
        failsafe: true,

        command: Command::Disarmed,
        slow_loop_target_rate: None,
    }));

pub struct SharedState {
    arm_state: ArmState,
    arm_blockers: u32,
    failsafe: bool,

    command: Command,
    slow_loop_target_rate: Option<RateControl>,
}

impl SharedState {
    pub fn arm_state(&self) -> ArmState {
        self.arm_state
    }
    pub fn in_failsafe(&self) -> bool {
        self.failsafe
    }
}

pub fn arm_state() -> ArmState {
    SHARED_STATE.lock(|state| state.borrow().arm_state)
}

pub fn in_failsafe() -> bool {
    SHARED_STATE.lock(|state| state.borrow().failsafe)
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ArmState {
    Disarmed,
    Armed,
    ArmFailed,
}

impl ArmState {
    pub fn is_armed(self) -> bool {
        self == ArmState::Armed
    }
}

#[derive(Clone, Copy)]
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

#[derive(Clone, Copy)]
pub struct Outputs<const MOTORS: usize, const SERVOS: usize, M: Motor, S: Motor> {
    pub motors: [M; MOTORS],
    pub servos: [S; SERVOS],
}

pub async fn task_command_guardian() {
    const VALID_COMMAND_RATE: Duration = Duration::from_millis(100);
    const FAILSAFE_MONITOR_PERIOD: Duration = Duration::from_millis(300);
    const FAILSAFE_RECOVERY_MIN_COMMAND_RATE: Duration = Duration::from_hz(10);
    const FAILSAFE_RECOVERY_PERIOD: Duration = Duration::from_millis(200);

    'failsafe_start: loop {
        info!("we are in failsafe");

        // Disarm on failsafe
        SHARED_STATE.lock(|state| {
            let mut state = state.borrow_mut();
            state.failsafe = true;
            state.arm_state = ArmState::Disarmed;
            state.command = Command::Disarmed;
        });

        // In failsafe
        let mut maybe_good_signal_timeout_start: Option<Instant> = None;
        loop {
            match embassy_time::with_timeout(
                FAILSAFE_RECOVERY_MIN_COMMAND_RATE,
                HEARTBEAT_RECEIVER_COMMAND.wait(),
            )
            .await
            {
                Ok(_) => {
                    if let Some(good_signal_timeout_start) = maybe_good_signal_timeout_start
                        && Instant::now() - good_signal_timeout_start >= FAILSAFE_RECOVERY_PERIOD
                    {
                        // We can exit failsafe now
                        break;
                    } else if maybe_good_signal_timeout_start.is_none() {
                        maybe_good_signal_timeout_start = Some(Instant::now());
                    }
                }
                Err(_) => maybe_good_signal_timeout_start = None,
            }
        }

        info!("failsafe has come to an end");
        SHARED_STATE.lock(|state| {
            state.borrow_mut().failsafe = false;
        });

        loop {
            // Receive command loop. No failsafe in sight
            while let Ok(()) =
                embassy_time::with_timeout(VALID_COMMAND_RATE, HEARTBEAT_RECEIVER_COMMAND.wait())
                    .await
            {
                let command = receiver::COMMAND.get();

                SHARED_STATE.lock(|shared_state| {
                    let mut shared_state = shared_state.borrow_mut();

                    // Arm state transitions
                    match shared_state.arm_state {
                        ArmState::Disarmed => match command {
                            Command::Manual { throttle, .. }
                            | Command::Rate { throttle, .. }
                            | Command::Angle { throttle, .. } => {
                                if throttle < 1100 && shared_state.arm_blockers == 0 {
                                    info!("armed");
                                    shared_state.arm_state = ArmState::Armed;
                                } else {
                                    info!("arming failed");
                                    shared_state.arm_state = ArmState::ArmFailed;
                                }
                            }
                            Command::Disarmed => (),
                        },
                        ArmState::Armed | ArmState::ArmFailed => {
                            if let Command::Disarmed = command {
                                info!("disarmed");
                                shared_state.arm_state = ArmState::Disarmed;
                            }
                        }
                    }

                    shared_state.command = if shared_state.arm_state == ArmState::Armed {
                        command
                    } else {
                        Command::Disarmed
                    };
                });
            }

            info!("failsafe detect stage 1");

            // Maybe we are in failsafe, do we receive any more packets in this recovery period?
            if embassy_time::with_timeout(
                FAILSAFE_MONITOR_PERIOD,
                HEARTBEAT_RECEIVER_COMMAND.wait(),
            )
            .await
            .is_err()
            {
                // No command received in the failsafe monitor period, so we enter failsafe again
                continue 'failsafe_start;
            }
        }
    }
}

struct InnerLoopState<const MOTORS: usize, const SERVOS: usize> {
    rate_controller: RateController,

    motor_mixer_config: [MotorMixerConfig; MOTORS],
    servo_mixer_config: [ServoMixerConfig; SERVOS],
}

pub async fn task_inner_loop<const MOTORS: usize, const SERVOS: usize, M: Motor, S: Motor>(
    mut outputs: Outputs<MOTORS, SERVOS, M, S>,
) {
    let cfg = config::INITIAL.get().await;

    let state: GuardedRefCell<InnerLoopState<MOTORS, SERVOS>> =
        GuardedRefCell::new(InnerLoopState {
            rate_controller: RateController {
                roll: AxisRateController::new(cfg.rate_controller_config.roll),
                pitch: AxisRateController::new(cfg.rate_controller_config.pitch),
                yaw: AxisRateController::new(cfg.rate_controller_config.yaw),
            },
            motor_mixer_config: core::array::from_fn(|i| cfg.motors_config[i]),
            servo_mixer_config: core::array::from_fn(|i| cfg.servos_config[i]),
        });

    let rate_control = async {
        const PERIOD: Duration = Duration::from_hz(500);

        let mut ticker = Ticker::every(PERIOD);
        loop {
            ticker.next().await;

            state.guarded_mut(|state| {
                let (command, slow_loop_target_rate) = SHARED_STATE.lock(|shared_state| {
                    let shared_state = shared_state.borrow_mut();
                    (
                        shared_state.command,
                        shared_state.slow_loop_target_rate.clone(),
                    )
                });

                // Will be `None` only if the passthrough rate_control from the slow loop is late
                let maybe_manual_control = match command {
                    Command::Disarmed => Some(ManualControl::IDLE),
                    Command::Manual { throttle, throw } => Some(ManualControl { throttle, throw }),
                    _ => {
                        let maybe_rate_control = if let Command::Rate { throttle, rate } = command {
                            Some(RateControl { throttle, rate })
                        } else {
                            slow_loop_target_rate
                        };

                        maybe_rate_control.map(|rate_control| {
                            let gyro = sensors::imu::DATA.get().gyro;
                            rate_control.target_manual(
                                &mut state.rate_controller,
                                &gyro,
                                PERIOD.dt_f32(),
                            )
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
    };

    let auto_tune = async {
        // TODO: feedback on which state is active through osd or telemetry

        loop {
            // Waiting to start mid point calibration, requires the drone to be armed
            loop {
                if AUTO_TUNE.wait().await
                    && SHARED_STATE.lock(|state| state.borrow().arm_state.is_armed())
                {
                    break;
                }
            }

            // Mid point tuning
            {
                let mut ticker = Ticker::every(Duration::from_hz(1));
                loop {
                    match select(AUTO_TUNE.wait(), ticker.next()).await {
                        Either::First(enable) if !enable => break,
                        _ => (),
                    }

                    state.guarded_mut(|state| {
                        if SHARED_STATE.lock(|state| state.borrow().arm_state.is_armed()) {
                            let i_term = state.rate_controller.i_term();
                            state.rate_controller.reset_i_term();

                            // this should have no side-effect as the i_term is reset
                            for cfg in state.servo_mixer_config.iter_mut() {
                                cfg.mid_point_us = cfg
                                    .mid_point_us
                                    .wrapping_add_signed(i_term.dot(&cfg.mixer_gain) as i16)
                                    .clamp(cfg.trim_min_us, cfg.trim_max_us);
                            }
                        }
                    });
                }
            }

            // TODO: add auto ff tuning

            // Waiting for config to be applied, requires the drone to be disarmed
            let _arm_guard = loop {
                if AUTO_TUNE.wait().await
                    && let Ok(arm_guard) = ArmBlocker::new()
                {
                    break arm_guard;
                }
            };

            let servo_mixer_config = state.guarded_ref(|state| state.servo_mixer_config);
            for (i, cfg) in servo_mixer_config.iter().enumerate() {
                config::UpdateItem::ServoMixerConfig {
                    index: i as u8,
                    config: *cfg,
                }
                .commit()
                .await
                .ok();
            }
        }
    };

    join!(rate_control, auto_tune);
}

pub async fn task_outer_loop() {
    const PERIOD: Duration = Duration::from_hz(100);

    let mut ticker = Ticker::every(PERIOD);
    loop {
        ticker.next().await;

        let command = SHARED_STATE.lock(|shared_state| shared_state.borrow().command);

        let slow_loop_target_rate = match command {
            Command::Angle {
                throttle,
                roll_angle,
                pitch_angle,
            } => {
                let attitude = sensors::fusion::ATTITUDE.get();

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
        };

        SHARED_STATE.lock(|shared_state| {
            shared_state.borrow_mut().slow_loop_target_rate = slow_loop_target_rate;
        });
    }
}

#[derive(Debug)]
pub struct CraftArmed;

pub struct ArmBlocker(());

impl ArmBlocker {
    pub fn new() -> Result<Self, CraftArmed> {
        SHARED_STATE.lock(|shared_state| {
            let mut shared_state = shared_state.borrow_mut();
            if shared_state.arm_state != ArmState::Armed {
                shared_state.arm_blockers += 1;
                Ok(Self(()))
            } else {
                Err(CraftArmed)
            }
        })
    }
}

impl Drop for ArmBlocker {
    fn drop(&mut self) {
        SHARED_STATE.lock(|shared_state| {
            let mut shared_state = shared_state.borrow_mut();
            assert!(shared_state.arm_blockers > 0);
            assert!(shared_state.arm_state != ArmState::Armed);
            shared_state.arm_blockers -= 1;
        });
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
                .map_or(Vector3::zeros(), |cross_normalized| {
                    cross_normalized * angle
                })
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
                throttle
                    .wrapping_add_signed(throw.dot(&motors_cfg[i].mixer_gain) as i16)
                    .clamp(motors_cfg[i].trim_min_us, motors_cfg[i].trim_max_us)
            }),
            servos: core::array::from_fn(|i| {
                servos_cfg[i]
                    .mid_point_us
                    .wrapping_add_signed(throw.dot(&servos_cfg[i].mixer_gain) as i16)
                    .clamp(servos_cfg[i].trim_min_us, servos_cfg[i].trim_max_us)
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
        const I_TERM_MAX_ERROR: f32 = 165.0;

        let error = target_rate - current_rate;

        let ff_term = self.cfg.kff * target_rate;
        let p_term = self.cfg.kp * error;

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
