use embassy_time::{Duration, Instant, Ticker};
use nalgebra::{UnitQuaternion, Vector3};

use crate::{
    SystemSpawner, motor_output, mutex_cells, receiver,
    sensors::imu,
    utils::{NEG_ONE_ONE_TO_US, US_TO_NEG_ONE_ONE, US_TO_ZERO_ONE, ZERO_ONE_TO_US},
};

pub fn init(spawner: SystemSpawner) {
    spawner.high_prio.spawn(nav_loop().unwrap());
    spawner.realtime.spawn(pid_loop().unwrap());
}

const NAV_LOOP_FREQ_HZ: u64 = 200;
const NAV_LOOP_FREQ_DT: f32 = 1.0 / NAV_LOOP_FREQ_HZ as f32;

const PID_LOOP_FREQ_HZ: u64 = 500;
const PID_LOOP_FREQ_DT: f32 = 1.0 / PID_LOOP_FREQ_HZ as f32;

mutex_cells! {
    TARGET: Option<Target> = None;
}

#[embassy_executor::task]
async fn nav_loop() {
    let mut failsafe = true;
    let mut is_armed = false;
    let mut arm_switch = BinarySwitch {
        state: true,
        on_state_min: 1500,
    };

    let mut ticker = Ticker::every(Duration::from_hz(NAV_LOOP_FREQ_HZ));

    loop {
        ticker.next().await;

        let (channels, channels_timestamp) =
            critical_section::with(|cs| receiver::CHANNELS.borrow(cs).get());

        if !failsafe && Instant::now() - channels_timestamp > Duration::from_millis(1000) {
            // disarm on failsafe
            is_armed = false;
            failsafe = true;
        } else {
            failsafe = false;
        }

        if let Some(arm_request) = arm_switch.update(channels[4]) {
            if arm_request && !is_armed && !failsafe {
                // TODO: check arm blockers
                is_armed = true;
            } else if !arm_request {
                is_armed = false;
            }
        }

        let target: Option<Target> = if !is_armed {
            let control = Control::Rate {
                throttle: US_TO_ZERO_ONE.map(channels[2] as f32),
                target_rate: Vector3::new(
                    US_TO_NEG_ONE_ONE.map(channels[0] as f32),
                    US_TO_NEG_ONE_ONE.map(channels[1] as f32),
                    US_TO_NEG_ONE_ONE.map(channels[3] as f32),
                ),
            };

            match control {
                Control::Idle => None,
                Control::Rate {
                    throttle,
                    target_rate,
                } => Some(Target {
                    throttle,
                    rate: target_rate,
                }),
                Control::Angle {
                    throttle: _,
                    target_attitude: _,
                } => todo!(),
                Control::Assisted {
                    speed: _,
                    climb_rate: _,
                    heading: _,
                } => todo!(),
            }
        } else {
            None
        };

        critical_section::with(|cs| TARGET.borrow(cs).set(target));
    }
}

#[embassy_executor::task]
pub async fn pid_loop() {
    const LEFT_ELEVON_MIXER: Vector3<f32> = Vector3::new(0.5, 0.5, 0.0);
    const RIGHT_ELEVON_MIXER: Vector3<f32> = Vector3::new(-0.5, 0.5, 0.0);

    let mut rate_controller = RateController {
        roll_controller: SingleAxisRateController::new(RatePidSettings {
            kp: 0.1,
            ki: 0.01,
            kff: 0.5,
            i_term_max_error: 10.0,
        }),
        pitch_controller: SingleAxisRateController::new(RatePidSettings {
            kp: 0.1,
            ki: 0.01,
            kff: 10.0,
            i_term_max_error: 10.0,
        }),
        yaw_controller: SingleAxisRateController::new(RatePidSettings {
            kp: 0.1,
            ki: 0.01,
            kff: 10.0,
            i_term_max_error: 10.0,
        }),
    };

    let mut ticker = Ticker::every(Duration::from_hz(PID_LOOP_FREQ_HZ));

    loop {
        ticker.next().await;

        let (gyro, target) =
            critical_section::with(|cs| (imu::GYRO.borrow(cs).get(), TARGET.borrow(cs).get()));

        let Some(target) = target else {
            motor_output::OUTPUT.signal(motor_output::Output::IDLE);
            continue;
        };

        let pids = rate_controller.update(&target.rate, &gyro, PID_LOOP_FREQ_DT);

        motor_output::OUTPUT.signal(motor_output::Output {
            throttle: ZERO_ONE_TO_US.map(target.throttle) as u16,
            left_elevon: NEG_ONE_ONE_TO_US.map(LEFT_ELEVON_MIXER.dot(&pids).clamp(-1.0, 1.0))
                as u16,
            right_elevon: NEG_ONE_ONE_TO_US.map(RIGHT_ELEVON_MIXER.dot(&pids).clamp(-1.0, 1.0))
                as u16,
        })
    }
}

#[derive(Clone, Copy)]
pub struct Target {
    pub throttle: f32,
    pub rate: Vector3<f32>,
}

#[derive(Clone, Copy)]
enum Control {
    Idle,
    Rate {
        /// between 0 and 1
        throttle: f32,
        /// in deg/s
        target_rate: Vector3<f32>,
    },
    Angle {
        /// between 0 and 1
        throttle: f32,
        target_attitude: UnitQuaternion<f32>,
    },
    Assisted {
        /// in m/s
        speed: f32,
        /// in m/s
        climb_rate: f32,
        /// degrees relative to north
        heading: f32,
    },
}

struct BinarySwitch {
    state: bool,
    on_state_min: u16,
}

impl BinarySwitch {
    pub fn update(&mut self, value: u16) -> Option<bool> {
        match self.state {
            false if value >= self.on_state_min => {
                self.state = true;
                Some(self.state)
            }
            true if value < self.on_state_min => {
                self.state = false;
                Some(self.state)
            }
            _ => None,
        }
    }
}

pub struct RatePidSettings {
    pub kff: f32,
    pub kp: f32,
    pub ki: f32,
    pub i_term_max_error: f32,
}

pub struct SingleAxisRateController {
    settings: RatePidSettings,
    i_term: f32,
}

impl SingleAxisRateController {
    pub fn new(settings: RatePidSettings) -> Self {
        Self {
            settings,
            i_term: 0.0,
        }
    }

    pub fn update(&mut self, target_rate: f32, current_rate: f32, dt: f32) -> f32 {
        let error = target_rate - current_rate;

        let ff_term = self.settings.kff * target_rate;
        let p_term = self.settings.kp * error;
        if error.abs() <= self.settings.i_term_max_error {
            self.i_term += self.settings.ki * error * dt;
        }

        ff_term + p_term + self.i_term
    }
}

pub struct RateController {
    pub roll_controller: SingleAxisRateController,
    pub pitch_controller: SingleAxisRateController,
    pub yaw_controller: SingleAxisRateController,
}

impl RateController {
    pub fn update(&mut self, target: &Vector3<f32>, gyro: &Vector3<f32>, dt: f32) -> Vector3<f32> {
        Vector3::new(
            self.roll_controller.update(target.x, gyro.x, dt),
            self.pitch_controller.update(target.y, gyro.y, dt),
            self.yaw_controller.update(target.z, gyro.z, dt),
        )
    }
}
