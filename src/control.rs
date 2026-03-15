use core::cell::Cell;

use embassy_sync::{
    blocking_mutex::{Mutex, raw::CriticalSectionRawMutex},
    channel::Channel,
};
use embassy_time::{Duration, Instant, Ticker};
use nalgebra::{UnitQuaternion, Vector3};

use crate::{
    SystemSpawner, imu, receiver,
    utils::{US_TO_NEG_ONE_ONE, US_TO_ZERO_ONE},
};

pub static TARGET: Mutex<CriticalSectionRawMutex, Cell<Option<Target>>> =
    Mutex::new(Cell::new(None));

pub fn init(spawner: SystemSpawner) {
    // TODO: make another priority for this
    spawner.high_prio.spawn(task().unwrap());
}

#[embassy_executor::task]
async fn task() {
    let mut failsafe = true;
    let mut is_armed = false;
    let mut arm_switch = BinarySwitch {
        state: true,
        on_state_min: 1500,
    };

    let mut ticker = Ticker::every(Duration::from_hz(200));

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

const SYSTEM_AUTHORITY_TOKEN: SystemAuthorityToken = SystemAuthorityToken { _priv: () };

pub struct SystemAuthorityToken {
    _priv: (),
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
