use core::{
    ops::RangeInclusive,
    sync::atomic::{AtomicU32, Ordering},
};

use embassy_time::{Duration, Instant, Ticker};
use nalgebra::Vector3;

use crate::{
    SystemSpawner, black_box, pid_loop, receiver,
    utils::{SyncWatch, US_TO_NEG_ONE_ONE},
};

pub static COMMAND: SyncWatch<Command> = SyncWatch::new(Command::Idle);

#[derive(Clone, Copy)]
pub enum Command {
    Idle,
    Manual { throttle: u16, throw: Vector3<f32> },
    Rate { throttle: u16, rate: Vector3<f32> },
}

pub fn init(spawner: SystemSpawner) {
    spawner.high_prio.spawn(nav_loop().unwrap());
}

const FREQ_HZ: u64 = 200;
const FREQ_DT: f32 = 1.0 / FREQ_HZ as f32;

#[embassy_executor::task]
async fn nav_loop() {
    let mut failsafe = true;
    let mut is_armed = false;

    let mut arm_condition = ChannelCondition {
        channel_idx: 4,
        range: 1500..=2000,
        state: true,
    };
    let mut flight_mode_manager = FlightModeManager {
        rate_mode_condition: ChannelCondition {
            channel_idx: 5,
            range: 1300..=1700,
            state: false,
        },
    };
    let mut servo_mid_point_calibration_condition = ChannelCondition {
        channel_idx: 6,
        range: 1500..=2000,
        state: true,
    };

    let mut ticker = Ticker::every(Duration::from_hz(FREQ_HZ));
    loop {
        ticker.next().await;

        let (channels, channels_timestamp) = receiver::CHANNELS.get_with_timestamp();

        let mut arm_request: Option<bool> = None;

        // TODO: failsafe logic should be abstracted and implemented as a state machine a more
        // complicated algorithm is implemented
        if !failsafe && Instant::now() - channels_timestamp > Duration::from_millis(1000) {
            // disarm on failsafe
            arm_request = Some(false);
            failsafe = true;
        } else {
            failsafe = false;
        }

        if let Some(arm_request) = arm_condition.update(&channels).or(arm_request) {
            match arm_request {
                true if !is_armed && !failsafe && ArmBlocker::arm().is_ok() => {
                    is_armed = true;
                    black_box::begin_new_file();
                    black_box::set_recording_enabled(true);
                }
                false => {
                    ArmBlocker::disarm();
                    is_armed = false;
                    black_box::set_recording_enabled(false);
                }
                _ => (),
            }
        }

        let flight_mode = flight_mode_manager.update(&channels);

        if let Some(enable) = servo_mid_point_calibration_condition.update(&channels) {
            pid_loop::set_mid_point_calibration_enabled(enable);
        }

        let command: Command = if is_armed {
            match flight_mode {
                FlightMode::Manual => Command::Manual {
                    throttle: channels[2],
                    throw: Vector3::new(
                        channels[0] as f32 - 1500.0,
                        channels[1] as f32 - 1500.0,
                        channels[3] as f32 - 1500.0,
                    ),
                },
                FlightMode::Rate => Command::Rate {
                    throttle: channels[2],
                    rate: Vector3::new(
                        // TODO: apply configurable rates
                        US_TO_NEG_ONE_ONE.map(channels[0] as f32) * 45.0,
                        US_TO_NEG_ONE_ONE.map(channels[1] as f32) * 45.0,
                        US_TO_NEG_ONE_ONE.map(channels[3] as f32) * 45.0,
                    ),
                },
            }
        } else {
            Command::Idle
        };

        COMMAND.set(command);
    }
}

enum FlightMode {
    Manual,
    Rate,
}

struct FlightModeManager {
    rate_mode_condition: ChannelCondition,
}

impl FlightModeManager {
    pub fn update(&mut self, channels: &[u16; 16]) -> FlightMode {
        self.rate_mode_condition.update(channels);
        match self.rate_mode_condition.state {
            false => FlightMode::Manual,
            true => FlightMode::Rate,
        }
    }
}

struct ChannelCondition {
    channel_idx: u8,
    range: RangeInclusive<u16>,
    state: bool,
}

impl ChannelCondition {
    fn update(&mut self, channels: &[u16; 16]) -> Option<bool> {
        let value = channels[self.channel_idx as usize];
        match self.state {
            false if self.range.contains(&value) => {
                self.state = true;
                Some(self.state)
            }
            true if !self.range.contains(&value) => {
                self.state = false;
                Some(self.state)
            }
            _ => None,
        }
    }
}

static ARM_BLOCK: AtomicU32 = AtomicU32::new(0);
const ARMED_FLAG_MASK: u32 = 1 << 31;

pub struct ArmBlocker(());

impl ArmBlocker {
    pub fn new() -> Result<Self, ()> {
        let state = ARM_BLOCK.fetch_add(1, Ordering::Relaxed);
        assert!(state & !ARMED_FLAG_MASK < 0xFFF_FFFF);
        if state & ARMED_FLAG_MASK != 0 {
            Err(())
        } else {
            Ok(Self(()))
        }
    }

    fn arm() -> Result<(), u32> {
        loop {
            let state = ARM_BLOCK.load(Ordering::Relaxed);
            // if already armed
            if state & ARMED_FLAG_MASK != 0 {
                break Ok(());
            }
            // if there are any arm blockers
            if state & !ARMED_FLAG_MASK != 0 {
                break Err(state & !ARMED_FLAG_MASK);
            }
            // this should never fail on a single core
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
        // if armed, reset the field
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

// #[derive(Clone, Copy)]
// enum Control {
//     Idle,
//     Rate {
//         /// between 0 and 1
//         throttle: f32,
//         /// in deg/s
//         target_rate: Vector3<f32>,
//     },
//     Angle {
//         /// between 0 and 1
//         throttle: f32,
//         target_attitude: UnitQuaternion<f32>,
//     },
//     Assisted {
//         /// in m/s
//         speed: f32,
//         /// in m/s
//         climb_rate: f32,
//         /// degrees relative to north
//         heading: f32,
//     },
// }
