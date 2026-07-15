pub mod inner_loop;

use core::cell::RefCell;

use embassy_sync::{blocking_mutex::Mutex, signal::Signal};
use nalgebra::Vector3;

use crate::{RawMutex, math::AngularVelocity3D, utils::SyncWatch};

pub static AUTO_TUNE: Signal<RawMutex, bool> = Signal::new();
pub static THROTTLE: SyncWatch<RawMutex, f32> = SyncWatch::new();
pub static ACTUATOR_COMMAND: SyncWatch<RawMutex, Vector3<f32>> = SyncWatch::new();

#[derive(Clone, Copy)]
pub enum Command {
    Disarmed,
    Manual {
        throttle: f32,
        throw: Vector3<f32>,
    },
    Rate {
        throttle: f32,
        rate: AngularVelocity3D,
    },
    Angle {
        throttle: f32,
        roll_angle: f32,
        pitch_angle: f32,
    },
}

static STATUS: Mutex<RawMutex, RefCell<Status>> = Mutex::new(RefCell::new(Status {
    arm_state: ArmState::ArmFailed,
    failsafe: true,
}));

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
pub struct Status {
    pub arm_state: ArmState,
    pub failsafe: bool,
}

pub fn status() -> Status {
    STATUS.lock(|state| *state.borrow())
}
