#![no_std]
#![allow(async_fn_in_trait)]

pub(crate) type RawMutex = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

pub const MAX_MOTOR_OUTPUTS: usize = 2;
pub const MAX_SERVO_OUTPUTS: usize = 5;

pub mod battery;
pub mod config;
pub mod control;
pub mod drivers;
pub mod gps;
pub mod imu;
pub mod receiver;
pub mod utils;
