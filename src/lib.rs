#![no_std]
#![allow(async_fn_in_trait)]
#![allow(clippy::new_without_default)]
#![recursion_limit = "256"]

pub(crate) type RawMutex = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

pub const MAX_MOTOR_OUTPUTS: usize = 2;
pub const MAX_SERVO_OUTPUTS: usize = 5;

#[macro_use]
pub mod log;

pub mod battery;
pub mod config;
pub mod control;
#[cfg(feature = "cpu-usage")]
pub mod cpu_usage;
pub mod drivers;
pub mod fusion;
pub mod gps;
pub mod imu;
pub mod receiver;
pub mod usb_interface;
pub mod utils;
