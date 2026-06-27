#![no_std]
#![allow(async_fn_in_trait)]
#![allow(clippy::new_without_default)]
#![recursion_limit = "256"]

use embassy_sync::signal::Signal;

pub(crate) type RawMutex = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

pub const MAX_MOTOR_OUTPUTS: usize = 2;
pub const MAX_SERVO_OUTPUTS: usize = 5;

/// Signal to trigger reboot. Logic is expected to be implemented outside of the crate.
pub static REBOOT: Signal<RawMutex, ()> = Signal::new();

#[macro_use]
pub mod log;

pub mod black_box;
pub mod config;
pub mod control;
#[cfg(feature = "cpu-usage")]
pub mod cpu_usage;
pub mod drivers;
pub mod math;
pub mod msp;
pub mod receiver;
pub mod sensors;
pub mod status_led;
pub mod usb_interface;
pub mod utils;
