#![no_std]
#![allow(async_fn_in_trait)]

pub(crate) type RawMutex = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

pub mod battery;
pub mod config;
pub mod drivers;
pub mod imu;
pub mod navigation;
pub mod receiver;
pub mod utils;
