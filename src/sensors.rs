use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex;
use nalgebra::{UnitQuaternion, Vector3};

use crate::{
    RawMutex,
    utils::{NalgebraQuatExt, NalgebraVec3Ext},
};

pub mod battery;
pub mod fusion;
pub mod gps;
pub mod imu;

pub static DATA: Mutex<RawMutex, RefCell<Data>> = Mutex::new(RefCell::new(Data {
    gyro: Vector3::ZEROS,
    accel: Vector3::ZEROS,

    gps: gps::Data {
        latitude: 0.0,
        longitude: 0.0,
        altitude: 0.0,
        h_acc: f32::INFINITY,
        v_acc: f32::INFINITY,
        speed: 0.0,
        heading: 0.0,
        num_sat: 0,
        hdop: f32::INFINITY,
        fix_type: gps::FixType::Undefined,
        vertical_velocity: 0.0,
    },

    battery: battery::Data {
        voltage: 0.0,
        current: 0.0,
        capacity_used: 0.0,
    },

    position: Vector3::ZEROS,
    attitude: UnitQuaternion::IDENTITY,
}));

pub struct Data {
    pub gyro: Vector3<f32>,
    pub accel: Vector3<f32>,

    pub gps: gps::Data,
    pub battery: battery::Data,

    pub position: Vector3<f32>,
    pub attitude: UnitQuaternion<f32>,
}
