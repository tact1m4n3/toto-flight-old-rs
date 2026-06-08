use core::cell::RefCell;

use ahrs::Ahrs;
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::{Duration, Ticker};
use nalgebra::{UnitQuaternion, Vector3};

use crate::{
    RawMutex,
    utils::{DurationExt, NalgebraQuatExt, NalgebraVec3Ext},
};

pub static DATA: Mutex<RawMutex, RefCell<Data>> = Mutex::new(RefCell::new(Data {
    position: Vector3::ZEROS,
    attitude: UnitQuaternion::IDENTITY,
}));

pub async fn task() {
    const PERIOD: Duration = Duration::from_hz(100);

    let mut filter = ahrs::Madgwick::new(PERIOD.dt_f32(), 0.1);

    let mut ticker = Ticker::every(PERIOD);
    loop {
        ticker.next().await;

        let (gyro, accel) = super::imu::DATA.lock(|state| {
            let state = state.borrow();
            (state.gyro, state.accel)
        });
        let gyro_nwu = Vector3::new(gyro.x, -gyro.y, -gyro.z);
        let accel_nwu = Vector3::new(accel.x, -accel.y, -accel.z);

        // TODO: If magnetometer reading is available, call update
        let quat = match filter.update_imu(&gyro_nwu, &accel_nwu) {
            Ok(quat) => quat,
            Err(_) => filter.update_gyro(&gyro_nwu),
        };

        DATA.lock(|state| {
            state.borrow_mut().attitude = *quat;
        });
    }
}

#[derive(Clone, Copy)]
pub struct Data {
    pub position: Vector3<f32>,
    pub attitude: UnitQuaternion<f32>,
}
