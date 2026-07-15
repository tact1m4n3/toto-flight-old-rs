use ahrs::Ahrs;
use embassy_time::{Duration, Ticker};
use nalgebra::{UnitQuaternion, Vector3};
use uom::si::{acceleration::standard_gravity, angular_velocity::radian_per_second};

use crate::{
    RawMutex,
    utils::{DurationExt, SyncWatch},
};

pub static ATTITUDE: SyncWatch<RawMutex, UnitQuaternion<f32>> = SyncWatch::new();

pub async fn task() {
    const PERIOD: Duration = Duration::from_hz(100);

    let mut filter = ahrs::Madgwick::new(PERIOD.dt_secs_f32(), 0.1);

    let mut ticker = Ticker::every(PERIOD);
    loop {
        ticker.next().await;

        let Some(imu) = super::imu::DATA.get() else {
            continue;
        };
        let gyro = imu.gyro.get::<radian_per_second>();
        let accel = imu.accel.get::<standard_gravity>();
        let gyro_nwu = Vector3::new(gyro.x, -gyro.y, -gyro.z);
        let accel_nwu = Vector3::new(accel.x, -accel.y, -accel.z);

        // TODO: If magnetometer reading is available, call update
        let quat = match filter.update_imu(&gyro_nwu, &accel_nwu) {
            Ok(quat) => quat,
            Err(_) => filter.update_gyro(&gyro_nwu),
        };

        ATTITUDE.set(*quat);
    }
}

#[derive(Clone, Copy)]
pub struct Data {
    pub position: Vector3<f32>,
    pub attitude: UnitQuaternion<f32>,
}
