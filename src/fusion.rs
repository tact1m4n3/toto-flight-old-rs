use ahrs::Ahrs;
use embassy_time::{Duration, Ticker};
use nalgebra::{UnitQuaternion, Vector3};

use crate::{
    RawMutex,
    utils::{DurationExt, NalgebraQuatExt, SyncWatch},
};

pub static ATTITUDE: SyncWatch<RawMutex, UnitQuaternion<f32>> =
    SyncWatch::new(UnitQuaternion::IDENTITY);

pub async fn task() {
    const PERIOD: Duration = Duration::from_hz(100);

    let mut filter = ahrs::Madgwick::new(PERIOD.dt_f32(), 0.1);

    let mut ticker = Ticker::every(PERIOD);
    loop {
        ticker.next().await;

        let gyro = super::imu::GYRO.get();
        let accel = super::imu::ACCEL.get();
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
