use embassy_time::{Duration, Ticker};
use nalgebra::UnitQuaternion;

use crate::{
    RawMutex,
    algorithms::madgwick::Madgwick,
    utils::{NalgebraQuatExt, SyncWatch},
};

pub static ATTITUDE: SyncWatch<RawMutex, UnitQuaternion<f32>> =
    SyncWatch::new(UnitQuaternion::IDENTITY);

pub async fn task() {
    const FREQ_HZ: u64 = 100;
    const DT: f32 = 1.0 / FREQ_HZ as f32;

    let mut filter = Madgwick::new(0.1);

    let mut ticker = Ticker::every(Duration::from_hz(FREQ_HZ));
    loop {
        ticker.next().await;

        let gyro = super::imu::GYRO.get();
        let accel = super::imu::ACCEL.get();

        // TODO: If magnetometer reading is available, call update
        filter.update_imu(&gyro, &accel, DT);

        ATTITUDE.set(filter.quat);
    }
}
