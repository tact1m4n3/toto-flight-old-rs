use ahrs::Ahrs;
use embassy_time::{Duration, Ticker};
use nalgebra::UnitQuaternion;

use crate::{
    SystemSpawner,
    sensors::imu,
    utils::{NalgebraQuatExt, SyncWatch},
};

// TODO: to be renamed to sensor fusion once we implement eskf

static ATTITUDE: SyncWatch<UnitQuaternion<f32>> = SyncWatch::new(UnitQuaternion::IDENTITY);

pub fn init(spawner: SystemSpawner) {
    spawner.high_prio.spawn(task().unwrap());
}

const FREQ_HZ: u64 = 1000;
const FREQ_DT: f32 = 1.0 / FREQ_HZ as f32;

#[embassy_executor::task]
async fn task() {
    let mut madgwick = ahrs::Madgwick::new(FREQ_DT, 0.1);

    let mut ticker: Ticker = Ticker::every(Duration::from_hz(FREQ_HZ));
    loop {
        ticker.next().await;

        let gyro = imu::GYRO.get();
        let accel = imu::ACCEL.get();

        let attitude = match madgwick.update_imu(&gyro, &accel) {
            Ok(quat) => quat,
            Err(_) => madgwick.update_gyro(&gyro),
        };

        ATTITUDE.set(attitude.clone());
    }
}
