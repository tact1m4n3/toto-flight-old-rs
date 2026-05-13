use embassy_time::{Duration, Ticker};
use nalgebra::{UnitQuaternion, Vector3};

use crate::{
    SystemSpawner,
    sensors::imu,
    utils::{NalgebraQuatExt, SyncWatch},
};

pub static ATTITUDE: SyncWatch<UnitQuaternion<f32>> = SyncWatch::new(UnitQuaternion::IDENTITY);

pub fn init(spawner: SystemSpawner) {
    spawner.default.spawn(task().unwrap());
}

const FREQ_HZ: u64 = 1000;
const FREQ_DT: f32 = 1.0 / FREQ_HZ as f32;

#[embassy_executor::task]
async fn task() {
    let mut filter = eskf_rs::NavigationFilter::new()
        .gyr_noise_density(0.1)
        .gyr_bias_random_walk(0.01)
        .acc_noise_density(0.1)
        .acc_noise_density(0.01)
        .with_gravity(Vector3::z() * 9.80665);

    let mut ticker: Ticker = Ticker::every(Duration::from_hz(FREQ_HZ));
    loop {
        ticker.next().await;

        let (gyro, accel) = critical_section::with(|cs| (imu::GYRO.get(cs), imu::ACCEL.get(cs)));

        filter.predict(gyro, accel, FREQ_DT);

        critical_section::with(|cs| {
            ATTITUDE.set(cs, filter.rotation);
        });
    }
}
