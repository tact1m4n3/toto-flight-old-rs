use ahrs::Ahrs;
use embassy_time::{Duration, Ticker};
use nalgebra::Vector3;

use crate::utils::DurationExt;

pub async fn task() {
    const PERIOD: Duration = Duration::from_hz(100);

    let mut filter = ahrs::Madgwick::new(PERIOD.dt_f32(), 0.1);

    let mut ticker = Ticker::every(PERIOD);
    loop {
        ticker.next().await;

        let (gyro, accel) = super::DATA.lock(|state| {
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

        super::DATA.lock(|state| {
            state.borrow_mut().attitude = *quat;
        });
    }
}
