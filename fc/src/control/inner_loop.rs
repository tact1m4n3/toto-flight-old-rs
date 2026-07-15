use embassy_time::{Duration, Ticker};

use crate::sensors;

pub async fn task() {
    const PERIOD: Duration = Duration::from_hz(100);

    let mut ticker = Ticker::every(PERIOD);
    loop {
        ticker.next().await;

        let Some(measured_rate) = sensors::imu::DATA.get() else {
            continue;
        };
    }
}
