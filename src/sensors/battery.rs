use embassy_time::{Duration, Ticker};

use crate::{drivers::battery::BatterySensor, utils::DurationExt};

pub async fn task<B: BatterySensor>(mut sensor: B) {
    const PERIOD: Duration = Duration::from_hz(10);

    let mut prev_current: f64 = 0.0;
    let mut capacity_used: f64 = 0.0;

    let mut ticker = Ticker::every(PERIOD);
    loop {
        ticker.next().await;

        let Ok(data) = sensor.read().await else {
            warn!("failed to read battery sensor");
            continue;
        };

        // trapezoidal integration
        capacity_used += data.current.midpoint(prev_current) * PERIOD.dt_f64() / 3600.0;
        prev_current = data.current;

        let data = Data {
            voltage: data.voltage,
            current: data.current,
            capacity_used,
        };
        super::DATA.lock(|cs| cs.borrow_mut().battery = data);
    }
}

#[derive(Clone, Copy)]
pub struct Data {
    /// in mV
    pub voltage: f64,
    /// in mA
    pub current: f64,
    /// in mAh
    pub capacity_used: f64,
}

impl Data {
    pub const DEFAULT: Data = Data {
        voltage: 0.0,
        current: 0.0,
        capacity_used: 0.0,
    };
}
