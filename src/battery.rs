use embassy_time::{Duration, Ticker};

use crate::{RawMutex, drivers::battery::BatterySensor, utils::SyncWatch};

pub static DATA: SyncWatch<RawMutex, Data> = SyncWatch::new(Data::DEFAULT);

const FREQ_HZ: u64 = 10;
const DT: f64 = 1.0 / FREQ_HZ as f64;

pub async fn task<B: BatterySensor>(mut sensor: B) {
    let mut prev_current = 0.0;
    let mut capacity_used = 0.0;

    let mut ticker = Ticker::every(Duration::from_hz(FREQ_HZ));
    loop {
        ticker.next().await;

        let data = match sensor.read().await {
            Ok(data) => data,
            Err(_) => {
                log::warn!("failed to read battery sensor");
                continue;
            }
        };

        // trapezoidal integration
        capacity_used += ((prev_current + data.current) / 2.0) * DT * 3600.0;
        prev_current = data.current;

        DATA.set(Data {
            voltage: data.voltage,
            current: data.current,
            capacity_used,
        });
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
