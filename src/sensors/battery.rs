use embassy_time::{Duration, Ticker};
use uom::{
    ConstZero,
    si::f32::{ElectricCharge, ElectricCurrent, ElectricPotential},
};

use crate::{
    RawMutex,
    drivers::battery::BatterySensor,
    utils::{DurationExt, SyncWatch},
};

pub static DATA: SyncWatch<RawMutex, Data> = SyncWatch::new(Data {
    voltage: ElectricPotential::ZERO,
    current: ElectricCurrent::ZERO,
    capacity_used: ElectricCharge::ZERO,
});

pub async fn task<B: BatterySensor>(mut sensor: B) {
    const PERIOD: Duration = Duration::from_hz(10);

    let mut prev_current = ElectricCurrent::ZERO;
    let mut capacity_used = ElectricCharge::ZERO;

    let mut ticker = Ticker::every(PERIOD);
    loop {
        ticker.next().await;

        let Ok(data) = sensor.read().await else {
            warn!("failed to read battery sensor");
            continue;
        };

        // trapezoidal integration
        capacity_used += (prev_current + data.current) / 2.0 * PERIOD.dt();
        prev_current = data.current;

        let data = Data {
            voltage: data.voltage,
            current: data.current,
            capacity_used,
        };
        DATA.set(data);
    }
}

#[derive(Clone, Copy)]
pub struct Data {
    pub voltage: ElectricPotential,
    pub current: ElectricCurrent,
    pub capacity_used: ElectricCharge,
}
