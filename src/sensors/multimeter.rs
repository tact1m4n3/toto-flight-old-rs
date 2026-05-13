use embassy_time::{Duration, Ticker};
use ina226_tp::{INA226, InaAverage, InaMode, InaVbusct, InaVshct, MaskEnable, SlaveAddressing};

use crate::{SharedI2cDevice, SystemSpawner, utils::SyncWatch};

const SHUNT_RESISTANCE: f64 = 0.02;
const MAX_CURRENT: f64 = 50.0;

pub static DATA: SyncWatch<Data> = SyncWatch::new(Data {
    voltage: 0.0,
    current: 0.0,
    capacity_used: 0.0,
});

#[derive(Clone, Copy)]
pub struct Data {
    /// in V
    pub voltage: f32,
    /// in A
    pub current: f32,
    /// in mAh
    pub capacity_used: f32,
}

// TODO: Maybe we should use f32 instead of f64 (the driver uses this).

pub async fn init(
    spawner: SystemSpawner,
    i2c_dev: SharedI2cDevice,
) -> Result<(), ina226_tp::Error> {
    let mut ina226 = INA226::new(None);
    let mut ina226 = ina226
        .set_ina_address(ina226_tp::convert_slave_address(
            SlaveAddressing::Gnd,
            SlaveAddressing::Gnd,
        ))
        .initialize(i2c_dev)
        .await?;

    ina226
        .set_ina_mode(InaMode::ShuntAndBusContinuous)
        .set_ina_calibration(SHUNT_RESISTANCE, MAX_CURRENT)
        .set_ina_vscht(InaVshct::_2_116_ms)
        .set_ina_vbusct(InaVbusct::_2_116_ms)
        .set_ina_average(InaAverage::_4)
        .set_ina_masks(MaskEnable::AlertLatchEnable)
        .commit()
        .await; // TODO: Error is ignored here by the driver.

    spawner.default.spawn(task(ina226).unwrap());

    Ok(())
}

const FREQ_HZ: u64 = 10;
const DT: f64 = 1.0 / FREQ_HZ as f64;

#[embassy_executor::task]
async fn task(mut ina226: INA226<SharedI2cDevice, ina226_tp::Operational>) {
    let mut ticker = Ticker::every(Duration::from_hz(10));

    let mut prev_current = 0.0;
    let mut total_current = 0.0;

    loop {
        ticker.next().await;

        let instant_voltage = ina226.read_voltage().await;
        let instant_current = ina226.read_current().await;

        // trapezoidal integration
        total_current += ((prev_current + instant_current) / 2.0) * DT * 3600.0;
        prev_current = instant_current;

        critical_section::with(|cs| {
            DATA.set(
                cs,
                Data {
                    voltage: instant_voltage as f32,
                    current: instant_current as f32,
                    capacity_used: total_current as f32,
                },
            );
        });
    }
}
