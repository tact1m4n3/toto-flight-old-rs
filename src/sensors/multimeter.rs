use embassy_time::{Duration, Ticker};
use ina226_tp::{INA226, InaAverage, InaMode, InaVbusct, InaVshct, MaskEnable, SlaveAddressing};

use crate::{SharedI2cDevice, SystemSpawner, mutex_cells};

const SHUNT_RESISTANCE: f64 = 0.02;
const MAX_CURRENT: f64 = 50.0;

// TODO: Maybe we should use f32 instead of f64 (the driver uses this).

mutex_cells! {
    VOLTAGE: f64 = 0.0;
    CURRENT: f64 = 0.0;
    CURRENT_DRAW: f64 = 0.0;
}

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

    spawner.low_prio.spawn(task(ina226).unwrap());

    Ok(())
}

const FREQ_HZ: u64 = 10;
const DT: f64 = 1.0 / FREQ_HZ as f64;

#[embassy_executor::task]
async fn task(mut ina226: INA226<SharedI2cDevice, ina226_tp::Operational>) {
    let mut ticker = Ticker::every(Duration::from_hz(10));

    loop {
        ticker.next().await;

        let instant_voltage = ina226.read_voltage().await;
        let instant_current = ina226.read_current().await;

        let (prev_current, prev_current_consumption) =
            critical_section::with(|cs| (CURRENT.borrow(cs).get(), CURRENT_DRAW.borrow(cs).get()));

        // trapezoidal integration
        let current_consumption =
            prev_current_consumption + (prev_current + instant_current) * DT / 2.0;

        critical_section::with(|cs| {
            VOLTAGE.borrow(cs).set(instant_voltage);
            CURRENT.borrow(cs).set(instant_current);
            CURRENT_DRAW.borrow(cs).set(current_consumption);
        });

        // TODO: log data to blackbox
    }
}
