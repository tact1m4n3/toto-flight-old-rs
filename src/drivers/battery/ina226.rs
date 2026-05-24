use embedded_hal_async::i2c::I2c;
use ina226_tp::{
    INA226, InaAverage, InaMode, InaVbusct, InaVshct, MaskEnable, Operational, SlaveAddressing,
};

const SHUNT_RESISTANCE: f64 = 0.02;
const MAX_CURRENT: f64 = 50.0;

pub struct Ina226<I2C: I2c> {
    driver: INA226<I2C, Operational>,
}

impl<I2C: I2c> Ina226<I2C> {
    pub async fn new(i2c: I2C) -> Result<Self, ina226_tp::Error> {
        let mut ina226 = INA226::new(None);
        let mut ina226 = ina226
            .set_ina_address(ina226_tp::convert_slave_address(
                SlaveAddressing::Gnd,
                SlaveAddressing::Gnd,
            ))
            .initialize(i2c)
            .await?;
        ina226
            .set_ina_mode(InaMode::ShuntAndBusContinuous)
            .set_ina_calibration(SHUNT_RESISTANCE, MAX_CURRENT)
            // TODO: Chosen to have one sample ready in roughly every 10ms. Should we make this
            // configurable? The battery task currently samples at this fixed rate.
            .set_ina_vscht(InaVshct::_2_116_ms)
            .set_ina_vbusct(InaVbusct::_2_116_ms)
            .set_ina_average(InaAverage::_4)
            .set_ina_masks(MaskEnable::AlertLatchEnable)
            .commit()
            .await; // TODO: Error is ignored here by the driver.
        Ok(Self { driver: ina226 })
    }
}

impl<I2C: I2c> super::BatterySensor for Ina226<I2C> {
    type Error = ();

    async fn read(&mut self) -> Result<super::Data, Self::Error> {
        let voltage_in_mv = self.driver.read_voltage().await; // TODO: Error is ignored here by the driver.
        let current_in_ma = self.driver.read_current().await;

        Ok(super::Data {
            voltage: voltage_in_mv,
            current: current_in_ma,
        })
    }
}
