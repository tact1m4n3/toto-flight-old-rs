pub mod ina226;

pub struct Data {
    /// Voltage in mV
    pub voltage: f64,
    /// Current in mA
    pub current: f64,
}

pub trait BatterySensor {
    type Error;
    async fn read(&mut self) -> Result<Data, <Self as BatterySensor>::Error>;
}
