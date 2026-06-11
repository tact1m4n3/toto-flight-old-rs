pub mod ina226;

pub struct Data {
    /// Voltage in V
    pub voltage: f64,
    /// Current in A
    pub current: f64,
}

pub trait BatterySensor {
    type Error;
    async fn read(&mut self) -> Result<Data, <Self as BatterySensor>::Error>;
}
