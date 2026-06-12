use uom::si::f32::{ElectricCurrent, ElectricPotential};

pub mod ina226;

pub struct Data {
    pub voltage: ElectricPotential,
    pub current: ElectricCurrent,
}

pub trait BatterySensor {
    type Error;
    async fn read(&mut self) -> Result<Data, <Self as BatterySensor>::Error>;
}
