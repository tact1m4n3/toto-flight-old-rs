use nalgebra::Vector3;

pub mod lsm6dsv;

#[derive(Debug, Clone, Copy, Default)]
pub struct Data {
    /// Angular rate in rad/s
    pub gyro: Vector3<f32>,
    /// Linear acceleration in m/s^2
    pub accel: Vector3<f32>,
}

pub trait ImuSensor {
    type Error;
    async fn read(&mut self) -> Result<Data, <Self as ImuSensor>::Error>;
}
