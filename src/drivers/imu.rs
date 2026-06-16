use crate::math::{Acceleration3D, AngularVelocity3D};

pub mod lsm6dsv;

#[derive(Clone, Copy)]
pub struct Data {
    /// Raw gyro reading, NED, in the sensor's frame
    pub gyro: AngularVelocity3D,
    /// Raw acceleration reading, NED, in the sensor's frame
    pub accel: Acceleration3D,
}

pub trait ImuSensor {
    type Error;
    async fn read(&mut self) -> Result<Data, <Self as ImuSensor>::Error>;
}
