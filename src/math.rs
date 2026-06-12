use nalgebra::Vector3;
use sguaba::system;
use uom::si::{angular_velocity, f32::AngularVelocity};

system!(struct BoardFrame using FRD);
system!(struct BodyFrame using FRD);

pub struct AngularVelocity3D {
    pub roll: AngularVelocity,
    pub pitch: AngularVelocity,
    pub yaw: AngularVelocity,
}

impl AngularVelocity3D {
    pub fn get<U: angular_velocity::Unit + uom::Conversion<f32, T = f32>>(&self) -> Vector3<f32> {
        Vector3::new(
            self.roll.get::<U>(),
            self.pitch.get::<U>(),
            self.pitch.get::<U>(),
        )
    }
}
