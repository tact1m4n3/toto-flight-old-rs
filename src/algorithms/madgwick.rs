#![allow(non_snake_case)]

use nalgebra::{Matrix4, Matrix6, Quaternion, UnitQuaternion, Vector2, Vector3, Vector4, Vector6};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Madgwick {
    beta: f32,
    pub quat: UnitQuaternion<f32>,
}

impl Default for Madgwick {
    fn default() -> Self {
        Self::new(0.1)
    }
}

impl Madgwick {
    pub fn new(beta: f32) -> Self {
        Self {
            beta,
            quat: UnitQuaternion::identity(),
        }
    }

    pub fn update_gyro(&mut self, gyro: &Vector3<f32>, dt: f32) {
        let q = self.quat.as_ref();
        let qDot = q * Quaternion::from_parts(0.0, *gyro) * 0.5;
        self.quat = UnitQuaternion::from_quaternion(q + qDot * dt);
    }

    pub fn update_imu(&mut self, gyro: &Vector3<f32>, accel: &Vector3<f32>, dt: f32) {
        let q = self.quat.as_ref();

        let accel = match accel.try_normalize(0.0) {
            Some(a) => a,
            None => {
                self.update_gyro(gyro, dt);
                return;
            }
        };

        #[rustfmt::skip]
        let F = Vector4::new(
            2.0*(      q[0]*q[2] + q[3]*q[1]) - accel[0],
            2.0*(      q[3]*q[0] - q[1]*q[2]) - accel[1],
            2.0*(0.5 - q[0]*q[0] - q[1]*q[1]) + accel[2],
            0.0,
        );

        #[rustfmt::skip]
        let J_t = Matrix4::new(
             2.0*q[2],  2.0*q[3], -4.0*q[0],  0.0,
             2.0*q[1],  2.0*q[0], -4.0*q[1],  0.0,
             2.0*q[0], -2.0*q[1],        0.0,  0.0,
             2.0*q[3], -2.0*q[2],        0.0,  0.0,
        );

        let Some(step) = (J_t * F).try_normalize(0.0) else {
            return self.update_gyro(gyro, dt);
        };

        let qDot = (q * Quaternion::from_parts(0.0, *gyro)) * 0.5
            - Quaternion::new(step[0], step[1], step[2], step[3]) * self.beta;

        self.quat = UnitQuaternion::from_quaternion(q + qDot * dt);
    }

    pub fn update(
        &mut self,
        gyro: &Vector3<f32>,
        accel: &Vector3<f32>,
        mag: &Vector3<f32>,
        dt: f32,
    ) {
        let q = self.quat.as_ref();

        let accel = match accel.try_normalize(0.0) {
            Some(a) => a,
            None => {
                self.update_gyro(gyro, dt);
                return;
            }
        };

        let mag = match mag.try_normalize(0.0) {
            Some(m) => m,
            None => {
                self.update_gyro(gyro, dt);
                return;
            }
        };

        let h = q * (Quaternion::from_parts(0.0, mag) * q.conjugate());
        let b = Quaternion::new(0.0, Vector2::new(h[0], h[1]).norm(), 0.0, h[2]);

        #[rustfmt::skip]
        let F = Vector6::new(
            2.0*(      q[0]*q[2] + q[3]*q[1]) - accel[0],
            2.0*(      q[3]*q[0] - q[1]*q[2]) - accel[1],
            2.0*(0.5 - q[0]*q[0] - q[1]*q[1]) + accel[2],
            2.0*b[0]*(0.5 - q[1]*q[1] - q[2]*q[2]) + 2.0*b[2]*(q[0]*q[2] + q[3]*q[1]) - mag[0],
            2.0*b[0]*(q[0]*q[1] - q[3]*q[2])        + 2.0*b[2]*(q[3]*q[0] - q[1]*q[2]) - mag[1],
            2.0*b[0]*(q[3]*q[1] + q[0]*q[2])        + 2.0*b[2]*(0.5 - q[0]*q[0] - q[1]*q[1]) - mag[2],
        );

        #[rustfmt::skip]
        let J_t = Matrix6::new(
             2.0*q[2],  2.0*q[3], -4.0*q[0],                 2.0*b[2]*q[2],  2.0*b[0]*q[3]-2.0*b[2]*q[1], 2.0*b[0]*q[2],
             2.0*q[1],  2.0*q[0], -4.0*q[1], -4.0*b[0]*q[1]-2.0*b[2]*q[3], -2.0*b[0]*q[2]+2.0*b[2]*q[0], 2.0*b[0]*q[1]+4.0*b[2]*q[3],  // intentional sign
             2.0*q[0], -2.0*q[1],       0.0, -4.0*b[0]*q[2]+2.0*b[2]*q[0],  2.0*b[0]*q[1]+2.0*b[2]*q[3], 2.0*b[0]*q[0]-4.0*b[2]*q[1],
             2.0*q[3], -2.0*q[2],       0.0, -4.0*b[0]*q[3]-2.0*b[2]*q[1], -2.0*b[0]*q[0]-2.0*b[2]*q[2], 2.0*b[0]*q[3],
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        );

        let Some(step) = (J_t * F).try_normalize(0.0) else {
            return self.update_gyro(gyro, dt);
        };

        let qDot = q * Quaternion::from_parts(0.0, *gyro) * 0.5
            - Quaternion::new(step[0], step[1], step[2], step[3]) * self.beta;

        self.quat = UnitQuaternion::from_quaternion(q + qDot * dt);
    }
}
