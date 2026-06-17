use core::marker::PhantomData;
use core::ops::{Add, Div, Mul, Neg, Sub};

use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use uom::ConstZero;
use uom::si::{Dimension, Quantity, SI};
use uom::si::{
    acceleration, angular_velocity,
    f32::{Acceleration, AngularVelocity},
};

pub type Acceleration3D = QuantityVector3<Acceleration>;
pub type AngularVelocity3D = QuantityVector3<AngularVelocity>;

#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct QuantityVector3<Q> {
    inner: Vector3<f32>,
    _quantity: PhantomData<Q>,
}

impl<Q> QuantityVector3<Q> {
    pub const fn from_base_unit(inner: Vector3<f32>) -> Self {
        Self {
            inner,
            _quantity: PhantomData,
        }
    }

    pub const fn as_base_unit(&self) -> Vector3<f32> {
        self.inner
    }

    pub fn transform<F: FnMut(Vector3<f32>) -> Vector3<f32>>(self, mut f: F) -> QuantityVector3<Q> {
        QuantityVector3 {
            inner: f(self.inner),
            _quantity: PhantomData,
        }
    }
}

impl Acceleration3D {
    pub fn new<U: acceleration::Unit + uom::Conversion<f32, T = f32>>(
        x: f32,
        y: f32,
        z: f32,
    ) -> Self {
        Self {
            inner: Vector3::new(
                Acceleration::new::<U>(x).get::<acceleration::meter_per_second_squared>(),
                Acceleration::new::<U>(y).get::<acceleration::meter_per_second_squared>(),
                Acceleration::new::<U>(z).get::<acceleration::meter_per_second_squared>(),
            ),
            _quantity: PhantomData,
        }
    }

    pub fn get<U: acceleration::Unit + uom::Conversion<f32, T = f32>>(&self) -> Vector3<f32> {
        Vector3::new(
            Acceleration::new::<acceleration::meter_per_second_squared>(self.inner.x).get::<U>(),
            Acceleration::new::<acceleration::meter_per_second_squared>(self.inner.y).get::<U>(),
            Acceleration::new::<acceleration::meter_per_second_squared>(self.inner.z).get::<U>(),
        )
    }
}

impl AngularVelocity3D {
    pub fn new<U: angular_velocity::Unit + uom::Conversion<f32, T = f32>>(
        x: f32,
        y: f32,
        z: f32,
    ) -> Self {
        Self {
            inner: Vector3::new(
                AngularVelocity::new::<U>(x).get::<angular_velocity::radian_per_second>(),
                AngularVelocity::new::<U>(y).get::<angular_velocity::radian_per_second>(),
                AngularVelocity::new::<U>(z).get::<angular_velocity::radian_per_second>(),
            ),
            _quantity: PhantomData,
        }
    }

    pub fn get<U: angular_velocity::Unit + uom::Conversion<f32, T = f32>>(&self) -> Vector3<f32> {
        Vector3::new(
            AngularVelocity::new::<angular_velocity::radian_per_second>(self.inner.x).get::<U>(),
            AngularVelocity::new::<angular_velocity::radian_per_second>(self.inner.y).get::<U>(),
            AngularVelocity::new::<angular_velocity::radian_per_second>(self.inner.z).get::<U>(),
        )
    }
}

impl<Q> Add for QuantityVector3<Q> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            inner: self.inner + rhs.inner,
            _quantity: PhantomData,
        }
    }
}

impl<Q> Sub for QuantityVector3<Q> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            inner: self.inner - rhs.inner,
            _quantity: PhantomData,
        }
    }
}

impl<Q> Mul<f32> for QuantityVector3<Q> {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        Self {
            inner: self.inner * rhs,
            _quantity: PhantomData,
        }
    }
}

impl<Q> Div<f32> for QuantityVector3<Q> {
    type Output = Self;

    fn div(self, rhs: f32) -> Self::Output {
        Self {
            inner: self.inner / rhs,
            _quantity: PhantomData,
        }
    }
}

impl<Q> Neg for QuantityVector3<Q> {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            inner: -self.inner,
            _quantity: PhantomData,
        }
    }
}

impl<Q> core::ops::AddAssign for QuantityVector3<Q> {
    fn add_assign(&mut self, rhs: Self) {
        self.inner += rhs.inner;
    }
}

impl<Q> core::ops::SubAssign for QuantityVector3<Q> {
    fn sub_assign(&mut self, rhs: Self) {
        self.inner -= rhs.inner;
    }
}

pub trait Vector3Ext {
    const ZEROS: Self;
}

impl Vector3Ext for Vector3<f32> {
    const ZEROS: Self = Vector3::new(0.0, 0.0, 0.0);
}

impl<D: ?Sized + Dimension> Vector3Ext for QuantityVector3<Quantity<D, SI<f32>, f32>>
where
    Quantity<D, SI<f32>, f32>: ConstZero,
{
    const ZEROS: Self = Self {
        inner: Vector3::new(
            <Quantity<D, SI<f32>, f32>>::ZERO.value,
            <Quantity<D, SI<f32>, f32>>::ZERO.value,
            <Quantity<D, SI<f32>, f32>>::ZERO.value,
        ),
        _quantity: PhantomData,
    };
}

pub trait QuaternionExt {
    const IDENTITY: Self;
}

impl QuaternionExt for UnitQuaternion<f32> {
    const IDENTITY: Self = UnitQuaternion::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0));
}
