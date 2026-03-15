use core::ops::Range;

use nalgebra::{Quaternion, UnitQuaternion, Vector3};

pub const US_TO_ZERO_ONE: Mapping = Mapping {
    from: 1000.0..2000.0,
    to: 0.0..1.0,
};

pub const US_TO_NEG_ONE_ONE: Mapping = Mapping {
    from: 1000.0..2000.0,
    to: -1.0..1.0,
};

pub const ZERO_ONE_TO_US: Mapping = Mapping {
    from: 0.0..1.0,
    to: 1000.0..2000.0,
};

pub const NEG_ONE_ONE_TO_US: Mapping = Mapping {
    from: -1.0..1.0,
    to: 1000.0..2000.0,
};

pub struct Mapping {
    pub from: Range<f32>,
    pub to: Range<f32>,
}

impl Mapping {
    pub fn map(&self, value: f32) -> f32 {
        self.to.start
            + (value - self.from.start) / (self.from.end - self.from.start)
                * (self.to.end - self.to.start)
    }
}

pub trait NalgebraVec3Ext {
    const ZERO: Self;
}

impl NalgebraVec3Ext for Vector3<f32> {
    const ZERO: Self = Vector3::new(0.0, 0.0, 0.0);
}

pub trait NalgebraQuatExt {
    const IDENTITY: Self;
}

impl NalgebraQuatExt for UnitQuaternion<f32> {
    const IDENTITY: Self = UnitQuaternion::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0));
}

#[macro_export]
macro_rules! mutex_cells {
    ($($name:ident: $ty:ty = $init:expr);* $(;)?) => {
        $(
            pub static $name: embassy_sync::blocking_mutex::Mutex<
                embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, core::cell::Cell<$ty>
            > = embassy_sync::blocking_mutex::Mutex::new(core::cell::Cell::new($init));
        )*
    };
}
