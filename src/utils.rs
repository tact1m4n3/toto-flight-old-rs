use core::{cell::RefCell, ops::Range};

use embassy_sync::blocking_mutex::{Mutex, raw::RawMutex};
use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};

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
    const ZEROS: Self;
    const ONES: Self;
}

impl NalgebraVec3Ext for Vector3<f32> {
    const ZEROS: Self = Vector3::new(0.0, 0.0, 0.0);
    const ONES: Self = Vector3::new(1.0, 1.0, 1.0);
}

pub trait NalgebraQuatExt {
    const IDENTITY: Self;
}

impl NalgebraQuatExt for UnitQuaternion<f32> {
    const IDENTITY: Self = UnitQuaternion::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0));
}

pub trait NalgebraMatrix3Ext {
    const IDENTITY: Self;
}

impl NalgebraMatrix3Ext for Matrix3<f32> {
    #[rustfmt::skip]
    const IDENTITY: Self = Matrix3::new(1.0, 0.0, 0.0,
                                        0.0, 1.0, 0.0,
                                        0.0, 0.0, 1.0);
}

pub fn pack<const N: usize>(s: &str) -> [u8; N] {
    let mut buf = [0u8; N];
    let b = s.as_bytes();
    let len = b.len().min(N);
    buf[..len].copy_from_slice(&b[..len]);
    buf
}

pub struct GuardedRefCell<T: ?Sized>(RefCell<T>);

impl<T> GuardedRefCell<T> {
    pub const fn new(value: T) -> Self {
        Self(RefCell::new(value))
    }
}

impl<T: ?Sized> GuardedRefCell<T> {
    pub fn guarded_ref<R, F: FnOnce(&T) -> R>(&self, f: F) -> R {
        f(&mut self.0.borrow())
    }

    pub fn guarded_mut<R, F: FnOnce(&mut T) -> R>(&self, f: F) -> R {
        f(&mut self.0.borrow_mut())
    }
}

#[macro_export]
macro_rules! assign_resources {
    {
        $(
            $(#[$outer:meta])*
            $group_name:ident : $group_struct:ident {
                $(
                    $(#[$inner:meta])*
                    $resource_name:ident: $resource_field:ident $(=$resource_alias:ident)?),*
                $(,)?
            }
            $(,)?
        )+
    } => {
        #[allow(dead_code,non_snake_case,missing_docs)]
        pub struct AssignedResources {
            $(pub $group_name : $group_struct),*
        }
        $(
            #[allow(dead_code,non_snake_case)]
            $(#[$outer])*
            pub struct $group_struct {
                $(
                    $(#[$inner])*
                    pub $resource_name: Peri<'static, peripherals::$resource_field>
                ),*
            }
        )+


        $($($(
            #[allow(missing_docs)]
            pub type $resource_alias = Peri<'static, peripherals::$resource_field>;
        )?)*)*

        #[macro_export]
        /// `split_resources!` macro
        macro_rules! split_resources (
            ($p:ident) => {
                AssignedResources {
                    $($group_name: $group_struct {
                        $($resource_name: $p.$resource_field),*
                    }),*
                }
            }
        );
    }
}

pub struct SyncWatch<M: RawMutex, T: Copy> {
    state: Mutex<M, RefCell<SyncWatchState<T>>>,
}

struct SyncWatchState<T: Copy> {
    value: T,
    current_version: u64,
}

impl<M: RawMutex, T: Copy> SyncWatch<M, T> {
    pub const fn new(initial_value: T) -> Self {
        Self {
            state: Mutex::new(RefCell::new(SyncWatchState {
                value: initial_value,
                current_version: 0,
            })),
        }
    }

    pub fn watcher<'a>(&'a self) -> SyncWatchReader<'a, M, T> {
        SyncWatchReader {
            state: &self.state,
            last_read_version: None,
        }
    }

    pub fn get(&self) -> T {
        self.state.lock(|state| state.borrow().value)
    }

    pub fn set(&self, new_value: T) {
        self.state.lock(|state| {
            let mut state = state.borrow_mut();
            state.value = new_value;
            state.current_version += 1;
        });
    }
}

pub struct SyncWatchReader<'a, M: RawMutex, T: Copy> {
    state: &'a Mutex<M, RefCell<SyncWatchState<T>>>,
    last_read_version: Option<u64>,
}

impl<'a, M: RawMutex, T: Copy> SyncWatchReader<'a, M, T> {
    pub fn get_if_changed(&mut self) -> Option<T> {
        self.state.lock(|state| {
            let state = state.borrow();
            if let Some(last_read_version) = self.last_read_version
                && state.current_version <= last_read_version
            {
                return None;
            }
            self.last_read_version = Some(state.current_version);
            Some(state.value)
        })
    }
}
