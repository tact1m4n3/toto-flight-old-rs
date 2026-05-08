use core::{cell::RefCell, ops::Range};

use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embassy_time::Instant;
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

pub fn pack<const N: usize>(s: &str) -> [u8; N] {
    let mut buf = [0u8; N];
    let b = s.as_bytes();
    let len = b.len().min(N);
    buf[..len].copy_from_slice(&b[..len]);
    buf
}

#[macro_export]
macro_rules! mutex_cells {
    ($($name:ident: $ty:ty = $init:expr);* $(;)?) => {
        $(
            static $name: embassy_sync::blocking_mutex::Mutex<
                embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, core::cell::Cell<$ty>
            > = embassy_sync::blocking_mutex::Mutex::new(core::cell::Cell::new($init));
        )*
    };
}

#[macro_export]
macro_rules! signals {
    ($($name:ident: $ty:ty);* $(;)?) => {
        $(
            static $name: embassy_sync::signal::Signal<
                embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex, $ty
            > = embassy_sync::signal::Signal::new();
        )*
    };
}

pub struct SyncWatch<T: Copy> {
    state: Mutex<CriticalSectionRawMutex, RefCell<SyncWatchState<T>>>,
}

struct SyncWatchState<T: Copy> {
    value: T,
    update_timestamp: Instant,
}

impl<T: Copy> SyncWatch<T> {
    pub const fn new(initial_value: T) -> Self {
        Self {
            state: Mutex::new(RefCell::new(SyncWatchState {
                value: initial_value,
                update_timestamp: Instant::MIN,
            })),
        }
    }

    pub fn watcher<'a>(&'a self) -> SyncWatchReader<'a, T> {
        SyncWatchReader {
            state: &self.state,
            last_read_timestamp: None,
        }
    }

    pub fn get(&self) -> T {
        self.state.lock(|state| state.borrow().value)
    }

    pub fn get_with_timestamp(&self) -> (T, Instant) {
        self.state.lock(|state| {
            let state = state.borrow();
            (state.value, state.update_timestamp)
        })
    }

    pub fn set(&self, new_value: T) {
        self.set_with_timestamp(new_value, Instant::now());
    }

    pub fn set_with_timestamp(&self, new_value: T, timestamp: Instant) {
        self.state.lock(|state| {
            *state.borrow_mut() = SyncWatchState {
                value: new_value,
                update_timestamp: timestamp,
            };
        });
    }
}

pub struct SyncWatchReader<'a, T: Copy> {
    state: &'a Mutex<CriticalSectionRawMutex, RefCell<SyncWatchState<T>>>,
    last_read_timestamp: Option<Instant>,
}

impl<'a, T: Copy> SyncWatchReader<'a, T> {
    pub fn get_if_changed(&mut self) -> Option<(T, Instant)> {
        self.state.lock(|state| {
            let state = state.borrow();
            if let Some(last_read_timestamp) = self.last_read_timestamp
                && state.update_timestamp <= last_read_timestamp
            {
                return None;
            }
            self.last_read_timestamp = Some(state.update_timestamp);
            Some((state.value, state.update_timestamp))
        })
    }
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
