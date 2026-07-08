use core::{
    cell::{Cell, RefCell},
    ops::Range,
    sync::atomic::{AtomicU32, Ordering},
};

use embassy_sync::blocking_mutex::{
    Mutex,
    raw::{CriticalSectionRawMutex, RawMutex},
};
use embassy_time::Duration;
use embedded_io_async::BufRead;
use heapless::Deque;
use uom::si::{f32::Time, time::microsecond};

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

pub trait DurationExt {
    fn time_f32(&self) -> Time;
    fn dt_f32(&self) -> f32;
}

impl DurationExt for Duration {
    fn time_f32(&self) -> Time {
        Time::new::<microsecond>(self.as_micros() as f32)
    }
    fn dt_f32(&self) -> f32 {
        self.as_micros() as f32 / 1_000_000.0
    }
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

struct SyncWatchState<T: Copy> {
    value: Option<T>,
    current_version: u32,
    sender: Option<()>,
}

pub(crate) struct SyncWatch<T: Copy> {
    state: Mutex<CriticalSectionRawMutex, RefCell<SyncWatchState<T>>>,
}

impl<T: Copy> SyncWatch<T> {
    pub const fn new() -> Self {
        Self {
            state: Mutex::new(RefCell::new(SyncWatchState {
                value: None,
                current_version: 0,
                sender: Some(()),
            })),
        }
    }

    pub fn sender<'a>(&'a self) -> SyncWatchSender<'a, T> {
        self.state
            .lock(|state| state.borrow_mut().sender.take())
            .unwrap();
        SyncWatchSender { watch: self }
    }

    pub fn receiver<'a>(&'a self) -> SyncWatchReceiver<'a, T> {
        SyncWatchReceiver {
            watch: self,
            last_read_version: None,
        }
    }
}

pub struct SyncWatchSender<'a, T: Copy> {
    watch: &'a SyncWatch<T>,
}

impl<'a, T: Copy> SyncWatchSender<'a, T> {
    pub fn send(&self, new_value: T) {
        self.watch.state.lock(|state| {
            let mut state = state.borrow_mut();
            state.value = Some(new_value);
            state.current_version = state.current_version.wrapping_add(1);
        });
    }
}

#[derive(Clone)]
pub struct SyncWatchReceiver<'a, T: Copy> {
    watch: &'a SyncWatch<T>,
    last_read_version: Option<u32>,
}

impl<'a, T: Copy> SyncWatchReceiver<'a, T> {
    pub fn get(&self) -> Option<T> {
        self.watch.state.lock(|state| state.borrow().value)
    }

    pub fn get_if_changed(&mut self) -> Option<T> {
        self.watch.state.lock(|state| {
            let state = state.borrow();

            if let Some(last_read_version) = self.last_read_version
                && state.current_version == last_read_version
            {
                return None;
            }
            self.last_read_version = Some(state.current_version);

            state.value
        })
    }
}

pub async fn read_process_and_consume<R: BufRead, F: FnMut(&[u8])>(
    reader: &mut R,
    mut f: F,
) -> Result<(), R::Error> {
    let n = {
        let data = match reader.fill_buf().await {
            Ok(data) => data,
            Err(e) => {
                // If we fail to read, there is an edge case in which the uart
                // interrupt maybe remains disabled and we don't read anything
                // anymore.
                // TODO: Inspect buffered uart implementation.
                reader.consume(1);
                return Err(e);
            }
        };
        f(data);
        data.len()
    };
    reader.consume(n);
    Ok(())
}
