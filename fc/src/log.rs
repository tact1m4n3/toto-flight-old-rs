#[cfg(feature = "defmt")]
#[macro_export]
macro_rules! trace {
    ($($arg:tt)*) => {
        ::defmt::trace!($($arg)*);
    };
}
#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! trace {
    ($($arg:tt)*) => {
        { $( let _ = &$arg; )* }
    };
}

#[cfg(feature = "defmt")]
#[macro_export]
macro_rules! debug {
    ($($arg:expr),*) => {
        ::defmt::debug!($($arg)*);
    };
}
#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! debug {
    ($($arg:expr),*) => {
        { $( let _ = &$arg; )* }
    };
}

#[cfg(feature = "defmt")]
#[macro_export]
macro_rules! info {
    ($($arg:expr),*) => {
        ::defmt::info!($($arg)*);
    };
}
#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! info {
    ($($arg:expr),*) => {
        { $( let _ = &$arg; )* }
    };
}

#[cfg(feature = "defmt")]
#[macro_export]
macro_rules! warn {
    ($($arg:expr),*) => {
        ::defmt::warn!($($arg)*);
    };
}
#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! warn {
    ($($arg:expr),*) => {
        { $( let _ = &$arg; )* }
    };
}

#[cfg(feature = "defmt")]
#[macro_export]
macro_rules! error {
    ($($arg:expr),*) => {
        ::defmt::error!($($arg)*);
    };
}
#[cfg(not(feature = "defmt"))]
#[macro_export]
macro_rules! error {
    ($($arg:expr),*) => {
        { $( let _ = &$arg; )* }
    };
}
