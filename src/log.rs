#[macro_export]
macro_rules! trace {
    ($($arg:tt)*) => {
        #[cfg(feature = "defmt")]
        ::defmt::trace!($($arg)*);
        #[cfg(not(feature = "defmt"))]
        { $( let _ = &$arg; )* }
    };
}

#[macro_export]
macro_rules! info {
    ($($arg:tt)*) => {
        #[cfg(feature = "defmt")]
        ::defmt::info!($($arg)*);
    };
}

#[macro_export]
macro_rules! warn {
    ($($arg:tt)*) => {
        #[cfg(feature = "defmt")]
        ::defmt::warn!($($arg)*);
    };
}

#[macro_export]
macro_rules! error {
    ($($arg:tt)*) => {
        #[cfg(feature = "defmt")]
        ::defmt::error!($($arg)*);
    };
}
