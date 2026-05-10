use core::ops::Range;

use embassy_rp::{flash, peripherals::FLASH};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use num_traits::FromPrimitive;
use sequential_storage::{cache::NoCache, map::Value};

use crate::{FlashPeripherals, Irqs};

const FLASH_STORAGE_RANGE: Range<u32> = 15 * 1024 * 1024..16 * 1024 * 1024; // store in the last megabyte
const FLASH_SIZE: usize = 16 * 1024 * 1024;
const FLASH_WORD_SIZE: usize = 4;

type Flash = flash::Flash<'static, FLASH, flash::Async, FLASH_SIZE>;
type MapStorage =
    sequential_storage::map::MapStorage<Key, Flash, sequential_storage::cache::NoCache>;

// static FLUSH: AtomicBool = AtomicBool::new(false);
static STORAGE: Mutex<CriticalSectionRawMutex, Option<MapStorage>> = Mutex::new(None);

pub async fn init(p: FlashPeripherals) {
    let flash_storage = Flash::new(p.flash, p.dma, Irqs);
    let storage = MapStorage::new(
        flash_storage,
        sequential_storage::map::MapConfig::new(FLASH_STORAGE_RANGE),
        NoCache,
    );
    STORAGE.lock().await.replace(storage);
}

/// Max key size + max value size aligned to the flash word size.
const BUF_SIZE: usize = 64;

pub async fn get<T: Copy + for<'a> Value<'a>>(key: Key) -> Option<T> {
    let mut storage = STORAGE.lock().await;
    let storage = storage.as_mut()?;
    let mut data_buffer: [u8; BUF_SIZE] = [0; BUF_SIZE];
    match storage.fetch_item::<T>(&mut data_buffer, &key).await {
        Ok(value) => value,
        Err(err) => {
            defmt::warn!("failed to read config for {}: {}", stringify!($name), err);
            None
        }
    }
}

pub async fn set<T: Copy + for<'a> Value<'a>>(key: Key, value: T) {
    let mut storage = STORAGE.lock().await;
    let Some(storage) = storage.as_mut() else {
        return;
    };
    let mut data_buffer: [u8; BUF_SIZE] = [0; BUF_SIZE];
    match storage
        .store_item::<T>(&mut data_buffer, &key, &value)
        .await
    {
        Ok(()) => (),
        Err(err) => {
            defmt::warn!("failed to write config for {}: {}", stringify!($name), err);
        }
    }
}

macro_rules! impl_get_set {
    ($name:ident, $key:expr, $typ:ty) => {
        const _: Key = $key; // assert key type

        paste::paste! {
            #[allow(unused)]
            pub async fn [<get_ $name>]() -> Option<$typ> {
                get($key).await
            }

            #[allow(unused)]
            pub async fn [<set_ $name>](value: $typ) {
                set($key, value).await
            }
        }
    };
}

impl_get_set!(board_forward_angle, Key::BoardForwardAngle, f32);
impl_get_set!(gyro_bias, Key::GyroBias, [f32; 3]);
impl_get_set!(imu_orientation, Key::ImuOrientation, [f32; 3 * 3]);

impl_get_set!(craft_name, Key::CraftName, [u8; 16]);

impl_get_set!(roll_rate_kp, Key::RollRateKp, f32);
impl_get_set!(roll_rate_ki, Key::RollRateKi, f32);
impl_get_set!(roll_rate_kff, Key::RollRateKff, f32);
impl_get_set!(pitch_rate_kp, Key::PitchRateKp, f32);
impl_get_set!(pitch_rate_ki, Key::PitchRateKi, f32);
impl_get_set!(pitch_rate_kff, Key::PitchRateKff, f32);
impl_get_set!(yaw_rate_kp, Key::YawRateKp, f32);
impl_get_set!(yaw_rate_ki, Key::YawRateKi, f32);
impl_get_set!(yaw_rate_kff, Key::YawRateKff, f32);

impl_get_set!(left_elevon_mid_point, Key::LeftElevonMidPoint, i16);
impl_get_set!(left_elevon_range_us, Key::LeftElevonRangeUs, [i16; 2]);
impl_get_set!(left_elevon_mixer_gain, Key::LeftElevonMixerGain, [f32; 3]);
impl_get_set!(right_elevon_mid_point, Key::RightElevonMidPoint, i16);
impl_get_set!(right_elevon_range_us, Key::RightElevonRangeUs, [i16; 2]);
impl_get_set!(right_elevon_mixer_gain, Key::RightElevonMixerGain, [f32; 3]);

#[derive(Clone, Copy, PartialEq, Eq, num_derive::FromPrimitive)]
#[repr(u8)]
pub enum Key {
    BoardForwardAngle = 0,
    GyroBias = 1,
    ImuOrientation = 2,

    CraftName = 40,

    RollRateKp = 80,
    RollRateKi = 81,
    RollRateKff = 82,
    PitchRateKp = 83,
    PitchRateKi = 84,
    PitchRateKff = 85,
    YawRateKp = 86,
    YawRateKi = 87,
    YawRateKff = 88,

    LeftElevonMidPoint = 100,
    LeftElevonRangeUs = 101,
    LeftElevonMixerGain = 102,
    RightElevonMidPoint = 103,
    RightElevonRangeUs = 104,
    RightElevonMixerGain = 105,
}

impl sequential_storage::map::Key for Key {
    fn serialize_into(
        &self,
        buffer: &mut [u8],
    ) -> Result<usize, sequential_storage::map::SerializationError> {
        if buffer.len() >= 1 {
            buffer[0] = *self as u8;
            Ok(1)
        } else {
            Err(sequential_storage::map::SerializationError::BufferTooSmall)
        }
    }

    fn deserialize_from(
        buffer: &[u8],
    ) -> Result<(Self, usize), sequential_storage::map::SerializationError> {
        if buffer.len() >= 1 {
            Self::from_u8(buffer[0]).map_or(
                Err(sequential_storage::map::SerializationError::InvalidData),
                |key| Ok((key, 1)),
            )
        } else {
            Err(sequential_storage::map::SerializationError::BufferTooSmall)
        }
    }
}
