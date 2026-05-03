use core::ops::Range;

use embassy_rp::{flash, peripherals::FLASH};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use sequential_storage::cache::NoCache;

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

macro_rules! impl_get_set {
    ($get_name:ident, $set_name:ident, $key:expr, $typ:ty) => {
        const _: Key = $key; // assert key type

        #[allow(unused)]
        pub async fn $get_name() -> Option<$typ> {
            let mut storage = STORAGE.lock().await;
            let Some(storage) = storage.as_mut() else {
                return None;
            };
            const SIZE: usize = (core::mem::size_of::<Key>() + core::mem::size_of::<$typ>()) + FLASH_WORD_SIZE;
            let mut data_buffer: [u8; SIZE] = [0; SIZE];
            match storage.fetch_item::<$typ>(&mut data_buffer, &$key).await {
                Ok(value) => value,
                Err(err) => {
                    defmt::warn!("failed to read config: {}", err);
                    None
                }
            }
        }

        #[allow(unused)]
        pub async fn $set_name(value: $typ) {
            let mut storage = STORAGE.lock().await;
            let Some(storage) = storage.as_mut() else {
                return;
            };
            const SIZE: usize = core::mem::size_of::<Key>() + core::mem::size_of::<$typ>() + FLASH_WORD_SIZE;
            let mut data_buffer: [u8; SIZE] = [0; SIZE];
            match storage
                .store_item::<$typ>(&mut data_buffer, &$key, &value)
                .await
            {
                Ok(()) => (),
                Err(err) => {
                    defmt::warn!("failed to write config: {}", err);
                }
            }
        }
    };
}

impl_get_set!(get_craft_name, set_craft_name, Key::CraftName, [u8; 16]);
impl_get_set!(get_gyro_bias, set_gyro_bias, Key::GyroBias, [f32; 3]);
impl_get_set!(get_accel_bias, set_accel_bias, Key::AccelBias, [f32; 3]);

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Key {
    CraftName = 0,
    GyroBias = 1,
    AccelBias = 2,
}

impl TryFrom<u8> for Key {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Key::CraftName),
            1 => Ok(Key::GyroBias),
            2 => Ok(Key::AccelBias),
            _ => Err(()),
        }
    }
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
            Self::try_from(buffer[0])
                .map(|key| (key, 1))
                .map_err(|_| sequential_storage::map::SerializationError::InvalidData)
        } else {
            Err(sequential_storage::map::SerializationError::BufferTooSmall)
        }
    }
}
