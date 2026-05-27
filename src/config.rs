use core::ops::Range;

use embassy_sync::{channel::Channel, once_lock::OnceLock};
use embedded_storage_async::nor_flash::NorFlash;
use nalgebra::{Matrix3, Vector3};
use sequential_storage::{
    cache::NoCache,
    map::{Key, MapConfig, MapStorage, PostcardValue, SerializationError},
};

pub use sequential_storage::Error;

use crate::{
    RawMutex,
    utils::{NalgebraMatrix3Ext, NalgebraVec3Ext},
};

const BUFFER_SIZE: usize = 64;

pub static INITIAL_CONFIG: OnceLock<FullConfig> = OnceLock::new();
pub static UPDATE_CONFIG: Channel<RawMutex, ConfigItem, 4> = Channel::new();

#[derive(Clone, Copy)]
pub enum ConfigItem {
    BoardForwardAngle(f32),
    ImuCalibration(ImuCalibration),
    BoardOrientation(Matrix3<f32>),
    RateControllerConfig(RateControllerConfig),
    ServoConfig { index: u8, config: ServoConfig },
}

pub struct FullConfig {
    pub board_forward_angle: f32,
    pub imu_calibration: ImuCalibration,
    pub board_orientation: Matrix3<f32>,
    pub rate_controller_config: RateControllerConfig,
    pub servos_config: [ServoConfig; 5],
}

impl Default for FullConfig {
    fn default() -> Self {
        Self {
            board_forward_angle: 0.0,
            imu_calibration: ImuCalibration {
                gyro_bias: Vector3::ZEROS,
                accel_bias: Vector3::ZEROS,
                accel_scale: Vector3::ONES,
            },
            board_orientation: Matrix3::IDENTITY,
            rate_controller_config: RateControllerConfig {
                roll: RateControllerAxisConfig {
                    kp: 7.0,
                    ki: 10.0,
                    kff: 50.0,
                },
                pitch: RateControllerAxisConfig {
                    kp: 7.0,
                    ki: 10.0,
                    kff: 90.0,
                },
                yaw: RateControllerAxisConfig {
                    kp: 0.0,
                    ki: 00.0,
                    kff: 40.0,
                },
            },
            servos_config: [ServoConfig {
                mid_point_us: 1500,
                min_us: 1000,
                max_us: 2000,
                mixer_gain: Vector3::ZEROS,
            }; 5],
        }
    }
}

pub async fn task<F: NorFlash>(flash: F, range: Range<u32>) {
    let mut storage = Storage::new(flash, range);
    let config = match storage.read_full_config().await {
        Ok(config) => config,
        Err(Error::Corrupted {}) => {
            defmt::warn!("config in flash is corrupted. erasing all config");
            storage.inner.erase_all().await.ok();
            FullConfig::default()
        }
        Err(err) => panic!(
            "unexpected error while reading config from flash: {:?}",
            err
        ),
    };
    INITIAL_CONFIG.init(config).ok();

    loop {
        // TODO: only receive while we are not armed
        let config_item = UPDATE_CONFIG.receive().await;
        if storage.update_config(config_item).await.is_err() {
            defmt::error!("failed to update config in flash");
        }
    }
}

struct Storage<F: NorFlash> {
    inner: MapStorage<StorageKey, F, NoCache>,
}

impl<F: NorFlash> Storage<F> {
    fn new(flash: F, range: Range<u32>) -> Storage<F> {
        Self {
            inner: MapStorage::new(flash, MapConfig::new(range), NoCache),
        }
    }

    async fn update_config(&mut self, config_item: ConfigItem) -> Result<(), Error<F::Error>> {
        match config_item {
            ConfigItem::BoardForwardAngle(angle) => {
                self.raw_write(
                    &StorageKey::BoardForwardAngle,
                    &StorageValue::BoardForwardAngle(angle),
                )
                .await?;
            }
            ConfigItem::ImuCalibration(calib) => {
                self.raw_write(
                    &StorageKey::ImuCalibration,
                    &StorageValue::ImuCalibration(calib),
                )
                .await?;
            }
            ConfigItem::BoardOrientation(orientation) => {
                self.raw_write(
                    &StorageKey::BoardOrientation,
                    &StorageValue::BoardOrientation(orientation),
                )
                .await?;
            }
            ConfigItem::RateControllerConfig(rcc) => {
                self.raw_write(
                    &StorageKey::RateControllerConfig,
                    &StorageValue::RateControllerConfig(rcc),
                )
                .await?;
            }
            ConfigItem::ServoConfig { index, config } => {
                self.raw_write(
                    &StorageKey::ServoConfig { index },
                    &StorageValue::ServoConfig(config),
                )
                .await?;
            }
        }

        Ok(())
    }

    async fn read_full_config(&mut self) -> Result<FullConfig, Error<F::Error>> {
        let mut config = FullConfig::default();

        match self.raw_read(&StorageKey::BoardForwardAngle).await? {
            Some(StorageValue::BoardForwardAngle(bfa)) => config.board_forward_angle = bfa,
            Some(_) => return Err(Error::Corrupted {}), // If this configuration has the wrong type
            None => (),
        };

        match self.raw_read(&StorageKey::ImuCalibration).await? {
            Some(StorageValue::ImuCalibration(imu_calib)) => config.imu_calibration = imu_calib,
            Some(_) => return Err(Error::Corrupted {}), // If this configuration has the wrong type
            None => (),
        };

        match self.raw_read(&StorageKey::RateControllerConfig).await? {
            Some(StorageValue::RateControllerConfig(rcc)) => config.rate_controller_config = rcc,
            Some(_) => return Err(Error::Corrupted {}), // If this configuration has the wrong type
            None => (),
        };

        for (i, servo_config) in config.servos_config.iter_mut().enumerate() {
            match self
                .raw_read(&StorageKey::ServoConfig { index: i as u8 })
                .await?
            {
                Some(StorageValue::ServoConfig(sc)) => *servo_config = sc,
                Some(_) => return Err(Error::Corrupted {}), // If this configuration has the wrong type
                None => (),
            };
        }

        Ok(config)
    }

    async fn raw_read(
        &mut self,
        key: &StorageKey,
    ) -> Result<Option<StorageValue>, Error<F::Error>> {
        let mut buffer = [0u8; BUFFER_SIZE];
        self.inner.fetch_item(&mut buffer, key).await
    }

    async fn raw_write(
        &mut self,
        key: &StorageKey,
        value: &StorageValue,
    ) -> Result<(), Error<F::Error>> {
        let mut buffer = [0u8; BUFFER_SIZE];
        self.inner.store_item(&mut buffer, key, value).await?;
        Ok(())
    }
}

#[derive(Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum StorageKey {
    CraftName,
    BoardForwardAngle,
    ImuCalibration,
    BoardOrientation,
    RateControllerConfig,
    ServoConfig { index: u8 },
}

impl Key for StorageKey {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        postcard::to_slice(self, buffer)
            .map(|used| used.len())
            .map_err(Into::into)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        let (key, rest): (Self, &[u8]) =
            postcard::take_from_bytes(buffer).map_err(SerializationError::from)?;
        Ok((key, buffer.len() - rest.len()))
    }

    fn get_len(buffer: &[u8]) -> Result<usize, SerializationError> {
        Self::deserialize_from(buffer).map(|(_, len)| len)
    }
}

#[derive(Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ConfigAxis {
    Roll,
    Pitch,
    Yaw,
}

#[derive(Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum PidControllerParamKind {
    Kp,
    Ki,
    Kd,
    Kff,
}

#[derive(serde::Serialize, serde::Deserialize)]
pub enum StorageValue {
    CraftName(heapless::String<16>),
    BoardForwardAngle(f32),
    ImuCalibration(ImuCalibration),
    BoardOrientation(Matrix3<f32>),
    RateControllerConfig(RateControllerConfig),
    ServoConfig(ServoConfig),
}

impl<'a> PostcardValue<'a> for StorageValue {}

#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct ImuCalibration {
    pub gyro_bias: Vector3<f32>,
    pub accel_bias: Vector3<f32>,
    pub accel_scale: Vector3<f32>,
}

#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct RateControllerConfig {
    roll: RateControllerAxisConfig,
    pitch: RateControllerAxisConfig,
    yaw: RateControllerAxisConfig,
}

#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct RateControllerAxisConfig {
    kp: f32,
    ki: f32,
    kff: f32,
}

#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct ServoConfig {
    mid_point_us: u16,
    min_us: u16,
    max_us: u16,
    mixer_gain: Vector3<f32>,
}
