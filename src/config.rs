use core::ops::Range;

use embassy_sync::channel::Channel;
use embedded_storage_async::nor_flash::NorFlash;
use heapless::String;
use nalgebra::{Matrix3, Vector3};
use sequential_storage::{
    cache::NoCache,
    map::{Key, MapConfig, MapStorage, PostcardValue, SerializationError},
};

pub use sequential_storage::Error;

use crate::{
    MAX_MOTOR_OUTPUTS, MAX_SERVO_OUTPUTS, RawMutex,
    control::ArmBlocker,
    utils::{NalgebraMatrix3Ext, NalgebraVec3Ext},
};

const BUFFER_SIZE: usize = 128;

pub static UPDATE: Channel<RawMutex, ConfigItem, 4> = Channel::new();

#[derive(Clone)]
pub enum ConfigItem {
    CraftName(String<16>),
    BoardForwardAngle(f32),
    ImuCalibration(ImuCalibration),
    BoardOrientation(Matrix3<f32>),
    RateControllerConfig(RateControllerConfig),
    MotorMixerConfig { index: u8, config: MotorMixerConfig },
    ServoMixerConfig { index: u8, config: ServoMixerConfig },
}

pub struct Config {
    pub craft_name: String<16>,
    pub board_forward_angle: f32,
    pub imu_calibration: ImuCalibration,
    pub board_orientation: Matrix3<f32>,
    pub rate_controller_config: RateControllerConfig,
    pub motors_config: [MotorMixerConfig; MAX_MOTOR_OUTPUTS],
    pub servos_config: [ServoMixerConfig; MAX_SERVO_OUTPUTS],
}

impl Config {
    // NOTE: temporary default configuration for my plane
    pub fn toto_plane() -> Config {
        let mut config = Config::default();

        config.craft_name.clear();
        _ = config.craft_name.push_str("Toto Plane");

        config.board_forward_angle = 90.0f32.to_radians();

        config.motors_config[0] = MotorMixerConfig {
            trim_min_us: 1000,
            trim_max_us: 2000,
            throttle_gain: 1.0,
            mixer_gain: Vector3::ZEROS,
        };
        config.servos_config[0] = ServoMixerConfig {
            mid_point_us: 1600,
            trim_min_us: 1200,
            trim_max_us: 1800,
            mixer_gain: Vector3::new(-1.0, 1.0, 0.0),
        };
        config.servos_config[1] = ServoMixerConfig {
            mid_point_us: 1450,
            trim_min_us: 1200,
            trim_max_us: 1800,
            mixer_gain: Vector3::new(-1.0, -1.0, 0.0),
        };

        config
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            craft_name: String::new(),
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
                    ki: 0.0,
                    kff: 40.0,
                },
            },
            motors_config: [MotorMixerConfig {
                trim_min_us: 1000,
                trim_max_us: 2000,
                throttle_gain: 0.0,
                mixer_gain: Vector3::ZEROS,
            }; MAX_MOTOR_OUTPUTS],
            servos_config: [ServoMixerConfig {
                mid_point_us: 1500,
                trim_min_us: 1000,
                trim_max_us: 2000,
                mixer_gain: Vector3::ZEROS,
            }; MAX_SERVO_OUTPUTS],
        }
    }
}

pub struct Storage<F: NorFlash> {
    inner: MapStorage<StorageKey, F, NoCache>,
}

impl<F: NorFlash> Storage<F> {
    pub fn new(flash: F, range: Range<u32>) -> Storage<F> {
        Self {
            inner: MapStorage::new(flash, MapConfig::new(range), NoCache),
        }
    }

    pub async fn read(&mut self) -> Result<Config, Error<F::Error>> {
        match self.read_inner().await {
            Ok(config) => Ok(config),
            Err(Error::Corrupted {}) => {
                warn!("config in flash is corrupted. erasing all config");
                self.inner.erase_all().await.ok();
                Ok(Config::default())
            }
            Err(err) => Err(err),
        }
    }

    pub async fn run(&mut self) {
        loop {
            let config_item = UPDATE.receive().await;
            if let Ok(_arm_guard) = ArmBlocker::new() {
                if self.update_config(config_item).await.is_err() {
                    error!("failed to store config in flash");
                }
            } else {
                warn!("config update dropped because craft is armed");
            }
        }
    }

    async fn update_config(&mut self, config_item: ConfigItem) -> Result<(), Error<F::Error>> {
        match config_item {
            ConfigItem::CraftName(name) => {
                self.raw_write(&StorageKey::CraftName, &StorageValue::CraftName(name))
                    .await?;
            }
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
            ConfigItem::MotorMixerConfig { index, config } => {
                self.raw_write(
                    &StorageKey::MotorConfig { index },
                    &StorageValue::MotorConfig(config),
                )
                .await?;
            }
            ConfigItem::ServoMixerConfig { index, config } => {
                self.raw_write(
                    &StorageKey::ServoConfig { index },
                    &StorageValue::ServoConfig(config),
                )
                .await?;
            }
        }

        Ok(())
    }

    async fn read_inner(&mut self) -> Result<Config, Error<F::Error>> {
        let mut cfg = Config::toto_plane(); // TODO: use default config

        match self.raw_read(&StorageKey::BoardForwardAngle).await? {
            Some(StorageValue::BoardForwardAngle(bfa)) => cfg.board_forward_angle = bfa,
            Some(_) => return Err(Error::Corrupted {}),
            None => (),
        }

        match self.raw_read(&StorageKey::ImuCalibration).await? {
            Some(StorageValue::ImuCalibration(imu_calib)) => cfg.imu_calibration = imu_calib,
            Some(_) => return Err(Error::Corrupted {}),
            None => (),
        }

        match self.raw_read(&StorageKey::BoardOrientation).await? {
            Some(StorageValue::BoardOrientation(board_orientation)) => {
                cfg.board_orientation = board_orientation;
            }
            Some(_) => return Err(Error::Corrupted {}),
            None => (),
        }

        match self.raw_read(&StorageKey::RateControllerConfig).await? {
            Some(StorageValue::RateControllerConfig(rcc)) => cfg.rate_controller_config = rcc,
            Some(_) => return Err(Error::Corrupted {}),
            None => (),
        }

        for (i, motor_config) in cfg.motors_config.iter_mut().enumerate() {
            match self
                .raw_read(&StorageKey::MotorConfig { index: i as u8 })
                .await?
            {
                Some(StorageValue::MotorConfig(mc)) => *motor_config = mc,
                Some(_) => return Err(Error::Corrupted {}),
                None => (),
            }
        }
        for (i, servo_config) in cfg.servos_config.iter_mut().enumerate() {
            match self
                .raw_read(&StorageKey::ServoConfig { index: i as u8 })
                .await?
            {
                Some(StorageValue::ServoConfig(sc)) => *servo_config = sc,
                Some(_) => return Err(Error::Corrupted {}),
                None => (),
            }
        }

        Ok(cfg)
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
    MotorConfig { index: u8 },
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

#[derive(serde::Serialize, serde::Deserialize)]
pub enum StorageValue {
    CraftName(String<16>),
    BoardForwardAngle(f32),
    ImuCalibration(ImuCalibration),
    BoardOrientation(Matrix3<f32>),
    RateControllerConfig(RateControllerConfig),
    MotorConfig(MotorMixerConfig),
    ServoConfig(ServoMixerConfig),
}

impl PostcardValue<'_> for StorageValue {}

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct ImuCalibration {
    pub gyro_bias: Vector3<f32>,
    pub accel_bias: Vector3<f32>,
    pub accel_scale: Vector3<f32>,
}

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct RateControllerConfig {
    pub roll: RateControllerAxisConfig,
    pub pitch: RateControllerAxisConfig,
    pub yaw: RateControllerAxisConfig,
}

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct RateControllerAxisConfig {
    pub kp: f32,
    pub ki: f32,
    pub kff: f32,
}

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct MotorMixerConfig {
    pub trim_min_us: u16,
    pub trim_max_us: u16,
    pub throttle_gain: f32,
    pub mixer_gain: Vector3<f32>,
}

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct ServoMixerConfig {
    pub mid_point_us: u16,
    pub trim_min_us: u16,
    pub trim_max_us: u16,
    pub mixer_gain: Vector3<f32>,
}
