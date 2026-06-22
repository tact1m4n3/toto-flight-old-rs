use core::ops::Range;

use embassy_sync::{channel::Channel, once_lock::OnceLock, signal::Signal};
use embedded_storage_async::nor_flash::NorFlash;
use heapless::String;
use nalgebra::{Matrix3, Vector3};
use sequential_storage::{
    cache::NoCache,
    map::{Key, MapConfig, MapStorage, PostcardValue, SerializationError},
};

pub use sequential_storage::Error;
use uom::{
    ConstZero,
    si::{angle::degree, f32::Angle},
};

use crate::{
    MAX_MOTOR_OUTPUTS, MAX_SERVO_OUTPUTS, RawMutex,
    control::ArmBlocker,
    math::{Acceleration3D, AngularVelocity3D, Vector3Ext},
};

const BUFFER_SIZE: usize = 128;

pub static INITIAL: OnceLock<Config> = OnceLock::new();

static UPDATE: Channel<RawMutex, UpdateItem, 1> = Channel::new();
static UPDATE_ACK: Signal<RawMutex, bool> = Signal::new();

/// Must run in the highest priority task. Tasks of higher priority waiting on
/// the initial config deadlock otherwise.
pub async fn task<F: NorFlash>(flash: F, range: Range<u32>) {
    let mut storage = Storage::new(flash, range);
    let config = storage.read().await;
    INITIAL.init(config).ok(); // This shouldn't happen, but if it does whatever

    loop {
        let config_item = UPDATE.receive().await;
        UPDATE_ACK.signal(storage.update_config(config_item).await.is_ok());
    }
}

#[derive(Debug)]
pub enum UpdateError {
    CraftArmed,
    OtherError,
}

#[derive(Clone)]
pub enum UpdateItem {
    CraftName(String<16>),
    BoardForwardAngle(Angle),
    ImuCalibration(ImuCalibration),
    ImuToBodyTransform(Matrix3<f32>),
    RateControllerConfig(RateControllerConfig),
    MotorMixerConfig { index: u8, config: MotorMixerConfig },
    ServoMixerConfig { index: u8, config: ServoMixerConfig },
}

impl UpdateItem {
    pub async fn commit(self) -> Result<(), UpdateError> {
        if let Ok(_arm_guard) = ArmBlocker::new() {
            UPDATE.send(self).await;
            if UPDATE_ACK.wait().await {
                Ok(())
            } else {
                warn!("unexpected error occured while updating config");
                Err(UpdateError::OtherError)
            }
        } else {
            warn!("can't update config while the craft is armed");
            Err(UpdateError::CraftArmed)
        }
    }
}

pub struct Config {
    pub craft_name: String<16>,
    pub board_forward_angle: Angle,
    pub imu_calibration: ImuCalibration,
    pub imu_to_body_transform: Matrix3<f32>,
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

        config.board_forward_angle = Angle::new::<degree>(90.0f32);

        config.motors_config[0] = MotorMixerConfig {
            trim_min_us: 1000,
            trim_max_us: 2000,
            mixer_gain: Vector3::ZEROS,
        };
        config.servos_config[0] = ServoMixerConfig {
            mid_point_us: 1600,
            trim_min_us: 1200,
            trim_max_us: 1800,
            mixer_gain: Vector3::new(-1.0, -1.0, 0.0),
        };
        config.servos_config[1] = ServoMixerConfig {
            mid_point_us: 1450,
            trim_min_us: 1200,
            trim_max_us: 1800,
            mixer_gain: Vector3::new(-1.0, 1.0, 0.0),
        };

        config
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            craft_name: String::new(),
            board_forward_angle: Angle::ZERO,
            imu_calibration: ImuCalibration {
                gyro_bias: AngularVelocity3D::ZEROS,
                accel_bias: Acceleration3D::ZEROS,
                accel_scale: Vector3::new(1.0, 1.0, 1.0),
            },
            imu_to_body_transform: Matrix3::identity(),
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
                mixer_gain: Vector3::zeros(),
            }; MAX_MOTOR_OUTPUTS],
            servos_config: [ServoMixerConfig {
                mid_point_us: 1500,
                trim_min_us: 1000,
                trim_max_us: 2000,
                mixer_gain: Vector3::zeros(),
            }; MAX_SERVO_OUTPUTS],
        }
    }
}

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct ImuCalibration {
    pub gyro_bias: AngularVelocity3D,
    pub accel_bias: Acceleration3D,
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
    pub mixer_gain: Vector3<f32>,
}

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct ServoMixerConfig {
    pub mid_point_us: u16,
    pub trim_min_us: u16,
    pub trim_max_us: u16,
    pub mixer_gain: Vector3<f32>,
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

    async fn read(&mut self) -> Config {
        match self.read_inner().await {
            Ok(config) => config,
            Err(err) => {
                if let Error::Corrupted {} = err {
                    warn!("config in flash is corrupted. erasing all config");
                    self.inner.erase_all().await.ok();
                } else {
                    warn!("unexpected error while reading config");
                }
                Config::default()
            }
        }
    }

    async fn update_config(&mut self, config_item: UpdateItem) -> Result<(), Error<F::Error>> {
        match config_item {
            UpdateItem::CraftName(name) => {
                self.raw_write(&StorageKey::CraftName, &StorageValue::CraftName(name))
                    .await?;
            }
            UpdateItem::BoardForwardAngle(angle) => {
                self.raw_write(
                    &StorageKey::BoardForwardAngle,
                    &StorageValue::BoardForwardAngle(angle),
                )
                .await?;
            }
            UpdateItem::ImuCalibration(calib) => {
                self.raw_write(
                    &StorageKey::ImuCalibration,
                    &StorageValue::ImuCalibration(calib),
                )
                .await?;
            }
            UpdateItem::ImuToBodyTransform(orientation) => {
                self.raw_write(
                    &StorageKey::BoardToCraftTransform,
                    &StorageValue::BoardToCraftTransform(orientation),
                )
                .await?;
            }
            UpdateItem::RateControllerConfig(rcc) => {
                self.raw_write(
                    &StorageKey::RateControllerConfig,
                    &StorageValue::RateControllerConfig(rcc),
                )
                .await?;
            }
            UpdateItem::MotorMixerConfig { index, config } => {
                self.raw_write(
                    &StorageKey::MotorConfig { index },
                    &StorageValue::MotorConfig(config),
                )
                .await?;
            }
            UpdateItem::ServoMixerConfig { index, config } => {
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

        match self.raw_read(&StorageKey::BoardToCraftTransform).await? {
            Some(StorageValue::BoardToCraftTransform(board_orientation)) => {
                cfg.imu_to_body_transform = board_orientation;
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
enum StorageKey {
    CraftName,
    BoardForwardAngle,
    ImuCalibration,
    BoardToCraftTransform,
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
enum StorageValue {
    CraftName(String<16>),
    BoardForwardAngle(Angle),
    ImuCalibration(ImuCalibration),
    BoardToCraftTransform(Matrix3<f32>),
    RateControllerConfig(RateControllerConfig),
    MotorConfig(MotorMixerConfig),
    ServoConfig(ServoMixerConfig),
}

impl PostcardValue<'_> for StorageValue {}
