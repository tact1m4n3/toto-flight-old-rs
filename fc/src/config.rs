use core::{cell::RefCell, ops::Range};

use embassy_sync::{blocking_mutex::Mutex, once_lock::OnceLock, signal::Signal};
use embedded_storage_async::nor_flash::NorFlash;
use heapless::String;
use nalgebra::{Matrix3, Vector3};
use sequential_storage::{
    cache::NoCache,
    map::{Key, MapConfig, MapStorage, PostcardValue, SerializationError},
};

pub use sequential_storage::Error;
use uom::si::f32::Angle;

use crate::{
    MAX_MOTOR_OUTPUTS, MAX_SERVO_OUTPUTS, RawMutex, control,
    math::{Acceleration3D, AngularVelocity3D},
};

const BUFFER_SIZE: usize = 128;

pub static INITIAL: OnceLock<Config> = OnceLock::new();
pub static UPDATE: Mutex<RawMutex, RefCell<Config>> = Mutex::new(RefCell::new(Config::DEFAULT));
pub static SAVE: Signal<RawMutex, ()> = Signal::new();

/// Must run in the highest priority task. Tasks of higher priority waiting on
/// the initial config deadlock otherwise.
pub async fn task<F: NorFlash>(flash: F, range: Range<u32>) {
    let mut storage = Storage::new(flash, range);
    {
        let config = storage.read().await;
        INITIAL.init(config.clone()).ok();
    }

    loop {
        SAVE.wait().await;
        if control::status().arm_state.is_armed() {
            warn!("config update while armed. ignoring");
            continue;
        }
        let update = UPDATE.lock(|update| {
            let mut update = update.borrow_mut();
            let update_clone = update.clone();
            *update = Config::DEFAULT;
            update_clone
        });
        if storage.write(&update).await.is_err() {
            warn!("failed to write config to flash");
        }
    }
}

#[derive(Clone)]
pub struct Config {
    pub craft_name: Option<String<16>>,
    pub board_forward_angle: Option<Angle>,
    pub imu_calibration: Option<ImuCalibration>,
    pub imu_to_body_transform: Option<Matrix3<f32>>,
    pub rate_controller_config: Option<RateControllerConfig>,
    pub motors_config: [Option<MotorMixerConfig>; MAX_MOTOR_OUTPUTS],
    pub servo_max_throw_us: Option<u16>,
    pub servos_config: [Option<ServoMixerConfig>; MAX_SERVO_OUTPUTS],
}

impl Config {
    pub const DEFAULT: Self = Self {
        craft_name: None,
        board_forward_angle: None,
        imu_calibration: None,
        imu_to_body_transform: None,
        rate_controller_config: None,
        motors_config: [None; MAX_MOTOR_OUTPUTS],
        servo_max_throw_us: None,
        servos_config: [None; MAX_SERVO_OUTPUTS],
    };
}

// impl Config {
//     // NOTE: temporary default configuration for my plane
//     pub fn toto_plane() -> Config {
//         let mut config = Config::default();

//         config.craft_name.clear();
//         _ = config.craft_name.push_str("Toto Plane");

//         config.board_forward_angle = Angle::new::<degree>(90.0f32);

//         config.motors_config[0] = MotorMixerConfig {
//             trim_min_us: 1000,
//             trim_max_us: 2000,
//             mixer_gain: Vector3::ZEROS,
//         };

//         config.servo_max_throw_us = 200;
//         config.servos_config[0] = ServoMixerConfig {
//             mid_point_us: 1700,
//             trim_min_us: 1200,
//             trim_max_us: 1900,
//             mixer_gain: Vector3::new(-1.0, -1.0, 0.0),
//         };
//         config.servos_config[1] = ServoMixerConfig {
//             mid_point_us: 1300,
//             trim_min_us: 1100,
//             trim_max_us: 1800,
//             mixer_gain: Vector3::new(-1.0, 1.0, 0.0),
//         };

//         config
//     }
// }

// impl Default for Config {
//     fn default() -> Self {
//         Self {
//             craft_name: String::new(),
//             board_forward_angle: Angle::ZERO,
//             imu_calibration: ImuCalibration {
//                 gyro_bias: AngularVelocity3D::ZEROS,
//                 accel_bias: Acceleration3D::ZEROS,
//                 accel_scale: Vector3::new(1.0, 1.0, 1.0),
//             },
//             imu_to_body_transform: Matrix3::identity(),
//             rate_controller_config: RateControllerConfig {
//                 i_term_limit: 50.0,
//                 roll: RateControllerAxisConfig {
//                     kp: 0.5,
//                     ki: 0.8,
//                     kff: 2.0,
//                 },
//                 pitch: RateControllerAxisConfig {
//                     kp: 0.5,
//                     ki: 0.8,
//                     kff: 2.5,
//                 },
//                 yaw: RateControllerAxisConfig {
//                     kp: 0.0,
//                     ki: 0.0,
//                     kff: 1.5,
//                 },
//             },
//             motors_config: [MotorMixerConfig {
//                 trim_min_us: 1000,
//                 trim_max_us: 2000,
//                 mixer_gain: Vector3::zeros(),
//             }; MAX_MOTOR_OUTPUTS],
//             servo_max_throw_us: 500,
//             servos_config: [ServoMixerConfig {
//                 mid_point_us: 1500,
//                 trim_min_us: 1000,
//                 trim_max_us: 2000,
//                 mixer_gain: Vector3::zeros(),
//             }; MAX_SERVO_OUTPUTS],
//         }
//     }
// }

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct ImuCalibration {
    pub gyro_bias: AngularVelocity3D,
    pub accel_bias: Acceleration3D,
    pub accel_scale: Vector3<f32>,
}

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct RateControllerConfig {
    pub i_term_limit: f32,
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
                Config::DEFAULT
            }
        }
    }

    async fn write(&mut self, update: &Config) -> Result<(), Error<F::Error>> {
        if let Some(ref name) = update.craft_name {
            self.raw_write(
                &StorageKey::CraftName,
                &StorageValue::CraftName(name.clone()),
            )
            .await?;
        }
        if let Some(angle) = update.board_forward_angle {
            self.raw_write(
                &StorageKey::BoardForwardAngle,
                &StorageValue::BoardForwardAngle(angle),
            )
            .await?;
        }
        if let Some(calib) = update.imu_calibration {
            self.raw_write(
                &StorageKey::ImuCalibration,
                &StorageValue::ImuCalibration(calib),
            )
            .await?;
        }
        if let Some(orientation) = update.imu_to_body_transform {
            self.raw_write(
                &StorageKey::BoardToCraftTransform,
                &StorageValue::BoardToCraftTransform(orientation),
            )
            .await?;
        }
        if let Some(rcc) = update.rate_controller_config {
            self.raw_write(
                &StorageKey::RateControllerConfig,
                &StorageValue::RateControllerConfig(rcc),
            )
            .await?;
        }
        for (i, motor_config) in update.motors_config.iter().enumerate() {
            if let Some(mc) = motor_config {
                self.raw_write(
                    &StorageKey::MotorConfig { index: i as u8 },
                    &StorageValue::MotorConfig(*mc),
                )
                .await?;
            }
        }
        if let Some(servo_max_throw_us) = update.servo_max_throw_us {
            self.raw_write(
                &StorageKey::ServoMaxThrow,
                &StorageValue::ServoMaxThrow(servo_max_throw_us),
            )
            .await?;
        }
        for (i, servo_config) in update.servos_config.iter().enumerate() {
            if let Some(sc) = servo_config {
                self.raw_write(
                    &StorageKey::ServoConfig { index: i as u8 },
                    &StorageValue::ServoConfig(*sc),
                )
                .await?;
            }
        }

        Ok(())
    }

    async fn read_inner(&mut self) -> Result<Config, Error<F::Error>> {
        let mut cfg = Config::DEFAULT;

        match self.raw_read(&StorageKey::BoardForwardAngle).await? {
            Some(StorageValue::BoardForwardAngle(bfa)) => cfg.board_forward_angle = Some(bfa),
            Some(_) => return Err(Error::Corrupted {}),
            None => (),
        }

        match self.raw_read(&StorageKey::ImuCalibration).await? {
            Some(StorageValue::ImuCalibration(imu_calib)) => cfg.imu_calibration = Some(imu_calib),
            Some(_) => return Err(Error::Corrupted {}),
            None => (),
        }

        match self.raw_read(&StorageKey::BoardToCraftTransform).await? {
            Some(StorageValue::BoardToCraftTransform(board_orientation)) => {
                cfg.imu_to_body_transform = Some(board_orientation);
            }
            Some(_) => return Err(Error::Corrupted {}),
            None => (),
        }

        match self.raw_read(&StorageKey::RateControllerConfig).await? {
            Some(StorageValue::RateControllerConfig(rcc)) => cfg.rate_controller_config = Some(rcc),
            Some(_) => return Err(Error::Corrupted {}),
            None => (),
        }

        for (i, motor_config) in cfg.motors_config.iter_mut().enumerate() {
            match self
                .raw_read(&StorageKey::MotorConfig { index: i as u8 })
                .await?
            {
                Some(StorageValue::MotorConfig(mc)) => *motor_config = Some(mc),
                Some(_) => return Err(Error::Corrupted {}),
                None => (),
            }
        }
        for (i, servo_config) in cfg.servos_config.iter_mut().enumerate() {
            match self
                .raw_read(&StorageKey::ServoConfig { index: i as u8 })
                .await?
            {
                Some(StorageValue::ServoConfig(sc)) => *servo_config = Some(sc),
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
    ServoMaxThrow,
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
    ServoMaxThrow(u16),
    ServoConfig(ServoMixerConfig),
}

impl PostcardValue<'_> for StorageValue {}
