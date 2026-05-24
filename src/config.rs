use core::ops::Range;

use embassy_sync::signal::Signal;
use embedded_storage_async::nor_flash::NorFlash;
use nalgebra::{Matrix3, Vector3};
use sequential_storage::{
    cache::NoCache,
    map::{Key, MapConfig, MapStorage, PostcardValue, SerializationError},
};

pub use sequential_storage::Error;

use crate::{
    RawMutex,
    utils::{NalgebraMatrix3Ext, NalgebraVec3Ext, SyncWatch},
};

const BUFFER_SIZE: usize = 64;

pub static BOARD_FORWARD_ANGLE: SyncWatch<RawMutex, f32> = SyncWatch::new(0.0);
pub static IMU_CALIBRATION: SyncWatch<RawMutex, ImuCalibration> =
    SyncWatch::new(ImuCalibration::DEFAULT);
pub static BOARD_ORIENTATION: SyncWatch<RawMutex, Matrix3<f32>> = SyncWatch::new(Matrix3::IDENTITY);
pub static RATE_CONTROLLER_CONFIG: SyncWatch<RawMutex, RateControllerConfig> =
    SyncWatch::new(RateControllerConfig::DEFAULT);
// TODO: Find a way to initialize a const array with the same value. See core::array::from_fn.
// Maybe use a macro.
pub static SERVOS_CONFIG: [SyncWatch<RawMutex, ServoConfig>; 2] = [
    SyncWatch::new(ServoConfig::DEFAULT),
    SyncWatch::new(ServoConfig::DEFAULT),
];
static COMMIT: Signal<RawMutex, ()> = Signal::new();

pub fn commit() {
    COMMIT.signal(());
}

pub struct Storage<F: NorFlash> {
    inner: MapStorage<StorageKey, F, NoCache>,
}

impl<F: NorFlash> Storage<F> {
    pub async fn new(flash: F, range: Range<u32>) -> Result<Storage<F>, Error<F::Error>> {
        let mut this = Self {
            inner: MapStorage::new(flash, MapConfig::new(range), NoCache),
        };

        if this.init().await.is_err() {
            this.inner.erase_all().await?;
            this.init().await?;
        }

        Ok(this)
    }

    pub async fn run(&mut self) {
        let mut board_forward_angle_watcher = BOARD_FORWARD_ANGLE.watcher();
        let mut imu_calibration_watcher = IMU_CALIBRATION.watcher();
        let mut board_orientation_watcher = BOARD_ORIENTATION.watcher();
        let mut rate_controller_config_watcher = RATE_CONTROLLER_CONFIG.watcher();
        let mut servo_config_watchers: [_; SERVOS_CONFIG.len()] =
            core::array::from_fn(|i| SERVOS_CONFIG[i].watcher());

        loop {
            // TODO: Do this only when arming is disabled
            COMMIT.wait().await;

            if let Some(board_forward_angle) = board_forward_angle_watcher.get_if_changed() {
                self.write(
                    &StorageKey::BoardForwardAngle,
                    &StorageValue::BoardForwardAngle(board_forward_angle),
                )
                .await
                .ok();
            }

            if let Some(imu_calibration) = imu_calibration_watcher.get_if_changed() {
                self.write(
                    &StorageKey::ImuCalibration,
                    &StorageValue::ImuCalibration(imu_calibration),
                )
                .await
                .ok();
            }

            if let Some(board_orientation) = board_orientation_watcher.get_if_changed() {
                self.write(
                    &StorageKey::BoardOrientation,
                    &StorageValue::BoardOrientation(board_orientation),
                )
                .await
                .ok();
            }

            if let Some(rate_controller_config) = rate_controller_config_watcher.get_if_changed() {
                self.write(
                    &StorageKey::RateControllerConfig,
                    &StorageValue::RateControllerConfig(rate_controller_config),
                )
                .await
                .ok();
            }

            for (i, watcher) in servo_config_watchers.iter_mut().enumerate() {
                if let Some(servo_config) = watcher.get_if_changed() {
                    self.write(
                        &StorageKey::ServoConfig { index: i as u8 },
                        &StorageValue::ServoConfig(servo_config),
                    )
                    .await
                    .ok();
                }
            }
        }
    }

    async fn init(&mut self) -> Result<(), Error<F::Error>> {
        match self.read(&StorageKey::BoardForwardAngle).await? {
            Some(StorageValue::BoardForwardAngle(bfa)) => BOARD_FORWARD_ANGLE.set(bfa),
            Some(_) => return Err(Error::Corrupted {}), // If this configuration has the wrong type
            None => (),
        };

        match self.read(&StorageKey::ImuCalibration).await? {
            Some(StorageValue::ImuCalibration(imu_calib)) => IMU_CALIBRATION.set(imu_calib),
            Some(_) => return Err(Error::Corrupted {}), // If this configuration has the wrong type
            None => (),
        };

        match self.read(&StorageKey::RateControllerConfig).await? {
            Some(StorageValue::RateControllerConfig(rcc)) => RATE_CONTROLLER_CONFIG.set(rcc),
            Some(_) => return Err(Error::Corrupted {}), // If this configuration has the wrong type
            None => (),
        };

        for (i, watch) in SERVOS_CONFIG.iter().enumerate() {
            match self
                .read(&StorageKey::ServoConfig { index: i as u8 })
                .await?
            {
                Some(StorageValue::ServoConfig(sc)) => watch.set(sc),
                Some(_) => return Err(Error::Corrupted {}), // If this configuration has the wrong type
                None => (),
            };
        }

        Ok(())
    }

    async fn read(&mut self, key: &StorageKey) -> Result<Option<StorageValue>, Error<F::Error>> {
        let mut buffer = [0u8; BUFFER_SIZE];
        self.inner.fetch_item(&mut buffer, key).await
    }

    async fn write(
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

impl ImuCalibration {
    const DEFAULT: ImuCalibration = ImuCalibration {
        gyro_bias: Vector3::ZEROS,
        accel_bias: Vector3::ZEROS,
        accel_scale: Vector3::ONES,
    };
}

#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct RateControllerConfig {
    roll: RateControllerAxisConfig,
    pitch: RateControllerAxisConfig,
    yaw: RateControllerAxisConfig,
}

impl RateControllerConfig {
    const DEFAULT: RateControllerConfig = RateControllerConfig {
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
    };
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

impl ServoConfig {
    const DEFAULT: ServoConfig = ServoConfig {
        mid_point_us: 1500,
        min_us: 1000,
        max_us: 2000,
        mixer_gain: Vector3::ZEROS,
    };
}
