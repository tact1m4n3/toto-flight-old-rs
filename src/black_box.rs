use core::sync::atomic::{AtomicBool, Ordering};
use core::{fmt::Write, ops::ControlFlow};

use embassy_time::{Duration, Instant};
use embedded_hal::delay::DelayNs;
use embedded_hal::spi::SpiDevice;
use embedded_sdmmc::{RawDirectory, RawFile, SdCard, VolumeManager};
use heapless::String;

use crate::receiver;
use crate::sensors;

mod ardupilot_black_box_logger;
use ardupilot_black_box_logger as log;

const LOG_FILE_PREFIX: &str = "LOG_";
const MAX_LOG_ID: u16 = 999;
const LOG_FILE_EXTENSION: &str = "BIN";
const MAX_FILE_NAME_LEN: usize = LOG_FILE_PREFIX.len() + 3 + ".".len() + LOG_FILE_EXTENSION.len();
const _: () = assert!(MAX_FILE_NAME_LEN <= 11);

const LOG_RATE_PERIOD: Duration = Duration::from_hz(10);

static RECORDING: AtomicBool = AtomicBool::new(false);
static NEW_FILE_FLAG: AtomicBool = AtomicBool::new(false);

pub fn set_recording_enabled(enabled: bool) {
    RECORDING.store(enabled, Ordering::Relaxed);
}

pub fn begin_new_file() {
    NEW_FILE_FLAG.store(true, Ordering::Relaxed);
}

pub const MAX_DIRS: usize = 1;
pub const MAX_FILES: usize = 1;
pub const MAX_VOLUMES: usize = 1;

pub type Result<T> = core::result::Result<T, embedded_sdmmc::Error<embedded_sdmmc::SdCardError>>;

pub struct Storage<SPI: SpiDevice, D: DelayNs> {
    volume_manager:
        VolumeManager<SdCard<SPI, D>, DummyTimesource, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    root_dir: Option<RawDirectory>,
    active_file: Option<RawFile>,
    next_blackbox_log_id: u16,
    last_tick: Instant,
}

impl<SPI: SpiDevice, D: DelayNs> Storage<SPI, D> {
    pub fn new(sd_card: SdCard<SPI, D>) -> Result<Self> {
        let volume_manager = VolumeManager::new_with_limits(sd_card, DummyTimesource, 0);
        let volume = volume_manager.open_volume(embedded_sdmmc::VolumeIdx(0))?;
        let root_dir = volume.open_root_dir()?;

        let mut next_blackbox_log_id: u16 = 0;
        root_dir.iterate_dir(|entry| {
            if entry.name.extension() == LOG_FILE_EXTENSION.as_bytes()
                && let Ok(name) = str::from_utf8(entry.name.base_name())
                && let Some(Ok(id)) = name
                    .strip_prefix(LOG_FILE_PREFIX)
                    .map(|id_str| id_str.parse::<u16>())
                && next_blackbox_log_id <= id
            {
                next_blackbox_log_id = id + 1;
            }
            ControlFlow::Continue(())
        })?;

        info!("next_blackbox_log_id: {}", next_blackbox_log_id);

        let root_dir_raw = root_dir.to_raw_directory();
        _ = volume.to_raw_volume(); // NOTE: we could also close it but we don't actually care

        Ok(Self {
            volume_manager,
            root_dir: Some(root_dir_raw),
            active_file: None,
            next_blackbox_log_id,
            last_tick: Instant::MIN,
        })
    }

    pub fn poll(&mut self) -> Result<()> {
        let now = Instant::now();
        if now - self.last_tick < LOG_RATE_PERIOD {
            return Ok(());
        }
        self.last_tick = now;

        if !RECORDING.load(Ordering::Relaxed) {
            return Ok(());
        }

        if NEW_FILE_FLAG.swap(false, Ordering::Relaxed)
            && let Some(raw_file) = self.active_file.take()
        {
            let file = raw_file.to_file(&self.volume_manager);
            file.close()?;
        }

        if self.active_file.is_none() {
            let log_id = self.next_blackbox_log_id;
            if log_id > MAX_LOG_ID {
                warn!("max log id reached, not creating new black box log");
                // TODO: use a custom error here
                return Err(embedded_sdmmc::Error::DiskFull);
            }
            self.next_blackbox_log_id += 1;

            let mut log_file_name = String::<MAX_FILE_NAME_LEN>::new();
            write!(
                log_file_name,
                "{}{}.{}",
                LOG_FILE_PREFIX, log_id, LOG_FILE_EXTENSION
            )
            .unwrap();

            let root_dir = self
                .root_dir
                .take()
                .unwrap()
                .to_directory(&self.volume_manager);

            let mut file = root_dir.open_file_in_dir(
                log_file_name.as_str(),
                embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
            )?;

            log::write_header(&mut file)?;

            self.active_file = Some(file.to_raw_file());
            self.root_dir = Some(root_dir.to_raw_directory());
        }

        match self.do_log(now) {
            Ok(()) => Ok(()),
            // means we exceeded 4GB file in which case we just create another file
            Err(embedded_sdmmc::Error::MaxFileSizeExceeded) => {
                info!("log file size limit reached, creating new log file");
                Ok(())
            }
            Err(err) => Err(err),
        }
    }

    /// Assumes there is an active file.
    fn do_log(&mut self, now: Instant) -> Result<()> {
        let mut file = self
            .active_file
            .take()
            .unwrap()
            .to_file(&self.volume_manager);

        let timestamp_us = now.as_micros();

        {
            let gyro = sensors::imu::DATA.lock(|data| data.borrow().gyro);
            log::write_gyro(&mut file, timestamp_us, gyro)?;
        }

        {
            let accel = sensors::imu::DATA.lock(|data| data.borrow().accel);
            log::write_accel(&mut file, timestamp_us, accel)?;
        }

        {
            let gps = sensors::gps::DATA.lock(|data| *data.borrow());
            log::write_gps(
                &mut file,
                timestamp_us,
                gps.fix_type as u8,
                gps.num_sat,
                gps.hdop,
                gps.latitude,
                gps.longitude,
                gps.altitude,
                gps.speed,
                gps.heading,
                gps.vertical_velocity,
            )?;
        }

        {
            let battery = sensors::battery::DATA.lock(|data| *data.borrow());
            log::write_bat(
                &mut file,
                timestamp_us,
                battery.voltage as f32,
                battery.current as f32,
                battery.capacity_used as f32,
            )?;
        }

        {
            let attitude = sensors::fusion::DATA.lock(|data| data.borrow().attitude);
            let (roll, pitch, yaw) = attitude.euler_angles();
            log::write_attitude(
                &mut file,
                timestamp_us,
                roll.to_degrees(),
                pitch.to_degrees(),
                yaw.to_degrees(),
            )?;
        }

        {
            let channels = receiver::STATE.lock(|state| *state.borrow().channels());
            log::write_rc_channels(&mut file, timestamp_us, &channels)?;
        }

        {
            let stats = receiver::STATE.lock(|state| *state.borrow().link_stats());
            log::write_rc_link_stats(
                &mut file,
                timestamp_us,
                stats.uplink_link_quality,
                stats.uplink_rssi,
                stats.downlink_link_quality,
                stats.downlink_rssi,
            )?;
        }

        file.flush()?;

        self.active_file = Some(file.to_raw_file());

        Ok(())
    }
}

pub struct DummyTimesource;

impl embedded_sdmmc::TimeSource for DummyTimesource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}
