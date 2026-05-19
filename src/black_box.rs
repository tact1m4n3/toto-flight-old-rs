use core::sync::atomic::{AtomicBool, Ordering};
use core::{fmt::Write, ops::ControlFlow};

use embassy_rp::{gpio, peripherals::SPI0, spi};
use embassy_time::{Delay, Duration, Instant};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use embedded_sdmmc::sdcard::spi::AcquireOpts;
use embedded_sdmmc::{RawDirectory, RawFile};
use heapless::String;
use nalgebra::Vector3;

use crate::sensors::{gps, imu, multimeter};
use crate::utils::SyncWatchReader;
use crate::{SdCardPeripherals, receiver, sensors::fusion};

mod ardupilot_black_box_logger;
use ardupilot_black_box_logger as log;

const LOG_FILE_PREFIX: &str = "LOG_";
const MAX_LOG_ID: u16 = 999;
const LOG_FILE_EXTENSION: &str = "BIN";
const MAX_FILE_NAME_LEN: usize = LOG_FILE_PREFIX.len() + 3 + ".".len() + LOG_FILE_EXTENSION.len();
const _: () = assert!(MAX_FILE_NAME_LEN <= 11);

const LOG_RATE_HZ: u64 = 100;
const LOG_RATE_PERIOD: Duration = Duration::from_hz(LOG_RATE_HZ);

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

pub type SpiBus = spi::Spi<'static, SPI0, spi::Blocking>;
pub type SpiDevice = ExclusiveDevice<SpiBus, gpio::Output<'static>, NoDelay>;
pub type SdCard = embedded_sdmmc::SdCard<SpiDevice, Delay>;
/// We will have only one volume and directory opened with a max of two files opened:
/// config.toml and the current blackbox log file.
pub type VolumeManager =
    embedded_sdmmc::VolumeManager<SdCard, DummyTimesource, MAX_DIRS, MAX_FILES, MAX_VOLUMES>;

pub type Result<T> = core::result::Result<T, embedded_sdmmc::Error<embedded_sdmmc::SdCardError>>;

pub struct Storage {
    volume_manager: VolumeManager,
    root_dir: Option<RawDirectory>,
    active_file: Option<RawFile>,
    next_blackbox_log_id: u16,
    last_tick: Instant,

    gyro_watcher: SyncWatchReader<'static, Vector3<f32>>,
    accel_watcher: SyncWatchReader<'static, Vector3<f32>>,
    channels_watcher: SyncWatchReader<'static, [u16; 16]>,
    link_stats_watcher: SyncWatchReader<'static, receiver::LinkStats>,
    gps_watcher: SyncWatchReader<'static, gps::Data>,
    multimeter_watcher: SyncWatchReader<'static, multimeter::Data>,
    attitude_watcher: SyncWatchReader<'static, nalgebra::UnitQuaternion<f32>>,
}

impl Storage {
    pub fn new(p: SdCardPeripherals) -> Result<Self> {
        let mut config = spi::Config::default();
        config.frequency = 100_000;
        let spi = spi::Spi::new_blocking(p.spi, p.clk_pin, p.mosi_pin, p.miso_pin, config);
        let spi_cs = gpio::Output::new(p.cs_pin, gpio::Level::High);
        let spi_dev = ExclusiveDevice::new_no_delay(spi, spi_cs).unwrap(); // NOTE: can't fail

        let sdcard = SdCard::new_with_options(
            spi_dev,
            Delay,
            AcquireOpts {
                use_crc: true,
                acquire_retries: 1,
            },
        );

        // this also initializes the sdcard
        defmt::info!("sd card size: {}", sdcard.num_bytes()?);

        // up the frequency after initialization
        let mut config = spi::Config::default();
        config.frequency = 12_000_000;
        sdcard.spi(|dev| dev.bus_mut().set_config(&config));

        let volume_manager = VolumeManager::new_with_limits(sdcard, DummyTimesource, 0);
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

        defmt::info!("next_blackbox_log_id: {}", next_blackbox_log_id);

        let root_dir_raw = root_dir.to_raw_directory();
        _ = volume.to_raw_volume(); // NOTE: we could also close it but we don't actually care

        Ok(Self {
            volume_manager,
            root_dir: Some(root_dir_raw),
            active_file: None,
            next_blackbox_log_id,
            last_tick: Instant::MIN,

            gyro_watcher: imu::GYRO.watcher(),
            accel_watcher: imu::ACCEL.watcher(),
            channels_watcher: receiver::CHANNELS.watcher(),
            link_stats_watcher: receiver::LINK_STATS.watcher(),
            gps_watcher: gps::DATA.watcher(),
            multimeter_watcher: multimeter::DATA.watcher(),
            attitude_watcher: fusion::ATTITUDE.watcher(),
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
                defmt::warn!("max log id reached, not creating new black box log");
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

        match self.do_log() {
            Ok(()) => Ok(()),
            // means we exceeded 4GB file in which case we just create another file
            Err(embedded_sdmmc::Error::MaxFileSizeExceeded) => {
                defmt::info!("log file size limit reached, creating new log file");
                Ok(())
            }
            Err(err) => Err(err),
        }
    }

    /// Assumes there is an active file.
    fn do_log(&mut self) -> Result<()> {
        let mut file = self
            .active_file
            .take()
            .unwrap()
            .to_file(&self.volume_manager);

        if let Some((gyro, timestamp)) =
            critical_section::with(|cs| self.gyro_watcher.get_if_changed(cs))
        {
            log::write_gyro(&mut file, timestamp.as_micros(), gyro)?;
        }

        if let Some((accel, timestamp)) =
            critical_section::with(|cs| self.accel_watcher.get_if_changed(cs))
        {
            log::write_accel(&mut file, timestamp.as_micros(), accel)?;
        }

        if let Some((channels, timestamp)) =
            critical_section::with(|cs| self.channels_watcher.get_if_changed(cs))
        {
            log::write_rc_channels(&mut file, timestamp.as_micros(), &channels)?;
        }

        if let Some((stats, timestamp)) =
            critical_section::with(|cs| self.link_stats_watcher.get_if_changed(cs))
        {
            log::write_rc_link_stats(
                &mut file,
                timestamp.as_micros(),
                stats.uplink_link_quality,
                stats.uplink_rssi,
                stats.downlink_link_quality,
                stats.downlink_rssi,
            )?;
        }

        if let Some((data, timestamp)) =
            critical_section::with(|cs| self.gps_watcher.get_if_changed(cs))
        {
            log::write_gps(
                &mut file,
                timestamp.as_micros(),
                data.fix_type as u8,
                data.num_sat,
                data.hdop,
                data.latitude,
                data.longitude,
                data.altitude,
                data.speed,
                data.heading,
                data.vertical_velocity,
            )?;
        }

        if let Some((data, timestamp)) =
            critical_section::with(|cs| self.multimeter_watcher.get_if_changed(cs))
        {
            log::write_bat(
                &mut file,
                timestamp.as_micros(),
                data.voltage,
                data.current,
                data.capacity_used,
            )?
        }

        if let Some((quat, timestamp)) =
            critical_section::with(|cs| self.attitude_watcher.get_if_changed(cs))
        {
            // Convert quaternion to Euler angles (radians) and then to degrees
            let (roll, pitch, yaw) = quat.euler_angles();

            log::write_attitude(
                &mut file,
                timestamp.as_micros(),
                roll.to_degrees(),
                pitch.to_degrees(),
                yaw.to_degrees(),
            )?
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
