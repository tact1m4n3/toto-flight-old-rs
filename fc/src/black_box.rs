use core::{fmt::Write, ops::ControlFlow};

use embassy_time::{Duration, Instant, Ticker};
use embedded_hal::delay::DelayNs;
use embedded_hal::spi::SpiDevice;
use fast_fat32::{DirEntry, FileSystem, FileWriter, FsError, ReadWriteSeek};
use heapless::String;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angle::degree;
use uom::si::angular_velocity::radian_per_second;
use uom::si::electric_charge::milliampere_hour;
use uom::si::electric_current::ampere;
use uom::si::electric_potential::volt;
use uom::si::length::meter;
use uom::si::velocity::meter_per_second;

use crate::{RawMutex, sensors};
use crate::{control, receiver, utils::SyncWatchReceiver};

mod ardupilot_black_box_logger;
use ardupilot_black_box_logger as log;

const LOG_FILE_PREFIX: &str = "LOG_";
const MAX_LOG_ID: u16 = 999;
const LOG_FILE_EXTENSION: &str = "BIN";
const MAX_FILE_NAME_LEN: usize = LOG_FILE_PREFIX.len() + 3 + ".".len() + LOG_FILE_EXTENSION.len();
const _: () = assert!(MAX_FILE_NAME_LEN <= 11);

const MIN_ALLOC_FILE_SIZE: u32 = 100 * 1024 * 1024; // 10 MB
const MAX_ALLOC_FILE_SIZE: u32 = 1024 * 1024 * 1024; // 1 GB

const LOG_RATE_PERIOD: Duration = Duration::from_hz(10);
const STOP_RECORDING_DELAY: Duration = Duration::from_secs(2);

pub async fn task<S: ReadWriteSeek>(storage: S) -> Result<(), FsError<S::Error>> {
    let mut fs = FileSystem::new(storage).await?;
    let root_dir = fs.root_dir();

    let mut next_blackbox_log_id: u16 = 0;
    {
        let mut it = fs.iter_dir(&root_dir);
        while let Some(dir_entry) = it.next().await? {
            let name = dir_entry.name();
            if name.ext() == LOG_FILE_EXTENSION.as_bytes()
                && let Ok(name) = str::from_utf8(name.base_name())
                && let Some(Ok(id)) = name
                    .strip_prefix(LOG_FILE_PREFIX)
                    .map(|id_str| id_str.parse::<u16>())
                && next_blackbox_log_id <= id
            {
                next_blackbox_log_id = id + 1;
            }
        }
    }

    info!("next_blackbox_log_id: {}", next_blackbox_log_id);

    let alloc_file = fs
        .get_or_create_cluster_allocator_file(10, MIN_ALLOC_FILE_SIZE..=MAX_ALLOC_FILE_SIZE)
        .await?;

    let mut file_writer = fs.file_writer(alloc_file);

    let mut ticker = Ticker::every(LOG_RATE_PERIOD);
    loop {
        ticker.next().await;
    }
}

// pub struct BlackBox<S: ReadWriteSeek> {
//     file_writer: FileWriter<S>,

//     next_blackbox_log_id: u16,

//     logging: bool,
//     file_close_request_instant: Option<Instant>,
// }

// impl<S: ReadWriteSeek> BlackBox<S> {
//     pub async fn new(storage: S) -> Result<Self, FsError<S::Error>> {
//         let mut fs = FileSystem::new(storage).await?;
//         let root_dir = fs.root_dir();

//         let mut next_blackbox_log_id: u16 = 0;
//         {
//             let mut it = fs.iter_dir(&root_dir);
//             while let Some(dir_entry) = it.next().await? {
//                 let name = dir_entry.name();
//                 if name.ext() == LOG_FILE_EXTENSION.as_bytes()
//                     && let Ok(name) = str::from_utf8(name.base_name())
//                     && let Some(Ok(id)) = name
//                         .strip_prefix(LOG_FILE_PREFIX)
//                         .map(|id_str| id_str.parse::<u16>())
//                     && next_blackbox_log_id <= id
//                 {
//                     next_blackbox_log_id = id + 1;
//                 }
//             }
//         }

//         info!("next_blackbox_log_id: {}", next_blackbox_log_id);

//         let alloc_file = fs
//             .get_or_create_cluster_allocator_file(10, MIN_ALLOC_FILE_SIZE..=MAX_ALLOC_FILE_SIZE)
//             .await?;

//         let file_writer = fs.file_writer(alloc_file);

//         Ok(Self {
//             file_writer,

//             next_blackbox_log_id,

//             logging: false,
//             file_close_request_instant: None,
//         })
//     }

//     pub async fn run(&mut self) -> Result<(), FsError<S::Error>> {
//         let mut ticker = Ticker::every(LOG_RATE_PERIOD);

//         loop {
//             ticker.next().await;

//             let status = control::status();

//             if status.arm_state.is_armed() && !self.logging {
//                 self.logging = true;
//                 info!("logging started");

//                 let log_id = self.next_blackbox_log_id;
//                 if log_id > MAX_LOG_ID {
//                     warn!("max log id reached, not creating new black box log");
//                     // TODO: use a custom error here
//                     return Err(FsError::OutOfSpace);
//                 }
//                 self.next_blackbox_log_id += 1;

//                 let mut log_file_name = String::<MAX_FILE_NAME_LEN>::new();
//                 write!(
//                     log_file_name,
//                     "{}{}.{}",
//                     LOG_FILE_PREFIX, log_id, LOG_FILE_EXTENSION
//                 )
//                 .unwrap();

//                 let mut file = root_dir.open_file_in_dir(
//                     log_file_name.as_str(),
//                     embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
//                 )?;

//                 log::write_header(&mut file)?;

//                 active_file = true;
//             }

//             if status.arm_state.is_armed() {
//                 self.file_close_request_instant = None;
//             } else if let Some(file_close_request_instant) = self.file_close_request_instant {
//                 if now - file_close_request_instant >= STOP_RECORDING_DELAY {
//                     self.file_close_request_instant = None;
//                     if let Some(raw_file) = self.active_file.take() {
//                         let file = raw_file.to_file(&self.volume_manager);
//                         file.close()?;
//                     }
//                 }
//             } else if (!status.arm_state.is_armed() || status.failsafe)
//                 && self.active_file.is_some()
//             {
//                 self.file_close_request_instant = Some(now);
//             }

//             match self.do_log(now) {
//                 Ok(()) => Ok(()),
//                 // means we exceeded 4GB file in which case we just create another file
//                 Err(embedded_sdmmc::Error::MaxFileSizeExceeded) => {
//                     info!("log file size limit reached, creating new log file");
//                     Ok(())
//                 }
//                 Err(err) => Err(err),
//             }
//         }
//     }

//     fn do_log(&mut self, now: Instant) -> Result<()> {
//         let Some(raw_file) = self.active_file.take() else {
//             return Ok(());
//         };
//         let mut file = raw_file.to_file(&self.volume_manager);

//         let timestamp_us = now.as_micros();

//         if let Some(imu) = self.imu_watcher.get_if_changed() {
//             log::write_gyro(&mut file, timestamp_us, imu.gyro.get::<radian_per_second>())?;
//             log::write_accel(
//                 &mut file,
//                 timestamp_us,
//                 imu.accel.get::<meter_per_second_squared>(),
//             )?;
//         }

//         if let Some(gps) = self.gps_watcher.get_if_changed() {
//             log::write_gps(
//                 &mut file,
//                 timestamp_us,
//                 gps.fix_type as u8,
//                 gps.num_sat,
//                 gps.hdop,
//                 gps.latitude.get::<degree>(),
//                 gps.longitude.get::<degree>(),
//                 gps.altitude.get::<meter>(),
//                 gps.velocity_over_ground.get::<meter_per_second>(),
//                 gps.heading.get::<degree>(),
//                 gps.vertical_velocity.get::<meter_per_second>(),
//             )?;
//         }

//         if let Some(battery) = self.battery_watcher.get_if_changed() {
//             log::write_bat(
//                 &mut file,
//                 timestamp_us,
//                 battery.voltage.get::<volt>(),
//                 battery.current.get::<ampere>(),
//                 battery.capacity_used.get::<milliampere_hour>(),
//             )?;
//         }

//         if let Some(attitude) = self.attitude_watcher.get_if_changed() {
//             let (roll, pitch, yaw) = attitude.euler_angles();
//             log::write_attitude(
//                 &mut file,
//                 timestamp_us,
//                 roll.to_degrees(),
//                 pitch.to_degrees(),
//                 yaw.to_degrees(),
//             )?;
//         }

//         if let Some(channels) = self.channels_watcher.get_if_changed() {
//             log::write_rc_channels(&mut file, timestamp_us, channels.as_slice())?;
//         }

//         if let Some(stats) = self.link_stats_watcher.get_if_changed() {
//             log::write_rc_link_stats(
//                 &mut file,
//                 timestamp_us,
//                 stats.uplink_link_quality,
//                 stats.uplink_rssi,
//                 stats.downlink_link_quality,
//                 stats.downlink_rssi,
//             )?;
//         }

//         file.flush()?;

//         self.active_file = Some(file.to_raw_file());

//         Ok(())
//     }
// }

// pub struct DummyTimesource;

// impl embedded_sdmmc::TimeSource for DummyTimesource {
//     fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
//         embedded_sdmmc::Timestamp {
//             year_since_1970: 0,
//             zero_indexed_month: 0,
//             zero_indexed_day: 0,
//             hours: 0,
//             minutes: 0,
//             seconds: 0,
//         }
//     }
// }
