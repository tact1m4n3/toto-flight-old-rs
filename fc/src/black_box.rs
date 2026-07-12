use core::fmt::Write as FmtWrite;

use embassy_time::{Duration, Instant, Ticker};
use embedded_io_async::Write;
use fast_fat32::{FileSystem, FsError, ReadWriteSeek, ShortFileName};
use heapless::String;
use uom::si::acceleration::meter_per_second_squared;
use uom::si::angle::degree;
use uom::si::angular_velocity::radian_per_second;
use uom::si::electric_charge::milliampere_hour;
use uom::si::electric_current::ampere;
use uom::si::electric_potential::volt;
use uom::si::length::meter;
use uom::si::velocity::meter_per_second;

use crate::sensors;
use crate::{control, receiver};

mod ardupilot_black_box_logger;
use ardupilot_black_box_logger as log;

pub async fn task<S: ReadWriteSeek>(storage: S) -> Result<(), FsError<S::Error>> {
    const LOG_FILE_PREFIX: &str = "LOG_";
    const MAX_LOG_ID: u16 = 999;
    const LOG_FILE_EXTENSION: &str = "BIN";
    const MAX_FILE_NAME_LEN: usize =
        LOG_FILE_PREFIX.len() + 3 + ".".len() + LOG_FILE_EXTENSION.len();
    const _: () = assert!(MAX_FILE_NAME_LEN <= 11);

    const MIN_ALLOC_FILE_SIZE: u32 = 100 * 1024 * 1024; // 10 MB
    const MAX_ALLOC_FILE_SIZE: u32 = 1024 * 1024 * 1024; // 1 GB

    const LOG_PERIOD: Duration = Duration::from_hz(10);

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

    let mut logging = false;

    let mut imu_watcher = sensors::imu::DATA.watcher();
    let mut gps_watcher = sensors::gps::DATA.watcher();
    let mut battery_watcher = sensors::battery::DATA.watcher();
    let mut attitude_watcher = sensors::fusion::ATTITUDE.watcher();
    let mut link_stats_watcher = receiver::LINK_STATS.watcher();

    let mut ticker = Ticker::every(LOG_PERIOD);
    loop {
        ticker.next().await;

        let status = control::status();
        if !logging && status.arm_state.is_armed() {
            logging = true;

            log::write_header(&mut file_writer).await?;
        } else if logging && !status.arm_state.is_armed() {
            logging = false;

            let log_id = next_blackbox_log_id;
            if log_id > MAX_LOG_ID {
                warn!("max log id reached, not creating new black box log");
                // TODO: use a custom error here
                return Err(FsError::OutOfSpace);
            }
            next_blackbox_log_id += 1;

            let mut log_file_name = String::<MAX_FILE_NAME_LEN>::new();
            write!(
                log_file_name,
                "{}{}.{}",
                LOG_FILE_PREFIX, log_id, LOG_FILE_EXTENSION
            )
            .unwrap();
            let log_file_name = ShortFileName::new(log_file_name.as_bytes()).unwrap();
            file_writer.save(&root_dir, log_file_name).await?;
        }

        let timestamp_us = Instant::now().as_micros();

        if let Some(imu) = imu_watcher.get_if_changed() {
            log::write_gyro(
                &mut file_writer,
                timestamp_us,
                imu.gyro.get::<radian_per_second>(),
            )
            .await?;
            log::write_accel(
                &mut file_writer,
                timestamp_us,
                imu.accel.get::<meter_per_second_squared>(),
            )
            .await?;
        }

        if let Some(gps) = gps_watcher.get_if_changed() {
            log::write_gps(
                &mut file_writer,
                timestamp_us,
                gps.fix_type as u8,
                gps.num_sat,
                gps.hdop,
                gps.latitude.get::<degree>(),
                gps.longitude.get::<degree>(),
                gps.altitude.get::<meter>(),
                gps.velocity_over_ground.get::<meter_per_second>(),
                gps.heading.get::<degree>(),
                gps.vertical_velocity.get::<meter_per_second>(),
            )
            .await?;
        }

        if let Some(battery) = battery_watcher.get_if_changed() {
            log::write_bat(
                &mut file_writer,
                timestamp_us,
                battery.voltage.get::<volt>(),
                battery.current.get::<ampere>(),
                battery.capacity_used.get::<milliampere_hour>(),
            )
            .await?;
        }

        if let Some(attitude) = attitude_watcher.get_if_changed() {
            let (roll, pitch, yaw) = attitude.euler_angles();
            log::write_attitude(
                &mut file_writer,
                timestamp_us,
                roll.to_degrees(),
                pitch.to_degrees(),
                yaw.to_degrees(),
            )
            .await?;
        }

        if let Some(stats) = link_stats_watcher.get_if_changed() {
            log::write_rc_link_stats(
                &mut file_writer,
                timestamp_us,
                stats.uplink_link_quality,
                stats.uplink_rssi,
                stats.downlink_link_quality,
                stats.downlink_rssi,
            )
            .await?;
        }

        file_writer.flush().await?;
    }
}
