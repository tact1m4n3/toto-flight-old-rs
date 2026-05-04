use core::{
    fmt::Write as FmtWrite,
    sync::atomic::{AtomicBool, Ordering},
};

use block_device_adapters::BufStreamError;
use embassy_futures::select::{Either, select};
use embassy_rp::{gpio, peripherals::SPI0, spi};
use embassy_time::{Delay, Duration, Ticker};
use embedded_fatfs::FsOptions;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_io_async::Write;
use heapless::String;

use crate::{
    Irqs, SdCardPeripherals, SystemSpawner, black_box::ardupilot_black_box_format::Logger,
    nav_loop, pid_loop, receiver, sensors::imu, signals,
};

mod ardupilot_black_box_format;

type SpiBus = spi::Spi<'static, SPI0, spi::Async>;
type CsPin = gpio::Output<'static>;
type SpiDevice = ExclusiveDevice<SpiBus, CsPin, Delay>;
type SdCard = sdspi::SdSpi<SpiDevice, Delay, aligned::A1>;
type BufStream = block_device_adapters::BufStream<SdCard, 512>;
type FileSystem = embedded_fatfs::FileSystem<
    BufStream,
    embedded_fatfs::NullTimeProvider,
    embedded_fatfs::LossyOemCpConverter,
>;
type Dir<'a> = embedded_fatfs::Dir<
    'a,
    BufStream,
    embedded_fatfs::NullTimeProvider,
    embedded_fatfs::LossyOemCpConverter,
>;
type File<'a> = embedded_fatfs::File<
    'a,
    BufStream,
    embedded_fatfs::NullTimeProvider,
    embedded_fatfs::LossyOemCpConverter,
>;

pub type Error = embedded_fatfs::Error<BufStreamError<sdspi::Error>>;
pub type Result<T> = core::result::Result<T, Error>;

const LOG_FILE_PREFIX: &str = "LOG_";
const MAX_LOG_ID: u32 = 999;
const LOG_FILE_EXTENSION: &str = "BIN";
const MAX_FILE_NAME_LEN: usize = LOG_FILE_PREFIX.len() + 3 + ".".len() + LOG_FILE_EXTENSION.len();
const _: () = assert!(MAX_FILE_NAME_LEN <= 11);
const LOG_RATE_HZ: u64 = 1000;

static RECORD: AtomicBool = AtomicBool::new(false);
signals! {
    NEW_FILE: ();
}

pub async fn init(spawner: SystemSpawner, p: SdCardPeripherals) -> Result<()> {
    let mut config = spi::Config::default();
    config.frequency = 100_000;
    let mut spi = spi::Spi::new(
        p.spi, p.clk_pin, p.mosi_pin, p.miso_pin, p.tx_dma, p.rx_dma, Irqs, config,
    );
    let mut spi_cs = gpio::Output::new(p.cs_pin, gpio::Level::High);

    sdspi::sd_init(&mut spi, &mut spi_cs).await.unwrap(); // NOTE: can't fail
    let spi_dev = ExclusiveDevice::new(spi, spi_cs, Delay).unwrap(); // NOTE: can't fail
    let mut sd_card = SdCard::new(spi_dev, Delay);
    sd_card
        .init()
        .await
        .map_err(|err| Error::Io(BufStreamError::Io(err)))?;

    // up the frequency after initialization
    let mut config = spi::Config::default();
    config.frequency = 20_000_000;
    sd_card.spi().bus_mut().set_config(&config);

    let fs = FileSystem::new(BufStream::new(sd_card), FsOptions::new()).await?;

    spawner.low_prio.spawn(task(fs).unwrap());

    Ok(())
}

pub fn set_recording_enabled(enable: bool) {
    RECORD.store(enable, Ordering::Release);
}

pub fn begin_new_file() {
    NEW_FILE.signal(());
}

#[embassy_executor::task]
async fn task(fs: FileSystem) {
    let root = fs.root_dir();

    let mut next_log_id = match find_next_black_box_log_id(root.clone()).await {
        Ok(next_log_id) => next_log_id,
        Err(err) => {
            defmt::warn!("failed to read file system: {}", err);
            return;
        }
    };

    // to not create a new useless file on every boot
    NEW_FILE.wait().await;

    loop {
        if next_log_id > MAX_LOG_ID {
            defmt::warn!("black box log files limit reached... how did you do that?");
            return;
        }

        let file = match create_log_file(root.clone(), next_log_id).await {
            Ok(file) => file,
            Err(err) => {
                defmt::warn!("failed to create log file: {}", err);
                return;
            }
        };

        match log_loop(file).await {
            Ok(()) => (),
            // means we exceeded 4GB file in which case we just create another file
            Err(Error::WriteZero) => (),
            Err(err) => {
                defmt::warn!("unexpected error while writing the log file: {}", err);
                return;
            }
        }

        next_log_id += 1;
    }
}

async fn log_loop(file: File<'_>) -> Result<()> {
    let mut l = Logger { writer: file };
    l.write_header().await?;

    defmt::debug!("beginning new log file");

    let t = embassy_time::Instant::now().as_micros();
    let gyro_bias = imu::get_gyro_bias();
    let accel_bias = imu::get_accel_bias();
    l.write_parm(t, "GYRO_BIAS_X", gyro_bias.x as f64).await?;
    l.write_parm(t, "GYRO_BIAS_Y", gyro_bias.y as f64).await?;
    l.write_parm(t, "GYRO_BIAS_Z", gyro_bias.z as f64).await?;
    l.write_parm(t, "ACCEL_BIAS_X", accel_bias.x as f64).await?;
    l.write_parm(t, "ACCEL_BIAS_Y", accel_bias.y as f64).await?;
    l.write_parm(t, "ACCEL_BIAS_Z", accel_bias.z as f64).await?;

    let mut gyro_watcher = imu::GYRO.watcher();
    let mut accel_watcher = imu::ACCEL.watcher();
    let mut motor_watcher = pid_loop::MOTOR_OUTPUT.watcher();
    let mut target_watcher = nav_loop::TARGET.watcher();
    let mut channels_watcher = receiver::CHANNELS.watcher();
    let mut link_stats_watcher = receiver::LINK_STATS.watcher();
    let mut multimeter_watcher = crate::sensors::multimeter::DATA.watcher();

    let mut ticker = Ticker::every(Duration::from_hz(LOG_RATE_HZ));
    loop {
        match select(NEW_FILE.wait(), ticker.next()).await {
            Either::First(()) => break,
            Either::Second(()) => (),
        }

        if let Some((gyro, timestamp)) = gyro_watcher.get_if_changed()
            && are_we_recording()
        {
            l.write_gyro(timestamp.as_micros(), gyro).await?;
        }

        if let Some((accel, timestamp)) = accel_watcher.get_if_changed()
            && are_we_recording()
        {
            l.write_accel(timestamp.as_micros(), accel).await?;
        }

        if let Some((motor, _timestamp)) = motor_watcher.get_if_changed()
            && are_we_recording()
        {
            l.write_motor_output(
                embassy_time::Instant::now().as_micros(),
                motor.throttle,
                motor.left_elevon,
                motor.right_elevon,
            )
            .await?;
        }

        if let Some((target, _timestamp)) = target_watcher.get_if_changed()
            && are_we_recording()
        {
            if let Some(t) = target {
                l.write_nav_target(
                    embassy_time::Instant::now().as_micros(),
                    t.throttle,
                    (t.rate.x, t.rate.y, t.rate.z),
                )
                .await?;
            }
        }

        if let Some((channels, timestamp)) = channels_watcher.get_if_changed()
            && are_we_recording()
        {
            l.write_rc_channels(timestamp.as_micros(), &channels).await?;
        }

        if let Some((stats, timestamp)) = link_stats_watcher.get_if_changed()
            && are_we_recording()
        {
            l.write_rc_link_stats(
                timestamp.as_micros(),
                stats.uplink_link_quality,
                stats.uplink_rssi,
                stats.downlink_link_quality,
                stats.downlink_rssi,
            )
            .await?;
        }

        if let Some((data, timestamp)) = multimeter_watcher.get_if_changed()
            && are_we_recording()
        {
            l.write_bat(timestamp.as_micros(), data.voltage, data.current, data.total_current)
                .await?;
        }

        l.writer.flush().await?;
    }

    Ok(())
}

async fn find_next_black_box_log_id(root: Dir<'_>) -> Result<u32> {
    let mut iter = root.iter();
    let mut next_log_id: u32 = 0;
    while let Some(entry) = iter.next().await {
        let entry = entry?;
        let Ok(name) = str::from_utf8(entry.short_file_name_as_bytes()) else {
            continue;
        };
        let Some((base, ext)) = name.split_once('.') else {
            continue;
        };
        if ext != LOG_FILE_EXTENSION {
            continue;
        }
        if let Some(Ok(id)) = base
            .strip_prefix(LOG_FILE_PREFIX)
            .map(|id_str| id_str.parse::<u32>())
        {
            if id >= next_log_id {
                next_log_id = id + 1;
            }
        }
    }
    Ok(next_log_id)
}

async fn create_log_file(root: Dir<'_>, log_id: u32) -> Result<File<'_>> {
    let mut log_file_name = String::<MAX_FILE_NAME_LEN>::new();
    log_file_name.clear();
    log_file_name
        .write_fmt(format_args!(
            "{}{}.{}",
            LOG_FILE_PREFIX, log_id, LOG_FILE_EXTENSION
        ))
        .unwrap();

    let mut file = root.create_file(&log_file_name).await?;
    file.truncate().await?;
    Ok(file)
}

fn are_we_recording() -> bool {
    RECORD.load(Ordering::Relaxed)
}
