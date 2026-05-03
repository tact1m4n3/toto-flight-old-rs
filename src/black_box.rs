use core::{
    fmt::Write as FmtWrite,
    sync::atomic::{AtomicBool, Ordering},
};

use block_device_adapters::BufStreamError;
use embassy_rp::{gpio, peripherals::SPI0, spi};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pipe::Pipe};
use embassy_time::{Delay, Instant};
use embedded_fatfs::FsOptions;
use embedded_hal_bus::spi::ExclusiveDevice;
use futures::FutureExt;
use nalgebra::Vector3;
use embedded_io_async::Write;

use crate::{Irqs, SdCardPeripherals, SystemSpawner, black_box::logger::Logger, signals};

mod logger;

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

const MESSAGE_QUEUE_LEN: usize = 64;

static RECORD: AtomicBool = AtomicBool::new(false);
static MESSAGE_QUEUE: Pipe<CriticalSectionRawMutex, MESSAGE_QUEUE_LEN> = Pipe::new();
signals! {
    NEXT_FILE: ();
    GYRO: (Vector3<f32>, Instant);
    ACCEL: (Vector3<f32>, Instant);
}

pub fn set_recording_enabled(enable: bool) {
    RECORD.store(enable, Ordering::Release);
}

pub fn write_message(msg: &str) -> bool {
    MESSAGE_QUEUE.try_write(msg.as_bytes()).is_ok()
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
    config.frequency = 25_000_000;
    sd_card.spi().bus_mut().set_config(&config);

    let fs = FileSystem::new(BufStream::new(sd_card), FsOptions::new()).await?;

    spawner.low_prio.spawn(task(fs).unwrap());

    Ok(())
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

    loop {
        if next_log_id > MAX_LOG_ID {
            defmt::warn!("black box log files limit reached... how did you do that?");
            return;
        }

        if let Err(err) = create_log_file(root.clone(), next_log_id).await {
            defmt::warn!("failed to create log file: {}", err);
            return;
        }

        match log_loop(root.clone(), next_log_id).await {
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

async fn log_loop(root: Dir<'_>, log_id: u32) -> Result<()> {
    let file = create_log_file(root, log_id).await?;
    let mut l = Logger { writer: file };
    l.write_header().await?;

    defmt::debug!("beginning new log file with id: {}", log_id);

    let mut msg_buf: [u8; MESSAGE_QUEUE_LEN] = [0; MESSAGE_QUEUE_LEN];
    loop {
        futures::select_biased! {
            _ = NEXT_FILE.wait().fuse() => break,
            (gyro, timestamp) = GYRO.wait().fuse() => if are_we_recording() { l.write_gyro(timestamp.as_micros(), gyro).await? },
            (accel, timestamp) = ACCEL.wait().fuse() => if are_we_recording() { l.write_accel(timestamp.as_micros(), accel).await? },
            n = MESSAGE_QUEUE.read(&mut msg_buf).fuse() => {
                // TODO: maybe we should split messages into separate packets?
                let msg = core::str::from_utf8(&msg_buf[..n]).unwrap();
                let time_us = Instant::now().as_micros();
                l.write_message(time_us, msg).await?;
            },
        };
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
    let mut log_file_name = heapless::String::<MAX_FILE_NAME_LEN>::new();
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
