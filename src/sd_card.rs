use embassy_rp::{gpio, peripherals::SPI0, spi};
use embassy_time::Delay;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};

use crate::SdCardPeripherals;

mod blackbox;
pub mod config;

pub const MAX_DIRS: usize = 1;
pub const MAX_FILES: usize = 2;
pub const MAX_VOLUMES: usize = 1;

pub type SpiBus = spi::Spi<'static, SPI0, spi::Blocking>;
pub type SpiDevice = ExclusiveDevice<SpiBus, gpio::Output<'static>, NoDelay>;
pub type SdCard = embedded_sdmmc::SdCard<SpiDevice, Delay>;
/// We will have only one volume and directory opened with a max of two files opened:
/// config.toml and the current blackbox log file.
pub type VolumeManager =
    embedded_sdmmc::VolumeManager<SdCard, DummyTimesource, MAX_DIRS, MAX_FILES, MAX_VOLUMES>;

pub type Result<T> = core::result::Result<T, embedded_sdmmc::Error<embedded_sdmmc::SdCardError>>;

const BLACKBOX_NAME_HEADER: &str = "LOG";

pub struct Storage {
    volume_manager: VolumeManager,
    config_manager: config::Manager,
}

impl Storage {
    pub fn new(p: SdCardPeripherals) -> Result<Self> {
        let mut config = spi::Config::default();
        config.frequency = 100_000;
        let spi = spi::Spi::new_blocking(p.spi, p.clk_pin, p.mosi_pin, p.miso_pin, config);
        let spi_cs = gpio::Output::new(p.cs_pin, gpio::Level::High);
        let spi_dev = ExclusiveDevice::new_no_delay(spi, spi_cs).unwrap(); // NOTE: can't fail

        let sdcard = SdCard::new(spi_dev, Delay);

        // this also initializes the sdcard
        defmt::info!("sd card size: {}", sdcard.num_bytes()?);

        // up the frequency after initialization
        let mut config = spi::Config::default();
        config.frequency = 16_000_000;
        sdcard.spi(|dev| dev.bus_mut().set_config(&config));

        let volume_manager = VolumeManager::new_with_limits(sdcard, DummyTimesource, 0);
        let volume0 = volume_manager.open_volume(embedded_sdmmc::VolumeIdx(0))?;
        let root = volume0.open_root_dir()?;

        let mut max_blackbox_log_id: u32 = 0;
        root.iterate_dir(|entry| {
            if let Ok(name) = str::from_utf8(entry.name.base_name()) {
                if let Some(Ok(id)) = name
                    .strip_prefix(BLACKBOX_NAME_HEADER)
                    .map(|id_str| id_str.parse())
                {
                    max_blackbox_log_id = max_blackbox_log_id.max(id);
                }
            }
        })?;

        defmt::info!("max_blackbox_log_id: {}", max_blackbox_log_id);

        let root_raw = root.to_raw_directory();
        let config_manager = config::Manager::new(&volume_manager, root_raw)?;

        drop(volume0);

        Ok(Self {
            volume_manager,
            config_manager,
        })
    }

    pub fn tick(&mut self) -> Result<()> {
        self.config_manager.tick(&self.volume_manager)?;
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
