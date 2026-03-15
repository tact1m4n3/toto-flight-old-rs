use core::{
    cell::RefCell,
    ffi::CStr,
    sync::atomic::{AtomicBool, Ordering},
};

use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use embedded_sdmmc::{RawDirectory, RawFile};
use zerocopy::{FromBytes, Immutable, IntoBytes};

use super::VolumeManager;

const MAGIC: u32 = 0xC0F1C0F1;
const FILE_NAME: &str = "CONFIG.BIN";
const CRAFT_NAME_MAX_LEN: usize = 15;

static FLUSH: AtomicBool = AtomicBool::new(false);
static CONFIG: Mutex<CriticalSectionRawMutex, RefCell<Config>> =
    Mutex::new(RefCell::new(Config::DEFAULT));

pub fn access<R, F: FnOnce(&Config) -> R>(cs: critical_section::CriticalSection, f: F) -> R {
    let cfg = CONFIG.borrow(cs).borrow();
    f(&cfg)
}

pub fn access_mut<R, F: FnOnce(&mut Config) -> R>(
    cs: critical_section::CriticalSection,
    f: F,
) -> R {
    let mut cfg = CONFIG.borrow(cs).borrow_mut();
    let ret = f(&mut cfg);
    FLUSH.store(true, Ordering::Release);
    ret
}

pub struct Manager {
    raw_file: Option<RawFile>,
}

impl Manager {
    pub fn new(
        volume_manager: &VolumeManager,
        dir: RawDirectory,
    ) -> crate::sd_card::Result<Manager> {
        match volume_manager.open_file_in_dir(dir, FILE_NAME, embedded_sdmmc::Mode::ReadWriteAppend)
        {
            Ok(raw_file) => {
                let file = raw_file.to_file(volume_manager);
                if file.length() as usize >= core::mem::size_of::<Config>() {
                    file.seek_from_start(0)?;

                    let mut config = Config::DEFAULT;
                    file.read(config.as_mut_bytes())?;

                    if config.magic == MAGIC {
                        critical_section::with(|cs| *CONFIG.borrow(cs).borrow_mut() = config);
                        return Ok(Manager {
                            raw_file: Some(file.to_raw_file()),
                        });
                    }
                }
            }
            Err(embedded_sdmmc::Error::NotFound) => (),
            Err(err) => return Err(err),
        }

        let raw_file = volume_manager.open_file_in_dir(
            dir,
            FILE_NAME,
            // delete all content from an existing file or create an empty file
            embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
        )?;
        let file = raw_file.to_file(volume_manager);

        let config = critical_section::with(|cs| CONFIG.borrow(cs).borrow().clone());
        file.write(config.as_bytes())?;
        file.flush()?;

        Ok(Manager {
            raw_file: Some(file.to_raw_file()),
        })
    }

    pub fn tick(&mut self, volume_manager: &VolumeManager) -> crate::sd_card::Result<()> {
        if !FLUSH.swap(false, Ordering::Relaxed) {
            return Ok(());
        }

        let raw_file = self.raw_file.take().unwrap(); // can't fail
        let file = raw_file.to_file(volume_manager);
        file.seek_from_start(0)?;
        let config = critical_section::with(|cs| CONFIG.borrow(cs).borrow().clone());
        file.write(config.as_bytes())?;
        file.flush()?;
        self.raw_file = Some(file.to_raw_file());
        Ok(())
    }
}

#[derive(Clone, FromBytes, IntoBytes, Immutable)]
#[repr(C)]
pub struct Config {
    magic: u32,
    pub craft_name: [u8; CRAFT_NAME_MAX_LEN + 1],
    pub calibration: Calibration,
}

impl Config {
    pub const DEFAULT: Config = Config {
        magic: MAGIC,
        craft_name: [0; 16],
        calibration: Calibration::DEFAULT,
    };

    pub fn craft_name(&self) -> Result<&[u8], core::ffi::FromBytesUntilNulError> {
        Ok(CStr::from_bytes_until_nul(&self.craft_name)?.to_bytes())
    }

    pub fn set_craft_name(&mut self, name: &[u8]) -> Result<(), StringTooLong> {
        if name.len() > CRAFT_NAME_MAX_LEN {
            return Err(StringTooLong);
        }

        self.craft_name[0..name.len()].copy_from_slice(name);
        self.craft_name[name.len()..].fill(0);

        Ok(())
    }
}

#[derive(Clone, FromBytes, IntoBytes, Immutable)]
#[repr(C)]
pub struct Calibration {
    pub gyro_bias: [f32; 3],
    pub accel_bias: [f32; 3],
}

impl Calibration {
    pub const DEFAULT: Calibration = Calibration {
        gyro_bias: [0.0; 3],
        accel_bias: [0.0; 3],
    };
}

pub struct StringTooLong;
