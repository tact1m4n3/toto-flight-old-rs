use core::fmt::Write;
use embassy_futures::join::join;
use embassy_rp::{peripherals::USB, usb};
use embassy_usb::{class::cdc_acm, driver::EndpointError};
use heapless::String;
use static_cell::StaticCell;

use crate::{Irqs, SystemSpawner, UsbPeripherals, nav_loop::ArmBlocker, sensors::imu};

mod config_commands;

pub fn init(spawner: SystemSpawner, p: UsbPeripherals) {
    let driver = usb::Driver::new(p.usb, Irqs);
    spawner.default.spawn(usb_task(driver).unwrap());
}

type Driver = usb::Driver<'static, USB>;
type CdcAcmClass = cdc_acm::CdcAcmClass<'static, Driver>;

#[embassy_executor::task]
async fn usb_task(driver: Driver) -> ! {
    // Create embassy-usb Config
    let config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("Toto");
        config.product = Some("Toto flight controller");
        config.serial_number = Some("00000");
        config.max_power = 100;
        config.max_packet_size_0 = 64;
        config
    };

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut builder = {
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

        embassy_usb::Builder::new(
            driver,
            config,
            CONFIG_DESCRIPTOR.init([0; 256]),
            BOS_DESCRIPTOR.init([0; 256]),
            &mut [], // no msos descriptors
            CONTROL_BUF.init([0; 64]),
        )
    };

    // Create classes on the builder.
    let cdc = {
        static STATE: StaticCell<cdc_acm::State> = StaticCell::new();
        let state = STATE.init(cdc_acm::State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    // Build the builder.
    let mut usb = builder.build();

    join(usb.run(), serial_handler(cdc)).await;

    unreachable!();
}

async fn serial_handler(mut cdc: CdcAcmClass) {
    loop {
        cdc.wait_connection().await;
        defmt::info!("usb connected");
        let _arm_guard = ArmBlocker::new();
        let _ = shell(&mut cdc).await;
        defmt::info!("usb disconnected");
    }
}

// TODO: usb interface error reporting (e.g. for imu calibration error)

// TODO: currently limited to 64 byte response. We could leverage embedded_io_async::Write but this
// supports no formatting out of the box.
async fn shell(cdc: &mut CdcAcmClass) -> Result<(), EndpointError> {
    let mut buf = [0; 64];
    let mut parser = ShellParser::<32>::new();

    loop {
        let n = cdc.read_packet(&mut buf).await?;
        let mut remaining = &buf[..n];

        let mut response: String<64> = String::new();

        while !remaining.is_empty() {
            let command;
            (command, remaining) = parser.parse(remaining);
            match command {
                Ok(Some(command)) => match command {
                    ShellCommand::CalibrateImu => {
                        imu::start_calibration();
                        defmt::info!("imu calibration started");
                    }
                    ShellCommand::Get { key } => {
                        _ = config_commands::get_config(&mut response, key).await;
                    }
                    ShellCommand::Set { key, value } => {
                        _ = config_commands::set_config(&mut response, key, value).await;
                    }
                },
                Ok(None) => continue,
                Err(err) => _ = write!(response, "error parsing command: {}", err),
            }
            _ = write!(response, "\n\r");

            cdc.write_packet(response.as_bytes()).await?;
        }
    }
}

struct ShellParser<const N: usize> {
    line_buf: [u8; N],
    line_buf_end: usize,
}

impl<const N: usize> ShellParser<N> {
    const fn new() -> Self {
        Self {
            line_buf: [0; N],
            line_buf_end: 0,
        }
    }

    fn parse<'a, 'b>(
        &'a mut self,
        data: &'b [u8],
    ) -> (Result<Option<ShellCommand<'a>>, ShellParserError>, &'b [u8]) {
        let mut line_iter = data.splitn(2, |&b| b == b'\n');
        let line_end = line_iter.next().unwrap(); // there is at least one item

        if self.line_buf_end + line_end.len() > self.line_buf.len() {
            // line too long
            self.line_buf_end = 0;
            return (
                Err(ShellParserError::LineTooLong),
                line_iter.next().unwrap_or(&[]),
            );
        }

        self.line_buf[self.line_buf_end..][..line_end.len()].copy_from_slice(line_end);
        self.line_buf_end += line_end.len();

        let remaining = match line_iter.next() {
            Some(remaining) => remaining,
            None => return (Ok(None), &[]),
        };

        // we have a line! let's parse it
        let line = &self.line_buf[..self.line_buf_end];
        self.line_buf_end = 0;

        let Ok(line) = core::str::from_utf8(line) else {
            return (Err(ShellParserError::InvalidUtf8Line), remaining);
        };

        let mut word_iter = line.split_whitespace();
        let command = match word_iter.next() {
            Some("calibrate") => Some(ShellCommand::CalibrateImu),
            Some("get") => {
                let Some(key) = word_iter.next() else {
                    return (Err(ShellParserError::InvalidCommand), remaining);
                };
                Some(ShellCommand::Get { key })
            }
            Some("set") => {
                let Some(key) = word_iter.next() else {
                    return (Err(ShellParserError::InvalidCommand), remaining);
                };
                let Some(value) = word_iter.remainder() else {
                    return (Err(ShellParserError::InvalidCommand), remaining);
                };
                Some(ShellCommand::Set { key, value })
            }
            Some(_) | None => return (Err(ShellParserError::InvalidCommand), remaining),
        };

        (Ok(command), remaining)
    }
}

enum ShellCommand<'a> {
    CalibrateImu,
    Get { key: &'a str },
    Set { key: &'a str, value: &'a str },
}

#[derive(Debug)]
enum ShellParserError {
    LineTooLong,
    InvalidUtf8Line,
    InvalidCommand,
}

impl core::fmt::Display for ShellParserError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ShellParserError::LineTooLong => write!(f, "line too long"),
            ShellParserError::InvalidUtf8Line => write!(f, "invalid utf8 line"),
            ShellParserError::InvalidCommand => write!(f, "invalid command"),
        }
    }
}
