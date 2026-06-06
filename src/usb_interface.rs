use core::fmt::Write;
use embassy_usb::{
    Builder,
    class::cdc_acm::{self, CdcAcmClass},
    driver::{Driver, EndpointError},
};
use futures::join;
use heapless::String;
use static_cell::StaticCell;

use crate::{
    imu,
    usb_interface::shell_parser::{CalibrationType, ShellCommand, ShellParser},
};

mod shell_parser;

const MAX_PACKET_SIZE: u8 = 64;

#[allow(unreachable_code)]
pub async fn task<D: Driver<'static>>(driver: D) {
    let config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("Toto");
        config.product = Some("Toto Flight Controller");
        config.serial_number = Some("00000");
        config.max_power = 100;
        config.max_packet_size_0 = MAX_PACKET_SIZE;
        config
    };

    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        &mut [],
        CONTROL_BUF.init([0; 64]),
    );

    let shell_cdc = {
        static STATE: StaticCell<cdc_acm::State> = StaticCell::new();
        let state = STATE.init(cdc_acm::State::new());
        CdcAcmClass::new(&mut builder, state, u16::from(MAX_PACKET_SIZE))
    };

    let mut usb = builder.build();

    join!(usb.run(), shell_cdc_run(shell_cdc));
}

async fn shell_cdc_run<D: Driver<'static>>(mut cdc: CdcAcmClass<'static, D>) {
    loop {
        cdc.wait_connection().await;
        info!("usb connected");
        // if let Ok(_guard) = ArmBlocker::new() {
        _ = shell(&mut cdc).await;
        // }
        info!("usb disconnected");
    }
}

async fn shell<D: Driver<'static>>(cdc: &mut CdcAcmClass<'static, D>) -> Result<(), EndpointError> {
    let mut buf = [0; 64];
    let mut parser = ShellParser::<32>::new();

    loop {
        let n = cdc.read_packet(&mut buf).await?;
        let mut remaining = &buf[..n];

        let mut response: String<64> = String::new();

        while let Some((result, new_remaining)) = parser.parse(remaining) {
            remaining = new_remaining;

            match result {
                Ok(command) => match command {
                    ShellCommand::Calibrate(CalibrationType::ImuEasyCalibration) => {
                        imu::start_calibration();
                        info!("imu calibration started");
                    }
                },
                Err(err) => _ = write!(response, "error parsing command: {err}"),
            }
            _ = write!(response, "\n\r");

            cdc.write_packet(response.as_bytes()).await?;
        }
    }
}
