use embassy_rp::{gpio, uart};
use embassy_time::{Duration, Instant};
use embedded_io_async::Read;
use static_cell::StaticCell;
use ublox::{FixedBuffer, Parser, UbxPacket, packets::packetref_proto33::PacketRef};

use crate::{GpsPeripherals, Irqs, SystemSpawner, mutex_cells, utils::SyncWatch};

pub static DATA: SyncWatch<Data> = SyncWatch::new(Data {
    latitude: 0.0,
    longitude: 0.0,
    altitude: 0.0,
    h_acc: f32::INFINITY,
    v_acc: f32::INFINITY,
    speed: 0.0,
    heading: 0.0,
    num_sat: 0,
    hdop: f32::INFINITY,
    fix_type: FixType::Undefined,
    vertical_velocity: 0.0,
});

mutex_cells! {
    PPS_TIMESTAMP: Option<Instant> = None;
}

pub fn init(spawner: SystemSpawner, p: GpsPeripherals) {
    static RX_BUFFER: StaticCell<[u8; 128]> = StaticCell::new();

    let mut config = uart::Config::default();
    config.baudrate = 115_200;
    let uart = uart::BufferedUartRx::new(p.uart, Irqs, p.rx_pin, RX_BUFFER.init([0; 128]), config);

    let pps_pin = gpio::Input::new(p.pps_pin, gpio::Pull::None);

    spawner.high_prio.spawn(main_task(uart).unwrap());
    spawner.high_prio.spawn(pps_listener_task(pps_pin).unwrap());
}

#[embassy_executor::task]
async fn main_task(mut uart_rx: uart::BufferedUartRx) {
    let mut measurement_delay = Duration::from_millis(80); // default measurement delay

    let mut buffer = [0u8; 128];
    let mut parser: ublox::Parser<FixedBuffer<128>> = Parser::new_fixed();
    loop {
        match uart_rx.read(&mut buffer).await {
            Ok(n) => {
                let mut packet_iter = parser.consume_ubx(&buffer[..n]);
                while let Some(result) = packet_iter.next() {
                    match result {
                        Ok(packet) => {
                            let UbxPacket::Proto33(ref packet) = packet;

                            match packet {
                                PacketRef::NavPvt(pvt) => {
                                    let now = Instant::now();
                                    if let Some(pps_instant) =
                                        PPS_TIMESTAMP.lock(|pps_timestamp| pps_timestamp.take())
                                    {
                                        measurement_delay = now - pps_instant;
                                    }

                                    let timestamp = now.saturating_sub(measurement_delay);
                                    let data = Data {
                                        latitude: pvt.latitude(),
                                        longitude: pvt.longitude(),
                                        altitude: pvt.height_msl() as f32,
                                        h_acc: pvt.horizontal_accuracy() as f32,
                                        v_acc: pvt.vertical_accuracy() as f32,
                                        speed: pvt.ground_speed_2d() as f32,
                                        heading: pvt.heading_motion() as f32,
                                        num_sat: pvt.num_satellites(),
                                        hdop: pvt.pdop() as f32,
                                        fix_type: pvt.fix_type().into(),
                                        vertical_velocity: pvt.vel_down() as f32,
                                    };

                                    DATA.set_with_timestamp(data, timestamp);
                                }
                                _ => {}
                            }
                        }
                        Err(_) => {
                            defmt::warn!("gps: failed to parse packet");
                        }
                    }
                }
            }
            Err(_) => {
                defmt::warn!("gps: failed to read uart");
            }
        };
    }
}

#[embassy_executor::task]
async fn pps_listener_task(mut pps_pin: gpio::Input<'static>) {
    loop {
        pps_pin.wait_for_rising_edge().await;
        let instant = Instant::now();
        critical_section::with(|cs| PPS_TIMESTAMP.borrow(cs).set(Some(instant)));
    }
}

#[derive(Clone, Copy, defmt::Format)]
pub struct Data {
    /// Latitude in degrees (positive = North)
    pub latitude: f64,
    /// Longitude in degrees (positive = East)
    pub longitude: f64,
    /// Altitude above ellipsoid in meters
    pub altitude: f32,
    /// Horizontal accuracy estimate in meters
    pub h_acc: f32,
    /// Vertical accuracy estimate in meters
    pub v_acc: f32,
    /// Speed over ground in m/s
    pub speed: f32,
    /// Course over ground in degrees (0-360)
    pub heading: f32,
    /// Number of satellites in use
    pub num_sat: u8,
    /// Horizontal dilution of precision
    pub hdop: f32,
    /// Fix type
    pub fix_type: FixType,
    /// Vertical velocity (speed downward) in m/s
    pub vertical_velocity: f32,
}

#[derive(Clone, Copy, defmt::Format)]
pub enum FixType {
    NoFix,
    DeadReckoningOnly,
    Fix2D,
    Fix3D,
    GPSPlusDeadReckoning,
    TimeOnlyFix,
    Undefined,
}

impl From<ublox::GnssFixType> for FixType {
    fn from(value: ublox::GnssFixType) -> Self {
        match value {
            ublox::GnssFixType::NoFix => FixType::NoFix,
            ublox::GnssFixType::DeadReckoningOnly => FixType::DeadReckoningOnly,
            ublox::GnssFixType::Fix2D => FixType::Fix2D,
            ublox::GnssFixType::Fix3D => FixType::Fix3D,
            ublox::GnssFixType::GPSPlusDeadReckoning => FixType::GPSPlusDeadReckoning,
            ublox::GnssFixType::TimeOnlyFix => FixType::TimeOnlyFix,
            _ => FixType::Undefined,
        }
    }
}

// async fn configure_gps(uart: &mut Uart) -> Result<(), Error> {
//     let mut parser: ublox::Parser<FixedBuffer<256>> = Parser::new_fixed();
//
//     const TARGET_BAUD: u32 = 115_200;
//     const BAUD_SWEEP: &[u32] = &[
//         9_600, 19_200, 38_400, 57_600, 115_200, 230_400, 460_800, 921_600,
//     ];
//
//     let mut ok = false;
//     for &probe in BAUD_SWEEP {
//         uart.set_baudrate(probe);
//
//         if let Ok(_) = detect(uart, &mut parser).await {
//             ok = true;
//             break;
//         }
//     }
//     if !ok {
//         return Err(Error::NotResponding);
//     }
//
//     send_cfg(uart, &mut parser, &[Uart1Baudrate(TARGET_BAUD)]).await?;
//     uart.set_baudrate(TARGET_BAUD);
//
//     send_cfg(
//         uart,
//         &mut parser,
//         &[
//             Uart1InProtUbx(true),
//             Uart1InProtNmea(false),
//             Uart1OutProtUbx(true),
//             Uart1OutProtNmea(false),
//         ],
//     )
//     .await?;
//
//     // Disable common NMEA, enable NAV-PVT + NAV-SIG, set rates and dynamics/constellations.
//     send_cfg(
//         uart,
//         &mut parser,
//         &[
//             MsgOutNmeaIdGgaUart1(0),
//             MsgOutNmeaIdGllUart1(0),
//             MsgOutNmeaIdGsaUart1(0),
//             MsgOutNmeaIdRmcUart1(0),
//             MsgOutNmeaIdVtgUart1(0),
//             MsgOutUbxNavPvtUart1(1),
//             MsgOutUbxNavSigUart1(0),
//             MsgOutUbxNavStatusUart1(0),
//         ],
//     )
//     .await?;
//
//     send_cfg(
//         uart,
//         &mut parser,
//         &[
//             SignalGpsEna(true),
//             SignalGpsL1caEna(true),
//             SignalGalEna(true),
//             SignalGalE1Ena(true),
//             SignalGloEna(true),
//             SignalGloL1Ena(true),
//             SignalBdsEna(false),
//             SignalBdsB1Ena(false),
//             SignalQzssEna(false),
//             SignalQzssL1caEna(false),
//         ],
//     )
//     .await?;
//
//     send_cfg(
//         uart,
//         &mut parser,
//         &[
//             NavSpgDynModel(NavDynamicModel::AirborneWithLess2gAcceleration),
//             RateMeas(1000), // 1Hz
//             RateNav(1),
//             RateTimeref(AlignmentToReferenceTime::Utc),
//         ],
//     )
//     .await?;
//
//     send_cfg(
//         uart,
//         &mut parser,
//         &[
//             TpTp1Ena(true),
//             TpPeriodTp1(1_000_000), // every 1s
//             TpLenTp1(100_000),      // 100ms
//             TpPolTp1(true),
//             TpTimegridTp1(AlignmentToReferenceTime::Utc),
//         ],
//     )
//     .await?;
//
//     Ok(())
// }
//
// async fn detect(uart: &mut Uart, parser: &mut Parser<FixedBuffer<256>>) -> Result<(), Error> {
//     // UBX-CFG-PRT poll (class=0x06, id=0x00, len=0, no payload)
//     // Checksum bytes: CK_A=0x06, CK_B=0x18
//     // TODO: maybe make this an actual uart
//     uart.write_all(&[0xB5, 0x62, 0x06, 0x00, 0x00, 0x00, 0x06, 0x18])
//         .await?;
//
//     let mut buffer = [0u8; 128];
//     embassy_time::with_timeout(Duration::from_millis(100), async {
//         loop {
//             let Ok(n) = uart.read(&mut buffer).await else {
//                 continue;
//             };
//
//             let mut it = parser.consume_ubx(&buffer[..n]);
//             while let Some(result) = it.next() {
//                 if let Ok(packet) = result {
//                     let UbxPacket::Proto33(packet) = packet;
//                     if let PacketRef::CfgPrtUart(_) = packet {
//                         return;
//                     }
//                 }
//             }
//         }
//     })
//     .await?;
//
//     Ok(())
// }
//
// async fn send_cfg(
//     uart: &mut Uart,
//     parser: &mut Parser<FixedBuffer<256>>,
//     cfgs: &[CfgVal],
// ) -> Result<(), Error> {
//     let mut packet: Vec<u8, 256> = Vec::new();
//
//     CfgValSetBuilder {
//         version: 1,
//         layers: CfgLayerSet::FLASH,
//         reserved1: 0,
//         cfg_data: cfgs,
//     }
//     .extend_to(&mut packet);
//
//     uart.write_all(&packet).await?;
//
//     let mut buffer = [0u8; 128];
//     let result = embassy_time::with_timeout(Duration::from_secs(1), async {
//         loop {
//             let Ok(n) = uart.read(&mut buffer).await else {
//                 continue;
//             };
//
//             let mut it = parser.consume_ubx(&buffer[..n]);
//             while let Some(result) = it.next() {
//                 match result {
//                     Ok(packet) => {
//                         let UbxPacket::Proto33(ref pkt) = packet;
//                         match pkt {
//                             PacketRef::AckAck(ack) => {
//                                 if AckAckRef::is_ack_for::<CfgValSet>(&ack) {
//                                     return Ok(());
//                                 }
//                             }
//                             PacketRef::AckNak(_) => {
//                                 return Err(Error::Nack);
//                             }
//                             _ => (),
//                         }
//                     }
//                     _ => {}
//                 }
//             }
//         }
//     })
//     .await?;
//
//     result?;
//
//     Ok(())
// }
//
// #[derive(Debug)]
// pub enum Error {
//     NotResponding,
//     Nack,
//     AckTimeout,
//     UartError,
// }
//
// impl From<embassy_time::TimeoutError> for Error {
//     fn from(_: embassy_time::TimeoutError) -> Self {
//         Self::AckTimeout
//     }
// }
//
// impl From<embassy_rp::uart::Error> for Error {
//     fn from(_: embassy_rp::uart::Error) -> Self {
//         Self::UartError
//     }
// }
