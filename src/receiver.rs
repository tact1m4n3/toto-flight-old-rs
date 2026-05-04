use embassy_rp::uart;
use embedded_io_async::Read;
use static_cell::StaticCell;

use crate::{Irqs, ReceiverPeripherals, utils::SyncWatch};

pub static CHANNELS: SyncWatch<[u16; 16]> = SyncWatch::new([1500; 16]);
pub static LINK_STATS: SyncWatch<LinkStats> = SyncWatch::new(LinkStats {
    uplink_link_quality: 0,
    uplink_rssi: 0,
    downlink_link_quality: 0,
    downlink_rssi: 0,
});

#[derive(Clone, Copy)]
pub struct LinkStats {
    pub uplink_link_quality: u8,
    pub uplink_rssi: u8,
    pub downlink_link_quality: u8,
    pub downlink_rssi: u8,
}

pub fn init(spawner: crate::SystemSpawner, p: ReceiverPeripherals) {
    static RX_BUFFER: StaticCell<[u8; 2 * crsf::MAX_PACKET_LEN]> = StaticCell::new();
    static TX_BUFFER: StaticCell<[u8; 2 * crsf::MAX_PACKET_LEN]> = StaticCell::new();

    let mut config = uart::Config::default();
    config.baudrate = 115_200;

    let uart = uart::BufferedUart::new(
        p.uart,
        p.tx_pin,
        p.rx_pin,
        Irqs,
        RX_BUFFER.init([0; 2 * crsf::MAX_PACKET_LEN]),
        TX_BUFFER.init([0; 2 * crsf::MAX_PACKET_LEN]),
        config,
    );
    let (uart_tx, uart_rx) = uart.split();

    spawner.high_prio.spawn(packet_receiver(uart_rx).unwrap());
    spawner.low_prio.spawn(telemetry_sender(uart_tx).unwrap());
}

#[embassy_executor::task]
async fn packet_receiver(mut uart_rx: uart::BufferedUartRx) {
    let mut parser = crsf::Parser::new(crsf::ParserConfig::default());

    let mut buf = [0; 2 * crsf::MAX_PACKET_LEN];
    loop {
        let Ok(n) = uart_rx.read(&mut buf).await else {
            defmt::warn!("receiver_task: failed to read from uart");
            continue;
        };

        for parse_result in parser.iter_packets(&buf[..n]) {
            let packet = match parse_result {
                Ok(packet) => packet,
                Err(_) => {
                    defmt::warn!("receiver_task: failed to parse crsf packet");
                    continue;
                }
            };

            match packet {
                crsf::Packet::RcChannelsPacked(raw_channels) => {
                    let us_channels: [u16; 16] =
                        core::array::from_fn(|i| crsf_read_raw_rc(raw_channels.0[i]));
                    CHANNELS.set(us_channels);
                }
                crsf::Packet::LinkStatistics(ls) => {
                    let link_stats = LinkStats {
                        uplink_link_quality: ls.uplink_link_quality,
                        uplink_rssi: if ls.active_antenna == 1 {
                            ls.uplink_rssi_1
                        } else {
                            ls.uplink_rssi_2
                        },
                        downlink_link_quality: ls.downlink_link_quality,
                        downlink_rssi: ls.downlink_rssi,
                    };
                    LINK_STATS.set(link_stats);
                }
                _ => {}
            }
        }
    }
}

#[embassy_executor::task]
async fn telemetry_sender(uart_tx: uart::BufferedUartTx) {}

fn crsf_read_raw_rc(raw: u16) -> u16 {
    debug_assert!(raw >= crsf::RcChannelsPacked::CHANNEL_VALUE_MIN);
    debug_assert!(raw <= crsf::RcChannelsPacked::CHANNEL_VALUE_MAX);
    return ((raw as u32) * 1024 / 1639 + 881) as u16;
}
