use crsf::PayloadDump;
use embassy_time::{Duration, Ticker};
use embedded_io_async::{BufRead, Write};

use crate::{RawMutex, utils::SyncWatch};

pub static CHANNELS: SyncWatch<RawMutex, [u16; 16]> = SyncWatch::new([1_500; 16]);
pub static LINK_STATS: SyncWatch<RawMutex, LinkStats> = SyncWatch::new(LinkStats::DEFAULT);

pub async fn task_read<R: BufRead>(mut reader: R) {
    let mut parser: crsf::Parser = crsf::Parser::new(crsf::ParserConfig::default());

    loop {
        let n = {
            let data = match reader.fill_buf().await {
                Ok(data) => data,
                Err(_) => {
                    parser.reset();
                    defmt::error!("failed to read from uart");
                    continue;
                }
            };

            for result in parser.iter_packets(data) {
                let packet = match result {
                    Ok(packet) => packet,
                    Err(_) => {
                        defmt::warn!("failed to parse crsf packet");
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

            data.len()
        };

        reader.consume(n);
    }
}

#[derive(Clone, Copy)]
pub struct LinkStats {
    pub uplink_link_quality: u8,
    pub uplink_rssi: u8,
    pub downlink_link_quality: u8,
    pub downlink_rssi: u8,
}

impl LinkStats {
    pub const DEFAULT: LinkStats = LinkStats {
        uplink_link_quality: 0,
        uplink_rssi: 255,
        downlink_link_quality: 0,
        downlink_rssi: 255,
    };
}

const TELEMETRY_FREQ_HZ: u64 = 10;

pub async fn task_telemetry<W: Write>(mut writer: W) {
    let mut battery_info_watcher = super::battery::DATA.watcher();

    let mut ticker = Ticker::every(Duration::from_hz(TELEMETRY_FREQ_HZ));
    loop {
        ticker.next().await;

        let mut buf: [u8; crsf::MAX_PACKET_LEN] = [0; crsf::MAX_PACKET_LEN];

        if let Some(battery) = battery_info_watcher.get_if_changed() {
            let mut packet = crsf::BatterySensor::new();
            packet.set_voltage(battery.voltage as u16); // already in mV
            packet.set_current(battery.current as u16); // already in mA
            packet.set_capacity_used(battery.capacity_used as u32); // already in mAh
            packet.set_remaining(0); // we don't have this info

            let len = packet.dump(&mut buf).unwrap(); // this can't fail as we have a big enough buffer
            if writer.write_all(&buf[..len]).await.is_err() {
                defmt::warn!("failed to write to uart");
            }
        }
    }
}

fn crsf_read_raw_rc(raw: u16) -> u16 {
    ((raw as u32) * 1024 / 1639 + 881) as u16
}
