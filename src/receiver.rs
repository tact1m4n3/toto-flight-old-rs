use core::ops::RangeInclusive;

use crsf::PayloadDump;
use embassy_time::{Duration, Ticker};
use embedded_io_async::{BufRead, Write};
use nalgebra::Vector3;

use crate::{
    RawMutex, control,
    utils::{SyncWatch, US_TO_NEG_ONE_ONE, read_process_and_consume},
};

pub static CHANNELS: SyncWatch<RawMutex, [u16; 16]> = SyncWatch::new([1_500; 16]);
pub static LINK_STATS: SyncWatch<RawMutex, LinkStats> = SyncWatch::new(LinkStats::DEFAULT);

pub async fn task_read<R: BufRead>(mut reader: R) {
    let mut arm_switch = ChannelCondition {
        channel_idx: 4,
        range: 1700..=2000,
        state: false,
    };
    let mut flight_mode_manager = FlightModeManager {
        rate_mode_condition: ChannelCondition {
            channel_idx: 5,
            range: 1400..=1600,
            state: false,
        },
        angle_mode_condition: ChannelCondition {
            channel_idx: 5,
            range: 1800..=2000,
            state: false,
        },
    };
    let mut auto_tune_switch = ChannelCondition {
        channel_idx: 6,
        range: 1700..=2000,
        state: false,
    };

    let mut parser = crsf::Parser::new(crsf::ParserConfig::default());

    loop {
        let result = read_process_and_consume(&mut reader, |data| {
            let mut iter = parser.iter_packets(data);
            while let Some(result) = iter.next() {
                let packet = match result {
                    Ok(packet) => packet,
                    Err(err) => {
                        log::warn!("failed to parse crsf packet: {err}");
                        continue;
                    }
                };

                match packet {
                    crsf::Packet::RcChannelsPacked(raw_channels) => {
                        let channels: [u16; 16] =
                            core::array::from_fn(|i| crsf_read_raw_rc(raw_channels.0[i]));

                        arm_switch.update(&channels);

                        let flight_mode = flight_mode_manager.update(&channels);

                        if let Some(auto_tune) = auto_tune_switch.update(&channels) {
                            control::AUTO_TUNE.signal(auto_tune);
                        }

                        let command = if arm_switch.state {
                            match flight_mode {
                                FlightMode::Manual => control::Command::Manual {
                                    throttle: channels[2],
                                    throw: Vector3::new(
                                        channels[0] as f32 - 1500.0,
                                        channels[1] as f32 - 1500.0,
                                        channels[3] as f32 - 1500.0,
                                    ),
                                },
                                FlightMode::Rate => control::Command::Rate {
                                    throttle: channels[2],
                                    rate: Vector3::new(
                                        US_TO_NEG_ONE_ONE.map(channels[0] as f32) * 45.0,
                                        US_TO_NEG_ONE_ONE.map(channels[1] as f32) * 45.0,
                                        US_TO_NEG_ONE_ONE.map(channels[3] as f32) * 45.0,
                                    ),
                                },
                                FlightMode::Angle => control::Command::Angle {
                                    throttle: channels[2],
                                    roll_angle: US_TO_NEG_ONE_ONE.map(channels[0] as f32) * 45.0,
                                    pitch_angle: US_TO_NEG_ONE_ONE.map(channels[1] as f32) * 45.0,
                                },
                            }
                        } else {
                            control::Command::Disarmed
                        };

                        CHANNELS.set(channels);

                        control::RC_COMMAND.signal(command);
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
        })
        .await;

        if let Err(err) = result {
            parser.reset();
            log::warn!("failed to read from uart {err}");
        }
    }
}

#[derive(Clone, Copy, Debug)]
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
                log::warn!("failed to write to uart");
            }
        }
    }
}

fn crsf_read_raw_rc(raw: u16) -> u16 {
    ((raw as u32) * 1024 / 1639 + 881) as u16
}

enum FlightMode {
    Manual,
    Rate,
    Angle,
}

struct FlightModeManager {
    rate_mode_condition: ChannelCondition,
    angle_mode_condition: ChannelCondition,
}

impl FlightModeManager {
    pub fn update(&mut self, channels: &[u16; 16]) -> FlightMode {
        self.rate_mode_condition.update(channels);
        self.angle_mode_condition.update(channels);
        if self.angle_mode_condition.state {
            FlightMode::Angle
        } else if self.rate_mode_condition.state {
            FlightMode::Rate
        } else {
            FlightMode::Manual
        }
    }
}

struct ChannelCondition {
    channel_idx: u8,
    range: RangeInclusive<u16>,
    state: bool,
}

impl ChannelCondition {
    fn update(&mut self, channels: &[u16; 16]) -> Option<bool> {
        let value = channels[self.channel_idx as usize];
        match self.state {
            false if self.range.contains(&value) => {
                self.state = true;
                Some(self.state)
            }
            true if !self.range.contains(&value) => {
                self.state = false;
                Some(self.state)
            }
            _ => None,
        }
    }
}
