use core::ops::RangeInclusive;

use crsf::PayloadDump;
use embassy_sync::{signal::Signal, watch::Watch};
use embassy_time::{Duration, Ticker};
use embedded_io_async::{BufRead, Write};
use nalgebra::Vector3;

use crate::{
    RawMutex,
    control::{self, Command},
    sensors,
    utils::{SyncWatch, US_TO_NEG_ONE_ONE, read_process_and_consume},
};

pub static CHANNELS: SyncWatch<RawMutex, [u16; 16]> = SyncWatch::new([1500; 16]);
pub static COMMAND: Watch<RawMutex, Command, 1> = Watch::new();
pub static LINK_STATS: SyncWatch<RawMutex, LinkStats> = SyncWatch::new(LinkStats {
    uplink_link_quality: 0,
    downlink_link_quality: 0,
    uplink_rssi: 255,
    downlink_rssi: 255,
});
static CHANNELS_RECEIVED: Signal<RawMutex, ()> = Signal::new();

#[derive(Clone, Copy, Debug)]
pub struct LinkStats {
    pub uplink_link_quality: u8,
    pub uplink_rssi: u8,
    pub downlink_link_quality: u8,
    pub downlink_rssi: u8,
}

pub async fn task_channels_manager() {
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

    let command_snd = COMMAND.sender();

    loop {
        CHANNELS_RECEIVED.wait().await;
        let channels = CHANNELS.get();

        arm_switch.update(&channels);

        let flight_mode = flight_mode_manager.update(&channels);

        if let Some(auto_tune) = auto_tune_switch.update(&channels) {
            control::AUTO_TUNE.signal(auto_tune);
        }

        let command = if arm_switch.state {
            match flight_mode {
                FlightMode::Manual => Command::Manual {
                    throttle: channels[2],
                    throw: Vector3::new(
                        channels[0] as f32 - 1500.0,
                        channels[1] as f32 - 1500.0,
                        channels[3] as f32 - 1500.0,
                    ),
                },
                FlightMode::Rate => Command::Rate {
                    throttle: channels[2],
                    rate: Vector3::new(
                        US_TO_NEG_ONE_ONE.map(channels[0] as f32) * 45.0,
                        US_TO_NEG_ONE_ONE.map(channels[1] as f32) * 45.0,
                        US_TO_NEG_ONE_ONE.map(channels[3] as f32) * 45.0,
                    ),
                },
                FlightMode::Angle => Command::Angle {
                    throttle: channels[2],
                    roll_angle: US_TO_NEG_ONE_ONE.map(channels[0] as f32) * 45.0,
                    pitch_angle: US_TO_NEG_ONE_ONE.map(channels[1] as f32) * 45.0,
                },
            }
        } else {
            Command::Disarmed
        };

        command_snd.send(command);
    }
}

pub async fn task_read_crsf<R: BufRead>(mut reader: R) {
    let mut parser = crsf::Parser::new(crsf::ParserConfig::default());

    loop {
        let result = read_process_and_consume(&mut reader, |data| {
            let mut iter = parser.iter_packets(data);
            while let Some(result) = iter.next() {
                let packet = match result {
                    Ok(packet) => packet,
                    Err(_) => {
                        warn!("failed to parse crsf packet");
                        continue;
                    }
                };

                match packet {
                    crsf::Packet::RcChannelsPacked(raw_channels) => {
                        let channels: [u16; 16] =
                            core::array::from_fn(|i| crsf_read_raw_rc(raw_channels.0[i]));
                        CHANNELS.set(channels);
                        CHANNELS_RECEIVED.signal(());
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

        if result.is_err() {
            parser.reset();
            warn!("failed to read crsf data");
        }
    }
}

pub async fn task_telemetry<W: Write>(mut writer: W) {
    const PERIOD: Duration = Duration::from_hz(10);

    let mut ticker = Ticker::every(PERIOD);
    loop {
        ticker.next().await;

        let mut buf: [u8; crsf::MAX_PACKET_LEN] = [0; crsf::MAX_PACKET_LEN];

        {
            let battery = sensors::battery::DATA.get();

            let packet = crsf::BatterySensor {
                voltage: (battery.voltage * 10.0) as i16,
                current: (battery.current * 10.0) as i16,
                capacity_used: battery.capacity_used as u32, // already in mAh
                remaining: 0,                                // we don't have this info
            };

            let len = packet.dump(&mut buf).unwrap(); // this can't fail as we have a big enough buffer
            if writer.write_all(&buf[..len]).await.is_err() {
                warn!("failed to write to battery telemetry");
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
