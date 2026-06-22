use core::ops::RangeInclusive;

use crsf::PayloadDump;
use embassy_sync::{signal::Signal, watch::Watch};
use embassy_time::{Duration, Ticker};
use embedded_io_async::{BufRead, Write};
use nalgebra::Vector3;
use uom::si::{
    electric_charge::milliampere_hour, electric_current::deciampere, electric_potential::decivolt,
    velocity::meter_per_second,
};

use crate::{
    RawMutex,
    control::{self, Command},
    sensors,
    utils::{SyncWatch, US_TO_NEG_ONE_ONE, read_process_and_consume},
};

pub static CHANNELS: SyncWatch<RawMutex, Channels> = SyncWatch::new(Channels::DEFAULT);
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
        aux_channel_index: 0,
        range: 1700..=2000,
        state: false,
    };
    let mut flight_mode_manager = FlightModeManager {
        rate_mode_condition: ChannelCondition {
            aux_channel_index: 1,
            range: 1400..=1600,
            state: false,
        },
        angle_mode_condition: ChannelCondition {
            aux_channel_index: 1,
            range: 1800..=2000,
            state: false,
        },
    };
    let mut auto_tune_switch = ChannelCondition {
        aux_channel_index: 2,
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
                    throttle: channels.throttle(),
                    throw: Vector3::new(
                        channels.roll() as f32 - 1500.0,
                        channels.pitch() as f32 - 1500.0,
                        channels.yaw() as f32 - 1500.0,
                    ),
                },
                FlightMode::Rate => Command::Rate {
                    throttle: channels.throttle(),
                    rate: Vector3::new(
                        US_TO_NEG_ONE_ONE.map(channels.roll() as f32) * 45.0,
                        US_TO_NEG_ONE_ONE.map(channels.pitch() as f32) * 45.0,
                        US_TO_NEG_ONE_ONE.map(channels.yaw() as f32) * 45.0,
                    ),
                },
                FlightMode::Angle => Command::Angle {
                    throttle: channels.throttle(),
                    roll_angle: US_TO_NEG_ONE_ONE.map(channels.roll() as f32) * 45.0,
                    pitch_angle: US_TO_NEG_ONE_ONE.map(channels.pitch() as f32) * 45.0,
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
                        let raw_channels: [u16; 16] =
                            core::array::from_fn(|i| crsf_read_raw_rc(raw_channels.0[i]));
                        let channels = Channels::from_aetr1234(&raw_channels);
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
                voltage: battery.voltage.get::<decivolt>() as i16,
                current: battery.current.get::<deciampere>() as i16,
                capacity_used: battery.capacity_used.get::<milliampere_hour>() as u32, // already in mAh
                remaining: 0, // we don't have this info
            };

            let len = packet.dump(&mut buf).unwrap(); // this can't fail as we have a big enough buffer
            if writer.write_all(&buf[..len]).await.is_err() {
                warn!("failed to write to battery telemetry");
            }
        }

        {
            let gps = sensors::gps::DATA.get();

            let packet = crsf::Gps {
                latitude: (gps.latitude.get::<uom::si::angle::degree>() * 1e7) as i32,
                longitude: (gps.longitude.get::<uom::si::angle::degree>() * 1e7) as i32,
                altitude: (gps.altitude.get::<uom::si::length::meter>() * 100.0) as u16,
                heading: (gps.heading.get::<uom::si::angle::degree>() * 100.0) as u16,
                groundspeed: (gps.velocity_over_ground.get::<meter_per_second>() * 100.0) as u16,
                satellites: gps.num_sat,
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
    pub fn update(&mut self, channels: &Channels) -> FlightMode {
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

/// Channels struct with guaranteed order: Throttle, Roll, Pitch, Yaw, AuxN.
#[derive(Clone, Copy)]
pub struct Channels([u16; 16]);

impl Channels {
    pub const DEFAULT: Channels = Channels([0; 16]);
    pub const AUX_CHANNEL_COUNT: usize = 12;

    pub fn from_aetr1234(raw: &[u16; 16]) -> Self {
        let mut inner = [0u16; 16];
        inner[0..4].copy_from_slice(&[raw[2], raw[0], raw[1], raw[3]]);
        inner[4..16].copy_from_slice(&raw[4..16]);
        Self(inner)
    }

    pub fn as_slice(&self) -> &[u16; 16] {
        &self.0
    }

    pub fn throttle(&self) -> u16 {
        self.0[0]
    }

    pub fn roll(&self) -> u16 {
        self.0[1]
    }

    pub fn pitch(&self) -> u16 {
        self.0[2]
    }

    pub fn yaw(&self) -> u16 {
        self.0[3]
    }

    pub fn aux(&self, index: usize) -> u16 {
        assert!(index < Self::AUX_CHANNEL_COUNT);
        self.0[4 + index]
    }

    pub fn to_msp_format(&self) -> [u16; 16] {
        let mut out = [0u16; 16];
        out[0..4].copy_from_slice(&[self.roll(), self.pitch(), self.yaw(), self.throttle()]);
        out[4..16].copy_from_slice(&self.0[4..16]);
        out
    }
}

struct ChannelCondition {
    aux_channel_index: usize,
    range: RangeInclusive<u16>,
    state: bool,
}

impl ChannelCondition {
    fn update(&mut self, channels: &Channels) -> Option<bool> {
        let value = channels.aux(self.aux_channel_index);
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
