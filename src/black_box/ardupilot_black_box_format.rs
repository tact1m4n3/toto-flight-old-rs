use core::mem::size_of;
use embassy_time::Instant;
use nalgebra::Vector3;
use zerocopy::byteorder::{F32, LE, U16, U32, U64};
use zerocopy::{Immutable, IntoBytes, KnownLayout};

use crate::utils::pack;

pub struct Logger<W> {
    pub writer: W,
}

impl<W: embedded_io_async::Write> Logger<W> {
    async fn write<T: IntoBytes + Immutable>(&mut self, msg: &T) -> Result<(), W::Error> {
        self.writer.write_all(msg.as_bytes()).await
    }

    async fn write_fmt(
        &mut self,
        type_id: u8,
        length: u8,
        name: &str,
        format: &str,
        columns: &str,
    ) -> Result<(), W::Error> {
        self.write(&FmtMsg {
            sync: SYNC,
            type_: FMT_TYPE_ID,
            type_id,
            length,
            name: pack(name),
            format: pack(format),
            columns: pack(columns),
        })
        .await
    }

    pub async fn write_header(&mut self) -> Result<(), W::Error> {
        let t = Instant::now().as_micros();

        self.write_fmt(
            TYPE_FMT,
            89,
            "FMT",
            "BBnNZ",
            "Type,Length,Name,Format,Columns",
        )
        .await?;

        self.write_fmt(
            TYPE_UNIT,
            size_of::<UnitMsg>() as u8,
            "UNIT",
            "QbZ",
            "TimeUS,Id,Label",
        )
        .await?;

        self.write_fmt(
            TYPE_MULT,
            size_of::<MultMsg>() as u8,
            "MULT",
            "QbQ",
            "TimeUS,Id,Mult",
        )
        .await?;

        self.write_fmt(
            TYPE_FMTU,
            size_of::<FmtuMsg>() as u8,
            "FMTU",
            "QBNN",
            "TimeUS,FmtType,UnitIds,MultIds",
        )
        .await?;

        self.write_fmt(
            TYPE_MSG,
            size_of::<MsgMsg>() as u8,
            "MSG",
            "QZ",
            "TimeUS,Message",
        )
        .await?;

        self.write_fmt(
            TYPE_PARM,
            size_of::<ParmMsg>() as u8,
            "PARM",
            "QNd",
            "TimeUS,Name,Value",
        )
        .await?;

        self.write_fmt(
            TYPE_ACCEL,
            size_of::<AccelMsg>() as u8,
            "ACC",
            "Qfff",
            "TimeUS,AccX,AccY,AccZ",
        )
        .await?;

        self.write_fmt(
            TYPE_GYRO,
            size_of::<GyroMsg>() as u8,
            "GYR",
            "Qfff",
            "TimeUS,GyrX,GyrY,GyrZ",
        )
        .await?;

        // self.write_fmt(
        //     TYPE_GPS,
        //     size_of::<GpsMsg>() as u8,
        //     "GPS",
        //     "QBIHBfLLffff",
        //     "TimeUS,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ",
        // )
        // .await?;

        self.write_fmt(
            TYPE_BATTERY,
            size_of::<BatteryMsg>() as u8,
            "BAT",
            "Qfff",
            "TimeUS,Volt,Curr,CurrTot",
        )
        .await?;

        self.write_fmt(
            TYPE_RC_CHANNELS,
            size_of::<RcChannelsMsg>() as u8,
            "RCIN",
            "QHHHHHHHHHHHHHHHH",
            "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14,C15,C16",
        )
        .await?;

        // self.write_fmt(
        //     TYPE_MOTOR_OUTPUT,
        //     size_of::<RcouMsg>() as u8,
        //     "RCOU",
        //     "QHHHHHHHHHHHHHH",
        //     "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14",
        // )
        // .await?;

        for (id, label) in [
            (b's', "s"),
            (b'm', "m"),
            (b'n', "m/s"),
            (b'o', "m/s/s"),
            (b'r', "rad/s"),
            (b'd', "deg"),
            (b'A', "A"),
            (b'V', "V"),
            (b'h', "mAh"),
            (b'J', "Wh"),
            (b'O', "Ohm"),
            (b'-', ""),
        ] {
            self.write(&UnitMsg {
                sync: SYNC,
                type_: TYPE_UNIT,
                time_us: U64::new(t),
                id,
                label: pack(label),
            })
            .await?;
        }

        // Multiplier id chars:
        //   'F' = 1e-6  (for TimeUS: stored as µs, displayed as seconds)
        //   '0' = 1.0   (no scaling)
        //   '-' = no multiplier
        for (id, val) in [(b'F', 1e-6_f64), (b'0', 1.0_f64), (b'-', 0.0_f64)] {
            self.write(&MultMsg {
                sync: SYNC,
                type_: TYPE_MULT,
                time_us: U64::new(t),
                id,
                val: val.to_le_bytes(),
            })
            .await?;
        }

        self.write(&FmtuMsg {
            sync: SYNC,
            type_: TYPE_FMTU,
            time_us: U64::new(t),
            fmt_type: TYPE_ACCEL,
            units: pack("s-sooo"),
            mults: pack("F-F000"),
        })
        .await?;

        self.write(&FmtuMsg {
            sync: SYNC,
            type_: TYPE_FMTU,
            time_us: U64::new(t),
            fmt_type: TYPE_GYRO,
            units: pack("s-srrr"),
            mults: pack("F-F000"),
        })
        .await?;

        // self.write(&FmtuMsg {
        //     sync: SYNC,
        //     type_: TYPE_FMTU,
        //     time_us: U64::new(t),
        //     fmt_type: TYPE_GPS,
        //     units: pack("s----−ddmnddn"),
        //     mults: pack("F----000000000"),
        // })
        // .await?;

        self.write(&FmtuMsg {
            sync: SYNC,
            type_: TYPE_FMTU,
            time_us: U64::new(t),
            fmt_type: TYPE_BATTERY,
            units: pack("sVAh"),
            mults: pack("F000"),
        })
        .await?;

        self.write(&FmtuMsg {
            sync: SYNC,
            type_: TYPE_FMTU,
            time_us: U64::new(t),
            fmt_type: TYPE_RC_CHANNELS,
            units: pack("s----------------"),
            mults: pack("F----------------"),
        })
        .await?;

        self.write_fmt(
            TYPE_RC_LINK_STATS,
            size_of::<RcLinkStatsMsg>() as u8,
            "RSL",
            "QBBBB",
            "TimeUS,UplinkLinkQuality,UplinkRSSI,DownlinkLinkQuality,DownlinkRSSI",
        )
        .await?;

        self.write(&FmtuMsg {
            sync: SYNC,
            type_: TYPE_FMTU,
            time_us: U64::new(t),
            fmt_type: TYPE_RC_LINK_STATS,
            units: pack("s----"),
            mults: pack("F----"),
        })
        .await?;

        self.write_fmt(
            TYPE_MOTOR_OUTPUT,
            size_of::<MotorOutputMsg>() as u8,
            "MOT",
            "QHHH",
            "TimeUS,Throttle,LeftElev,RightElev",
        )
        .await?;

        self.write_fmt(
            TYPE_NAV_TARGET,
            size_of::<NavTargetMsg>() as u8,
            "TAR",
            "Qffff",
            "TimeUS,Throttle,RateRoll,RatePitch,RateYaw",
        )
        .await?;

        self.write(&FmtuMsg {
            sync: SYNC,
            type_: TYPE_FMTU,
            time_us: U64::new(t),
            fmt_type: TYPE_MOTOR_OUTPUT,
            units: pack("s---"),
            mults: pack("F---"),
        })
        .await?;

        self.write(&FmtuMsg {
            sync: SYNC,
            type_: TYPE_FMTU,
            time_us: U64::new(t),
            fmt_type: TYPE_NAV_TARGET,
            units: pack("s-rrr"),
            mults: pack("F-000"),
        })
        .await?;

        self.write(&MsgMsg {
            sync: SYNC,
            type_: TYPE_MSG,
            time_us: U64::new(t),
            message: pack("Toto Plane"), // TODO: change for cfg craft name
        })
        .await?;

        // TODO: what parameters should we log
        self.write(&ParmMsg {
            sync: SYNC,
            type_: TYPE_PARM,
            time_us: U64::new(t),
            name: pack("LOG_VERSION"),
            value: 1.0_f64.to_le_bytes(),
        })
        .await?;

        Ok(())
    }

    pub async fn write_message(&mut self, time_us: u64, msg: &str) -> Result<(), W::Error> {
        self.write(&MsgMsg {
            sync: SYNC,
            type_: TYPE_MSG,
            time_us: U64::new(time_us),
            message: pack(msg), // drops extra characters
        })
        .await
    }

    pub async fn write_accel(&mut self, time_us: u64, accel: Vector3<f32>) -> Result<(), W::Error> {
        self.write(&AccelMsg {
            sync: SYNC,
            type_: TYPE_ACCEL,
            time_us: U64::new(time_us),
            accel_x: F32::new(accel.x),
            accel_y: F32::new(accel.y),
            accel_z: F32::new(accel.z),
        })
        .await
    }

    pub async fn write_gyro(&mut self, time_us: u64, gyro: Vector3<f32>) -> Result<(), W::Error> {
        self.write(&GyroMsg {
            sync: SYNC,
            type_: TYPE_GYRO,
            time_us: U64::new(time_us),
            gyro_x: F32::new(gyro.x),
            gyro_y: F32::new(gyro.y),
            gyro_z: F32::new(gyro.z),
        })
        .await
    }

    // pub async fn write_gps(
    //     &mut self,
    //     time_us: u64,
    //     status: u8,
    //     gms: u32,
    //     gwk: u16,
    //     nsats: u8,
    //     hdop: f32,
    //     lat: i32,
    //     lng: i32,  // × 1e7 degrees
    //     alt: f32,  // metres MSL
    //     spd: f32,  // m/s ground speed
    //     gcrs: f32, // degrees ground course
    //     vz: f32,   // m/s vertical (positive = down)
    // ) -> Result<(), W::Error> {
    //     self.write(&GpsMsg {
    //         sync: SYNC,
    //         type_: TYPE_GPS,
    //         time_us: U64::new(time_us),
    //         status,
    //         gms: U32::new(gms),
    //         gwk: U16::new(gwk),
    //         nsats,
    //         hdop: F32::new(hdop),
    //         lat: I32::new(lat),
    //         lng: I32::new(lng),
    //         alt: F32::new(alt),
    //         spd: F32::new(spd),
    //         gcrs: F32::new(gcrs),
    //         vz: F32::new(vz),
    //     })
    //     .await
    // }

    pub async fn write_bat(
        &mut self,
        time_us: u64,
        volt: f32,
        curr: f32,
        curr_tot: f32,
    ) -> Result<(), W::Error> {
        self.write(&BatteryMsg {
            sync: SYNC,
            type_: TYPE_BATTERY,
            time_us: U64::new(time_us),
            volt: F32::new(volt),
            curr: F32::new(curr),
            curr_tot: F32::new(curr_tot),
        })
        .await
    }

    pub async fn write_rc_channels(
        &mut self,
        time_us: u64,
        channels: &[u16; 16],
    ) -> Result<(), W::Error> {
        self.write(&RcChannelsMsg {
            sync: SYNC,
            type_: TYPE_RC_CHANNELS,
            time_us: U64::new(time_us),
            channels: channels.map(U16::new),
        })
        .await
    }

    pub async fn write_rc_link_stats(
        &mut self,
        time_us: u64,
        uplink_link_quality: u8,
        uplink_rssi: u8,
        downlink_link_quality: u8,
        downlink_rssi: u8,
    ) -> Result<(), W::Error> {
        self.write(&RcLinkStatsMsg {
            sync: SYNC,
            type_: TYPE_RC_LINK_STATS,
            time_us: U64::new(time_us),
            uplink_link_quality,
            uplink_rssi,
            downlink_link_quality,
            downlink_rssi,
        })
        .await
    }

    pub async fn write_motor_output(
        &mut self,
        time_us: u64,
        throttle: u16,
        left_elevon: u16,
        right_elevon: u16,
    ) -> Result<(), W::Error> {
        self.write(&MotorOutputMsg {
            sync: SYNC,
            type_: TYPE_MOTOR_OUTPUT,
            time_us: U64::new(time_us),
            throttle: U16::new(throttle),
            left_elevon: U16::new(left_elevon),
            right_elevon: U16::new(right_elevon),
        })
        .await
    }

    pub async fn write_nav_target(
        &mut self,
        time_us: u64,
        throttle: f32,
        rate: (f32, f32, f32),
    ) -> Result<(), W::Error> {
        self.write(&NavTargetMsg {
            sync: SYNC,
            type_: TYPE_NAV_TARGET,
            time_us: U64::new(time_us),
            throttle: F32::new(throttle),
            rate_roll: F32::new(rate.0),
            rate_pitch: F32::new(rate.1),
            rate_yaw: F32::new(rate.2),
        })
        .await
    }

    pub async fn write_parm(
        &mut self,
        time_us: u64,
        name: &str,
        value: f64,
    ) -> Result<(), W::Error> {
        self.write(&ParmMsg {
            sync: SYNC,
            type_: TYPE_PARM,
            time_us: U64::new(time_us),
            name: pack(name),
            value: value.to_le_bytes(),
        })
        .await
    }
}

const SYNC: [u8; 2] = [0xA3, 0x95];
const FMT_TYPE_ID: u8 = 0x80;

// Message type IDs
const TYPE_FMT: u8 = 0x80;
const TYPE_UNIT: u8 = 0x81;
const TYPE_MULT: u8 = 0x82;
const TYPE_FMTU: u8 = 0x83;
const TYPE_MSG: u8 = 0x84;
const TYPE_PARM: u8 = 0x85;

const TYPE_ACCEL: u8 = 0x01;
const TYPE_GYRO: u8 = 0x02;
const TYPE_GPS: u8 = 0x03;
const TYPE_BATTERY: u8 = 0x04;
const TYPE_RC_CHANNELS: u8 = 0x05;
const TYPE_RC_LINK_STATS: u8 = 0x06;
const TYPE_FLIGHT_STATE: u8 = 0x08;
const TYPE_MOTOR_OUTPUT: u8 = 0x09;
const TYPE_NAV_TARGET: u8 = 0x0A;

// Messages: every message starts with: sync[2] + type[1].

#[derive(IntoBytes, KnownLayout, Immutable)]
#[repr(C, packed)]
struct FmtMsg {
    sync: [u8; 2],
    type_: u8,   // always FMT_TYPE_ID = 0x80
    type_id: u8, // type being described
    length: u8,  // total byte length of that message type
    name: [u8; 4],
    format: [u8; 16],
    columns: [u8; 64],
}

#[derive(IntoBytes, KnownLayout, Immutable)]
#[repr(C, packed)]
struct UnitMsg {
    sync: [u8; 2],
    type_: u8,
    time_us: U64<LE>,
    id: u8,
    label: [u8; 64],
}

#[derive(IntoBytes, KnownLayout, Immutable)]
#[repr(C, packed)]
struct MultMsg {
    sync: [u8; 2],
    type_: u8,
    time_us: U64<LE>,
    id: u8,
    val: [u8; 8], // double stored as 8 raw bytes (little-endian f64)
}

#[derive(IntoBytes, KnownLayout, Immutable)]
#[repr(C, packed)]
struct FmtuMsg {
    sync: [u8; 2],
    type_: u8,
    time_us: U64<LE>,
    fmt_type: u8, // which TYPE_* this applies to
    units: [u8; 16],
    mults: [u8; 16],
}

#[derive(IntoBytes, KnownLayout, Immutable)]
#[repr(C, packed)]
struct MsgMsg {
    sync: [u8; 2],
    type_: u8,
    time_us: U64<LE>,
    message: [u8; 64],
}

#[derive(IntoBytes, KnownLayout, Immutable)]
#[repr(C, packed)]
struct ParmMsg {
    sync: [u8; 2],
    type_: u8,
    time_us: U64<LE>,
    name: [u8; 16],
    value: [u8; 8], // f64 little-endian
}

#[derive(IntoBytes, KnownLayout, Immutable)]
#[repr(C, packed)]
struct ArmMsg {
    sync: [u8; 2],
    type_: u8,
    time_us: U64<LE>,
    arm_state: u8, // 1 = armed, 0 = disarmed
    arm_checks: U32<LE>,
    forced: u8,
    method: u8,
}

#[derive(IntoBytes, KnownLayout, Immutable)]
#[repr(C, packed)]
struct AccelMsg {
    sync: [u8; 2],
    type_: u8,
    time_us: U64<LE>,
    accel_x: F32<LE>,
    accel_y: F32<LE>,
    accel_z: F32<LE>,
}

#[derive(IntoBytes, KnownLayout, Immutable)]
#[repr(C, packed)]
struct GyroMsg {
    sync: [u8; 2],
    type_: u8,
    time_us: U64<LE>,
    gyro_x: F32<LE>,
    gyro_y: F32<LE>,
    gyro_z: F32<LE>,
}

// #[derive(IntoBytes, KnownLayout, Immutable)]
// #[repr(C, packed)]
// struct GpsMsg {
//     sync: [u8; 2],
//     type_: u8,
//     time_us: U64<LE>,
//     status: u8,    // 0=no fix, 1=2D, 2=3D, 3=3D+DGPS, 4=3D+RTK
//     gms: U32<LE>,  // GPS time milliseconds from start of week
//     gwk: U16<LE>,  // GPS week
//     nsats: u8,     // number of satellites
//     hdop: F32<LE>, // horizontal dilution of precision
//     lat: I32<LE>,  // latitude  × 1e7 degrees
//     lng: I32<LE>,  // longitude × 1e7 degrees
//     alt: F32<LE>,  // altitude MSL (m)
//     spd: F32<LE>,  // ground speed (m/s)
//     gcrs: F32<LE>, // ground course (degrees)
//     vz: F32<LE>,   // vertical speed (m/s, positive = down)
// }

#[derive(IntoBytes, KnownLayout, Immutable)]
#[repr(C, packed)]
struct BatteryMsg {
    sync: [u8; 2],
    type_: u8,
    time_us: U64<LE>,
    volt: F32<LE>,     // volts
    curr: F32<LE>,     // amps
    curr_tot: F32<LE>, // total mAh consumed
}

#[derive(IntoBytes, KnownLayout, Immutable)]
#[repr(C, packed)]
struct RcChannelsMsg {
    sync: [u8; 2],
    type_: u8,
    time_us: U64<LE>,
    channels: [U16<LE>; 16],
}

#[derive(IntoBytes, KnownLayout, Immutable)]
#[repr(C, packed)]
struct RcLinkStatsMsg {
    sync: [u8; 2],
    type_: u8,
    time_us: U64<LE>,
    uplink_link_quality: u8,
    uplink_rssi: u8,
    downlink_link_quality: u8,
    downlink_rssi: u8,
}

#[derive(IntoBytes, KnownLayout, Immutable)]
#[repr(C, packed)]
struct MotorOutputMsg {
    sync: [u8; 2],
    type_: u8,
    time_us: U64<LE>,
    throttle: U16<LE>,
    left_elevon: U16<LE>,
    right_elevon: U16<LE>,
}

#[derive(IntoBytes, KnownLayout, Immutable)]
#[repr(C, packed)]
struct NavTargetMsg {
    sync: [u8; 2],
    type_: u8,
    time_us: U64<LE>,
    throttle: F32<LE>,
    rate_roll: F32<LE>,
    rate_pitch: F32<LE>,
    rate_yaw: F32<LE>,
}

// #[derive(IntoBytes, KnownLayout, Immutable)]
// #[repr(C, packed)]
// struct RcouMsg {
//     sync: [u8; 2],
//     type_: u8,
//     time_us: U64<LE>,
//     chan: [U16<LE>; 14],
// }
