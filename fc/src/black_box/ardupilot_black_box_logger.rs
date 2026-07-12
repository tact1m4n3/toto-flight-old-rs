use core::mem::size_of;
use embassy_time::Instant;
use nalgebra::Vector3;
use zerocopy::byteorder::{F32, LE, U16, U64};
use zerocopy::{I32, Immutable, IntoBytes, KnownLayout};

use crate::utils::pack;

use embedded_io_async::Write;

pub async fn write_header<W: Write>(writer: &mut W) -> Result<(), W::Error> {
    let t = Instant::now().as_micros();

    write_fmt_msg(
        writer,
        TYPE_FMT,
        89,
        "FMT",
        "BBnNZ",
        "Type,Length,Name,Format,Columns",
    )
    .await?;
    write_fmt_msg(
        writer,
        TYPE_UNIT,
        size_of::<UnitMsg>() as u8,
        "UNIT",
        "QbZ",
        "TimeUS,Id,Label",
    )
    .await?;
    write_fmt_msg(
        writer,
        TYPE_MULT,
        size_of::<MultMsg>() as u8,
        "MULT",
        "QbQ",
        "TimeUS,Id,Mult",
    )
    .await?;
    write_fmt_msg(
        writer,
        TYPE_FMTU,
        size_of::<FmtuMsg>() as u8,
        "FMTU",
        "QBNN",
        "TimeUS,FmtType,UnitIds,MultIds",
    )
    .await?;
    write_fmt_msg(
        writer,
        TYPE_MSG,
        size_of::<MsgMsg>() as u8,
        "MSG",
        "QZ",
        "TimeUS,Message",
    )
    .await?;
    write_fmt_msg(
        writer,
        TYPE_PARM,
        size_of::<ParmMsg>() as u8,
        "PARM",
        "QNd",
        "TimeUS,Name,Value",
    )
    .await?;
    write_fmt_msg(
        writer,
        TYPE_ACCEL,
        size_of::<AccelMsg>() as u8,
        "ACC",
        "Qfff",
        "TimeUS,AccX,AccY,AccZ",
    )
    .await?;
    write_fmt_msg(
        writer,
        TYPE_GYRO,
        size_of::<GyroMsg>() as u8,
        "GYR",
        "Qfff",
        "TimeUS,GyrX,GyrY,GyrZ",
    )
    .await?;
    write_fmt_msg(
        writer,
        TYPE_GPS,
        size_of::<GpsMsg>() as u8,
        "GPS",
        "QBBfLLffff",
        "TimeUS,Status,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ",
    )
    .await?;
    write_fmt_msg(
        writer,
        TYPE_BATTERY,
        size_of::<BatteryMsg>() as u8,
        "BAT",
        "Qfff",
        "TimeUS,Volt,Curr,CurrTot",
    )
    .await?;
    write_fmt_msg(
        writer,
        TYPE_RC_CHANNELS,
        size_of::<RcChannelsMsg>() as u8,
        "RCIN",
        "QHHHHHHHHHHHHHHHH",
        "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14,C15,C16",
    )
    .await?;
    write_fmt_msg(
        writer,
        TYPE_RC_LINK_STATS,
        size_of::<RcLinkStatsMsg>() as u8,
        "RSL",
        "QBBBB",
        "TimeUS,UplinkLinkQuality,UplinkRSSI,DownlinkLinkQuality,DownlinkRSSI",
    )
    .await?;
    write_fmt_msg(
        writer,
        TYPE_ATTITUDE,
        size_of::<AttitudeMsg>() as u8,
        "ATT",
        "Qfff",
        "TimeUS,Roll,Pitch,Yaw",
    )
    .await?;

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
        write(
            writer,
            &UnitMsg {
                sync: SYNC,
                type_: TYPE_UNIT,
                time_us: U64::new(t),
                id,
                label: pack(label),
            },
        )
        .await?;
    }

    for (id, val) in [(b'F', 1e-6_f64), (b'0', 1.0_f64), (b'-', 0.0_f64)] {
        write(
            writer,
            &MultMsg {
                sync: SYNC,
                type_: TYPE_MULT,
                time_us: U64::new(t),
                id,
                val: val.to_le_bytes(),
            },
        )
        .await?;
    }

    for fmtu in [
        FmtuMsg {
            sync: SYNC,
            type_: TYPE_FMTU,
            time_us: U64::new(t),
            fmt_type: TYPE_ACCEL,
            units: pack("s-sooo"),
            mults: pack("F-F000"),
        },
        FmtuMsg {
            sync: SYNC,
            type_: TYPE_FMTU,
            time_us: U64::new(t),
            fmt_type: TYPE_GYRO,
            units: pack("s-srrr"),
            mults: pack("F-F000"),
        },
        // TODO: gps
        FmtuMsg {
            sync: SYNC,
            type_: TYPE_FMTU,
            time_us: U64::new(t),
            fmt_type: TYPE_BATTERY,
            units: pack("sVAh"),
            mults: pack("F000"),
        },
        FmtuMsg {
            sync: SYNC,
            type_: TYPE_FMTU,
            time_us: U64::new(t),
            fmt_type: TYPE_RC_CHANNELS,
            units: pack("s----------------"),
            mults: pack("F----------------"),
        },
        FmtuMsg {
            sync: SYNC,
            type_: TYPE_FMTU,
            time_us: U64::new(t),
            fmt_type: TYPE_RC_LINK_STATS,
            units: pack("s----"),
            mults: pack("F----"),
        },
        FmtuMsg {
            sync: SYNC,
            type_: TYPE_FMTU,
            time_us: U64::new(t),
            fmt_type: TYPE_ATTITUDE,
            units: pack("s-ddd"),
            mults: pack("F-000"),
        },
    ] {
        write(writer, &fmtu).await?;
    }

    write(
        writer,
        &MsgMsg {
            sync: SYNC,
            type_: TYPE_MSG,
            time_us: U64::new(t),
            message: pack("Toto Plane"), // TODO: change for cfg craft name
        },
    )
    .await?;

    write(
        writer,
        &ParmMsg {
            sync: SYNC,
            type_: TYPE_PARM,
            time_us: U64::new(t),
            name: pack("LOG_VERSION"),
            value: 1.0_f64.to_le_bytes(),
        },
    )
    .await?;

    Ok(())
}

// pub fn write_message<W: Write>(writer: &mut W, time_us: u64, msg: &str) -> Result<(), W::Error> {
//     write(
//         writer,
//         &MsgMsg {
//             sync: SYNC,
//             type_: TYPE_MSG,
//             time_us: U64::new(time_us),
//             message: pack(msg),
//         },
//     )
// }

pub async fn write_accel<W: Write>(
    writer: &mut W,
    time_us: u64,
    accel: Vector3<f32>,
) -> Result<(), W::Error> {
    write(
        writer,
        &AccelMsg {
            sync: SYNC,
            type_: TYPE_ACCEL,
            time_us: U64::new(time_us),
            accel_x: F32::new(accel.x),
            accel_y: F32::new(accel.y),
            accel_z: F32::new(accel.z),
        },
    )
    .await
}

pub async fn write_gyro<W: Write>(
    writer: &mut W,
    time_us: u64,
    gyro: Vector3<f32>,
) -> Result<(), W::Error> {
    write(
        writer,
        &GyroMsg {
            sync: SYNC,
            type_: TYPE_GYRO,
            time_us: U64::new(time_us),
            gyro_x: F32::new(gyro.x),
            gyro_y: F32::new(gyro.y),
            gyro_z: F32::new(gyro.z),
        },
    )
    .await
}

#[allow(clippy::too_many_arguments)]
pub async fn write_gps<W: Write>(
    writer: &mut W,
    time_us: u64,
    status: u8,
    nsats: u8,
    hdop: f32,
    lat: f64,
    lng: f64,
    alt: f32,
    spd: f32,
    crs: f32,
    vz: f32,
) -> Result<(), W::Error> {
    write(
        writer,
        &GpsMsg {
            sync: SYNC,
            type_: TYPE_GPS,
            time_us: U64::new(time_us),
            status,
            nsats,
            hdop: F32::new(hdop),
            lat: I32::new((lat * 1e7) as i32),
            lng: I32::new((lng * 1e7) as i32),
            alt: F32::new(alt),
            spd: F32::new(spd),
            crs: F32::new(crs),
            vz: F32::new(vz),
        },
    )
    .await
}

pub async fn write_bat<W: Write>(
    writer: &mut W,
    time_us: u64,
    volt: f32,
    curr: f32,
    curr_tot: f32,
) -> Result<(), W::Error> {
    write(
        writer,
        &BatteryMsg {
            sync: SYNC,
            type_: TYPE_BATTERY,
            time_us: U64::new(time_us),
            volt: F32::new(volt),
            curr: F32::new(curr),
            curr_tot: F32::new(curr_tot),
        },
    )
    .await
}

pub async fn write_rc_channels<W: Write>(
    writer: &mut W,
    time_us: u64,
    channels: &[u16; 16],
) -> Result<(), W::Error> {
    write(
        writer,
        &RcChannelsMsg {
            sync: SYNC,
            type_: TYPE_RC_CHANNELS,
            time_us: U64::new(time_us),
            channels: channels.map(U16::new),
        },
    )
    .await
}

pub async fn write_rc_link_stats<W: Write>(
    writer: &mut W,
    time_us: u64,
    uplink_link_quality: u8,
    uplink_rssi: u8,
    downlink_link_quality: u8,
    downlink_rssi: u8,
) -> Result<(), W::Error> {
    write(
        writer,
        &RcLinkStatsMsg {
            sync: SYNC,
            type_: TYPE_RC_LINK_STATS,
            time_us: U64::new(time_us),
            uplink_link_quality,
            uplink_rssi,
            downlink_link_quality,
            downlink_rssi,
        },
    )
    .await
}

pub async fn write_attitude<W: Write>(
    writer: &mut W,
    time_us: u64,
    roll: f32,
    pitch: f32,
    yaw: f32,
) -> Result<(), W::Error> {
    write(
        writer,
        &AttitudeMsg {
            sync: SYNC,
            type_: TYPE_ATTITUDE,
            time_us: U64::new(time_us),
            roll: F32::new(roll),
            pitch: F32::new(pitch),
            yaw: F32::new(yaw),
        },
    )
    .await
}

async fn write<W: Write, T: IntoBytes + Immutable>(
    writer: &mut W,
    msg: &T,
) -> Result<(), W::Error> {
    writer.write_all(msg.as_bytes()).await
}

async fn write_fmt_msg<W: Write>(
    writer: &mut W,
    type_id: u8,
    length: u8,
    name: &str,
    format: &str,
    columns: &str,
) -> Result<(), W::Error> {
    write(
        writer,
        &FmtMsg {
            sync: SYNC,
            type_: FMT_TYPE_ID,
            type_id,
            length,
            name: pack(name),
            format: pack(format),
            columns: pack(columns),
        },
    )
    .await
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
const TYPE_ATTITUDE: u8 = 0x07;
// const TYPE_FLIGHT_STATE: u8 = 0x07;
// const TYPE_NAV_TARGET: u8 = 0x08;
// const TYPE_MOTOR_OUTPUT: u8 = 0x09;

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

// #[derive(IntoBytes, KnownLayout, Immutable)]
// #[repr(C, packed)]
// struct ArmMsg {
//     sync: [u8; 2],
//     type_: u8,
//     time_us: U64<LE>,
//     arm_state: u8, // 1 = armed, 0 = disarmed
//     arm_checks: U32<LE>,
//     forced: u8,
//     method: u8,
// }

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

#[derive(IntoBytes, KnownLayout, Immutable)]
#[repr(C, packed)]
struct GpsMsg {
    sync: [u8; 2],
    type_: u8,
    time_us: U64<LE>,
    status: u8,    // GPS fix type
    nsats: u8,     // number of satellites
    hdop: F32<LE>, // horizontal dilution of precision
    lat: I32<LE>,  // latitude in degrees * 1e7
    lng: I32<LE>,  // longitude in degrees * 1e7
    alt: F32<LE>,  // altitude in meters
    spd: F32<LE>,  // ground speed in m/s
    crs: F32<LE>,  // ground course in degrees
    vz: F32<LE>,   // vertical speed in m/s
}

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
struct AttitudeMsg {
    sync: [u8; 2],
    type_: u8,
    time_us: U64<LE>,
    roll: F32<LE>,  // roll in degrees
    pitch: F32<LE>, // pitch in degrees
    yaw: F32<LE>,   // yaw in degrees
}

// #[derive(IntoBytes, KnownLayout, Immutable)]
// #[repr(C, packed)]
// struct MotorOutputMsg {
//     sync: [u8; 2],
//     type_: u8,
//     time_us: U64<LE>,
//     throttle: U16<LE>,
//     left_elevon: U16<LE>,
//     right_elevon: U16<LE>,
// }

// #[derive(IntoBytes, KnownLayout, Immutable)]
// #[repr(C, packed)]
// struct NavTargetMsg {
//     sync: [u8; 2],
//     type_: u8,
//     time_us: U64<LE>,
//     throttle: F32<LE>,
//     rate_roll: F32<LE>,
//     rate_pitch: F32<LE>,
//     rate_yaw: F32<LE>,
// }
