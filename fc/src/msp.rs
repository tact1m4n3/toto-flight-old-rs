use embassy_sync::channel::Channel;
use embedded_io_async::{Read, ReadExactError, Write};

use crate::{RawMutex, control, utils::SyncWatch};

static REQUESTS: Channel<RawMutex, MessageType, 8> = Channel::new();
// NOTE: there might be a better place for this
pub static CHANNELS: SyncWatch<RawMutex, [u16; 16]> = SyncWatch::new();

pub async fn task_read<R: Read>(mut reader: R) {
    let mut buf = [0u8; 64];
    loop {
        match next_message(&mut reader, &mut buf).await {
            Ok(Message::Request { typ }) => {
                REQUESTS.send(typ).await;
                // info!("msp: received message request {}", typ);
            }
            Ok(Message::Response { typ, payload }) => {
                _ = typ;
                _ = payload;
                info!("msp: received message response {}: {}", typ, payload);
            }
            Err(_) => {
                warn!("msp: io error");
            }
        }
    }
}

pub async fn task_write<W: Write>(mut writer: W) {
    loop {
        let typ = REQUESTS.receive().await;
        match typ {
            MessageType::Rc => {
                if let Some(channels) = CHANNELS.get() {
                    let rc = message::Rc(channels);
                    rc.send(&mut writer).await.ok();
                }
            }
            MessageType::OsdCanvas => {
                let canvas = message::OsdCanvas { cols: 50, rows: 18 };
                canvas.send(&mut writer).await.ok();
            }
            MessageType::VtxConfig => {
                // We don't specify any valid VTX config, we let the vtx load it from persistent memory
                let vtx_config = message::VtxConfig {
                    vtx_type: 0,
                    band: 0,
                    channel: 0,
                    power: 0,
                    pit_mode: 0,
                    freq: 0,
                    is_ready: 0,
                    low_power_disarm: 0,
                };
                vtx_config.send(&mut writer).await.ok();
            }
            MessageType::FcVariant => {
                let fc_variant = message::FcVariant(*b"TOTO");
                fc_variant.send(&mut writer).await.ok();
            }
            MessageType::Status => {
                // We only care about the arm state, this is guaranteed to be the first bit of the mode_flags field
                let status = message::Status {
                    cycle_time: 0,
                    i2c_error_counter: 0,
                    sensor_present: u16::MAX,
                    mode_flags: control::status().arm_state.is_armed() as u32,
                    current_profile: 1,
                };
                status.send(&mut writer).await.ok();
            }
            _ => {
                warn!("msp request for unimplemented type {}", typ);
            }
        }
    }
}

pub async fn next_message<'a, R: Read>(
    reader: &mut R,
    payload_buf: &'a mut [u8],
) -> Result<Message<'a>, ReadExactError<R::Error>> {
    let mut reader = Reader { inner: reader };

    loop {
        if reader.read_u8().await? != b'$' {
            continue;
        }

        if reader.read_u8().await? != b'M' {
            continue;
        }

        if Direction::try_from(reader.read_u8().await?).is_err() {
            warn!("msp: invalid direction");
            continue;
        }

        let size = reader.read_u8().await?;

        let message_type_byte = reader.read_u8().await?;
        let Ok(message_type) = MessageType::try_from(message_type_byte) else {
            warn!("msp: unknown message type: {}", message_type_byte);
            continue;
        };

        if size as usize > payload_buf.len() {
            warn!("msp: payload too long");
            continue;
        }

        if size > 0 {
            reader
                .inner
                .read_exact(&mut payload_buf[..size as usize])
                .await?;
        }

        let expected_crc = reader.read_u8().await?;

        let mut actual_crc = size ^ message_type_byte;
        for byte in &payload_buf[..size as usize] {
            actual_crc ^= *byte;
        }

        if expected_crc != actual_crc {
            warn!("msp: crc byte mismatch");
            continue;
        }

        if size == 0 {
            return Ok(Message::Request { typ: message_type });
        } else {
            return Ok(Message::Response {
                typ: message_type,
                payload: &payload_buf[..size as usize],
            });
        }
    }
}

pub enum Message<'a> {
    Request { typ: MessageType },
    Response { typ: MessageType, payload: &'a [u8] },
}

#[repr(u8)]
#[derive(Clone, Copy, num_enum::TryFromPrimitive)]
pub enum Direction {
    ToFC = b'<',
    FromFC = b'>',
}

#[repr(u8)]
#[derive(Clone, Copy, num_enum::TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(non_camel_case_types)]
pub enum MessageType {
    ApiVersion = 1,
    FcVariant = 2,
    FcVersion = 3,
    BoardInfo = 4,
    BuildInfo = 5,

    Name = 10,
    SetName = 11,

    // Cleanflight original features (32-62)
    BatteryConfig = 32,
    SetBatteryConfig = 33,
    ModeRanges = 34,
    SetModeRange = 35,
    FeatureConfig = 36,
    SetFeatureConfig = 37,
    BoardAlignmentConfig = 38,
    SetBoardAlignmentConfig = 39,
    CurrentMeterConfig = 40,
    SetCurrentMeterConfig = 41,
    MixerConfig = 42,
    SetMixerConfig = 43,
    RxConfig = 44,
    SetRxConfig = 45,
    LedColors = 46,
    SetLedColors = 47,
    LedStripConfig = 48,
    SetLedStripConfig = 49,
    RssiConfig = 50,
    SetRssiConfig = 51,
    AdjustmentRanges = 52,
    SetAdjustmentRange = 53,
    CfSerialConfig = 54,
    SetCfSerialConfig = 55,
    VoltageMeterConfig = 56,
    SetVoltageMeterConfig = 57,
    SonarAltitude = 58,
    PidController = 59,
    SetPidController = 60,
    ArmingConfig = 61,
    SetArmingConfig = 62,

    // Baseflight MSP commands (64-89)
    RxMap = 64,
    SetRxMap = 65,
    Reboot = 68,
    DataflashSummary = 70,
    DataflashRead = 71,
    DataflashErase = 72,
    FailsafeConfig = 75,
    SetFailsafeConfig = 76,
    RxfailConfig = 77,
    SetRxfailConfig = 78,
    SdcardSummary = 79,
    BlackboxConfig = 80,
    SetBlackboxConfig = 81,
    TransponderConfig = 82,
    SetTransponderConfig = 83,
    OsdConfig = 84,
    SetOsdConfig = 85,
    OsdCharRead = 86,
    OsdCharWrite = 87,
    VtxConfig = 88,
    SetVtxConfig = 89,

    // Betaflight Additional Commands (90-99)
    AdvancedConfig = 90,
    SetAdvancedConfig = 91,
    FilterConfig = 92,
    SetFilterConfig = 93,
    PidAdvanced = 94,
    SetPidAdvanced = 95,
    SensorConfig = 96,
    SetSensorConfig = 97,
    CameraControl = 98,
    SetArmingDisabled = 99,

    // Multiwii original MSP commands (101-139)
    Status = 101,
    RawImu = 102,
    Servo = 103,
    Motor = 104,
    Rc = 105,
    RawGps = 106,
    CompGps = 107,
    Attitude = 108,
    Altitude = 109,
    Analog = 110,
    RcTuning = 111,
    Pid = 112,
    Boxnames = 116,
    Pidnames = 117,
    Wp = 118,
    Boxids = 119,
    ServoConfigurations = 120,
    NavStatus = 121,
    NavConfig = 122,
    Motor3dConfig = 124,
    RcDeadband = 125,
    SensorAlignment = 126,
    LedStripModecolor = 127,
    VoltageMeters = 128,
    CurrentMeters = 129,
    BatteryState = 130,
    MotorConfig = 131,
    GpsConfig = 132,
    CompassConfig = 133,
    EscSensorData = 134,
    GpsRescue = 135,
    GpsRescuePids = 136,
    VtxtableBand = 137,
    VtxtablePowerlevel = 138,
    MotorTelemetry = 139,

    // Simplified tuning commands (140-145)
    SimplifiedTuning = 140,
    SetSimplifiedTuning = 141,
    CalculateSimplifiedPid = 142,
    CalculateSimplifiedGyro = 143,
    CalculateSimplifiedDterm = 144,
    ValidateSimplifiedTuning = 145,

    // Additional non-MultiWii commands (150-166)
    StatusEx = 150,
    Uid = 160,
    Gpssvinfo = 164,
    Gpsstatistics = 166,
    AttitudeQuaternion = 167,

    // OSD specific commands (180-189)
    OsdVideoConfig = 180,
    SetOsdVideoConfig = 181,
    Displayport = 182,
    CopyProfile = 183,
    BeeperConfig = 184,
    SetBeeperConfig = 185,
    SetTxInfo = 186,
    TxInfo = 187,
    SetOsdCanvas = 188,
    OsdCanvas = 189,

    // Set commands (200-229)
    SetRawRc = 200,
    SetRawGps = 201,
    SetPid = 202,
    SetRcTuning = 204,
    AccCalibration = 205,
    MagCalibration = 206,
    ResetConf = 208,
    SetWp = 209,
    SelectSetting = 210,
    SetHeading = 211,
    SetServoConfiguration = 212,
    SetMotor = 214,
    SetNavConfig = 215,
    SetMotor3dConfig = 217,
    SetRcDeadband = 218,
    SetResetCurrPid = 219,
    SetSensorAlignment = 220,
    SetLedStripModecolor = 221,
    SetMotorConfig = 222,
    SetGpsConfig = 223,
    SetCompassConfig = 224,
    SetGpsRescue = 225,
    SetGpsRescuePids = 226,
    SetVtxtableBand = 227,
    SetVtxtablePowerlevel = 228,

    // Multiple MSP and special commands (230-255)
    MultipleMsp = 230,
    ModeRangesExtra = 238,
    SetAccTrim = 239,
    AccTrim = 240,
    ServoMixRules = 241,
    SetServoMixRule = 242,
    SetPassthrough = 245,
    SetRtc = 246,
    Rtc = 247,
    SetBoardInfo = 248,
    SetSignature = 249,
    EepromWrite = 250,
    Reserve1 = 251,
    Reserve2 = 252,
    Debugmsg = 253,
    Debug = 254,
    V2Frame = 255,
}

#[allow(unused)]
mod message {
    use embedded_io_async::Write;

    use crate::msp::{Direction, MessageType, MessageWriter};

    pub struct Rc(pub [u16; 16]);

    impl Rc {
        pub async fn send<W: Write>(&self, writer: &mut W) -> Result<(), W::Error> {
            let mut writer =
                MessageWriter::begin(writer, Direction::FromFC, 32, MessageType::Rc).await?;
            for channel in self.0.iter() {
                writer.write(&channel.to_le_bytes()).await?;
            }
            writer.end().await?;
            Ok(())
        }
    }

    pub struct OsdCanvas {
        pub cols: u8,
        pub rows: u8,
    }

    impl OsdCanvas {
        pub async fn send<W: Write>(&self, writer: &mut W) -> Result<(), W::Error> {
            let mut writer =
                MessageWriter::begin(writer, Direction::FromFC, 2, MessageType::OsdCanvas).await?;
            writer.write(&[self.cols, self.rows]).await?;
            writer.end().await?;
            Ok(())
        }
    }

    pub struct VtxConfig {
        pub vtx_type: u8,
        pub band: u8,
        pub channel: u8,
        pub power: u8,
        pub pit_mode: u8,
        pub freq: u16,
        pub is_ready: u8,
        pub low_power_disarm: u8,
    }

    impl VtxConfig {
        pub async fn send<W: Write>(&self, writer: &mut W) -> Result<(), W::Error> {
            let mut writer =
                MessageWriter::begin(writer, Direction::FromFC, 9, MessageType::VtxConfig).await?;

            writer
                .write(&[
                    self.vtx_type,
                    self.band,
                    self.channel,
                    self.power,
                    self.pit_mode,
                ])
                .await?;
            writer.write(&self.freq.to_le_bytes()).await?;
            writer
                .write(&[self.is_ready, self.low_power_disarm])
                .await?;

            writer.end().await?;
            Ok(())
        }
    }

    pub struct FcVariant(pub [u8; 4]);

    impl FcVariant {
        pub async fn send<W: Write>(&self, writer: &mut W) -> Result<(), W::Error> {
            let mut writer =
                MessageWriter::begin(writer, Direction::FromFC, 4, MessageType::FcVariant).await?;
            writer.write(&self.0).await?;
            writer.end().await?;
            Ok(())
        }
    }

    pub struct Status {
        pub cycle_time: u16,
        pub i2c_error_counter: u16,
        pub sensor_present: u16,
        pub mode_flags: u32,
        pub current_profile: u8,
    }

    impl Status {
        pub async fn send<W: Write>(&self, writer: &mut W) -> Result<(), W::Error> {
            let mut writer =
                MessageWriter::begin(writer, Direction::FromFC, 11, MessageType::Status).await?;

            writer.write(&self.cycle_time.to_le_bytes()).await?;
            writer.write(&self.i2c_error_counter.to_le_bytes()).await?;
            writer.write(&self.sensor_present.to_le_bytes()).await?;
            writer.write(&self.mode_flags.to_le_bytes()).await?;
            writer.write(&[self.current_profile]).await?;

            writer.end().await?;
            Ok(())
        }
    }
}

pub struct Reader<'a, R: Read> {
    inner: &'a mut R,
}

impl<'a, R: Read> Reader<'a, R> {
    #[inline(always)]
    pub async fn read_u8(&mut self) -> Result<u8, ReadExactError<R::Error>> {
        Ok(self.read::<1>().await?[0])
    }
    #[inline(always)]
    pub async fn read_u16_le(&mut self) -> Result<u16, ReadExactError<R::Error>> {
        Ok(u16::from_le_bytes(self.read::<2>().await?))
    }
    #[inline(always)]
    pub async fn read_u16_be(&mut self) -> Result<u16, ReadExactError<R::Error>> {
        Ok(u16::from_be_bytes(self.read::<2>().await?))
    }
    #[inline(always)]
    pub async fn read_i16_le(&mut self) -> Result<i16, ReadExactError<R::Error>> {
        Ok(i16::from_le_bytes(self.read::<2>().await?))
    }
    #[inline(always)]
    pub async fn read_i16_be(&mut self) -> Result<i16, ReadExactError<R::Error>> {
        Ok(i16::from_be_bytes(self.read::<2>().await?))
    }
    #[inline(always)]
    pub async fn read_u32_le(&mut self) -> Result<u32, ReadExactError<R::Error>> {
        Ok(u32::from_le_bytes(self.read::<4>().await?))
    }
    #[inline(always)]
    pub async fn read_u32_be(&mut self) -> Result<u32, ReadExactError<R::Error>> {
        Ok(u32::from_be_bytes(self.read::<4>().await?))
    }
    #[inline(always)]
    pub async fn read_i32_le(&mut self) -> Result<i32, ReadExactError<R::Error>> {
        Ok(i32::from_le_bytes(self.read::<4>().await?))
    }
    #[inline(always)]
    pub async fn read_i32_be(&mut self) -> Result<i32, ReadExactError<R::Error>> {
        Ok(i32::from_be_bytes(self.read::<4>().await?))
    }
    #[inline(always)]
    pub async fn read<const COUNT: usize>(
        &mut self,
    ) -> Result<[u8; COUNT], ReadExactError<R::Error>> {
        let mut buf = [0u8; COUNT];
        self.inner.read_exact(&mut buf).await?;
        Ok(buf)
    }
}

pub struct MessageWriter<'a, W: Write> {
    inner: &'a mut W,
    crc: u8,
}

impl<'a, W: Write> MessageWriter<'a, W> {
    pub async fn begin(
        inner: &'a mut W,
        dir: Direction,
        size: u8,
        typ: MessageType,
    ) -> Result<Self, W::Error> {
        inner
            .write_all(&[b'$', b'M', dir as u8, size, typ as u8])
            .await?;

        Ok(Self {
            inner,
            crc: size ^ typ as u8,
        })
    }

    pub async fn end(self) -> Result<(), W::Error> {
        self.inner.write_all(&[self.crc]).await?;
        Ok(())
    }

    pub async fn write(&mut self, bytes: &[u8]) -> Result<(), W::Error> {
        self.inner.write_all(bytes).await?;
        for byte in bytes {
            self.crc ^= *byte;
        }
        self.inner.flush().await?;
        Ok(())
    }
}
