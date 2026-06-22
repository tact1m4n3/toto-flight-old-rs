//! LSM6DSV IMU driver
//! Based on Betaflight's accgyro_spi_lsm6dsv16x.c
//!
//! Generic over embedded-hal-async SpiDevice.
//! ODR: High-Accuracy mode 1 @ 1000 Hz for both sensors.
//! Full scale: ±2000 dps gyro, ±16 g accel.

// TODO: add support for multiple data rates

use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};
use uom::si::{acceleration::standard_gravity, angular_velocity::degree_per_second};

use crate::math::{Acceleration3D, AngularVelocity3D};

pub type Lsm6dsv<SPI> = Lsm6dsvGeneric<SPI, 0x70, false>;
pub type Lsm6dsv16x<SPI> = Lsm6dsvGeneric<SPI, 0x70, false>;
pub type Lsm6dsv16bAccelAxesRev<SPI> = Lsm6dsvGeneric<SPI, 0x71, true>;
pub type Lsm6dsv16b<SPI> = Lsm6dsvGeneric<SPI, 0x71, false>;

#[derive(Debug)]
pub enum InitError<SpiE> {
    Spi(SpiE),
    WhoAmI(u8),
}

impl<SpiE> From<SpiE> for InitError<SpiE> {
    fn from(e: SpiE) -> Self {
        InitError::Spi(e)
    }
}

pub struct Lsm6dsvGeneric<SPI, const WHO_AM_I_REG: u8, const ACCEL_AXES_ORDER_REVERSED: bool> {
    spi: SPI,
}

impl<SPI, const WHO_AM_I_REG: u8, const ACCEL_AXES_ORDER_REVERSED: bool>
    Lsm6dsvGeneric<SPI, WHO_AM_I_REG, ACCEL_AXES_ORDER_REVERSED>
where
    SPI: SpiDevice,
{
    pub async fn new<D: DelayNs>(spi: SPI, delay: D) -> Result<Self, InitError<SPI::Error>> {
        let mut this = Self { spi };
        this.init(delay).await?;
        Ok(this)
    }

    async fn init<D: DelayNs>(&mut self, mut delay: D) -> Result<(), InitError<SPI::Error>> {
        // Verify device identity
        let whoami = self.read_reg(WHO_AM_I).await?;
        if whoami != WHO_AM_I_REG {
            return Err(InitError::WhoAmI(whoami));
        }

        // Software reset (sets all registers to default)
        self.write_reg(CTRL3, CTRL3_SW_RESET).await?;
        // Poll until reset completes (bit self-clears)
        loop {
            delay.delay_ms(1).await;
            let v = self.read_reg(CTRL3).await?;
            if v & CTRL3_SW_RESET == 0 {
                break;
            }
        }

        // Enable register auto-increment + Block Data Update
        // BDU prevents reading mismatched high/low bytes across a sample boundary.
        self.write_reg(CTRL3, CTRL3_IF_INC | CTRL3_BDU).await?;

        // Select High-Accuracy ODR table 1
        // (gives 1000 Hz when ODR nibble = 9, matching CTRL1_ODR_XL_1000HZ / CTRL2_ODR_G_1000HZ)
        self.write_reg(HAODR_CFG, HAODR_MODE1).await?;

        // Accelerometer: ±16 g full scale + Strong LPF2 for vibration rejection
        self.write_reg(CTRL8, CTRL8_HP_LPF2_XL_BW_STRONG | CTRL8_FS_XL_16G)
            .await?;

        // Gyroscope: ±2000 dps full scale + UltraLight LPF1 (lowest latency)
        self.write_reg(CTRL6, CTRL6_LPF1_G_BW_ULTRALIGHT | CTRL6_FS_G_2000DPS)
            .await?;

        // Enable gyroscope LPF1 digital filter
        self.write_reg(CTRL7, CTRL7_LPF1_G_EN).await?;

        // Enable accelerometer LPF2 digital filter
        self.write_reg(CTRL9, CTRL9_LPF2_XL_EN).await?;

        // Accelerometer: High-Accuracy mode, 1000 Hz ODR
        self.write_reg(CTRL1, CTRL1_OP_MODE_XL_HIGH_ACCURACY | CTRL1_ODR_XL_1000HZ)
            .await?;

        // Gyroscope: High-Accuracy mode, 1000 Hz ODR
        self.write_reg(CTRL2, CTRL2_OP_MODE_G_HIGH_ACCURACY | CTRL2_ODR_G_1000HZ)
            .await?;

        // DRDY interrupt: pulsed (clears automatically, no register read needed to unlatch)
        self.write_reg(CTRL4, CTRL4_DRDY_PULSED).await?;

        // Route gyro DRDY only to INT1 pin
        //     The interrupt fires when gyro data is ready; accel at the same ODR
        //     will be ready by the time we complete the SPI transaction.
        self.write_reg(INT1_CTRL, INT1_DRDY_G).await?;

        Ok(())
    }

    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), SPI::Error> {
        // MSB = 0 → write
        self.spi.write(&[reg & !SPI_READ, val]).await?;
        Ok(())
    }

    async fn read_reg(&mut self, reg: u8) -> Result<u8, SPI::Error> {
        let mut buf = [reg | SPI_READ, 0x00];
        self.spi.transfer_in_place(&mut buf).await?;
        Ok(buf[1])
    }
}

impl<SPI, const WHO_AM_I_REG: u8, const ACCEL_AXES_ORDER_REVERSED: bool> super::ImuSensor
    for Lsm6dsvGeneric<SPI, WHO_AM_I_REG, ACCEL_AXES_ORDER_REVERSED>
where
    SPI: SpiDevice,
{
    type Error = SPI::Error;

    async fn read(&mut self) -> Result<super::Data, Self::Error> {
        // 1 address byte + 12 data bytes
        let mut buf = [0u8; 13];
        buf[0] = OUTX_L_G | SPI_READ;
        self.spi.transfer_in_place(&mut buf).await?;

        // buf[0]  = dummy (address echo)
        // buf[1..=2]   = OUTX_L_G / OUTX_H_G  → gyro X
        // buf[3..=4]   = OUTY_L_G / OUTY_H_G  → gyro Y
        // buf[5..=6]   = OUTZ_L_G / OUTZ_H_G  → gyro Z

        let gx = i16::from_le_bytes([buf[1], buf[2]]);
        let gy = i16::from_le_bytes([buf[3], buf[4]]);
        let gz = i16::from_le_bytes([buf[5], buf[6]]);

        let (ax, ay, az) = if ACCEL_AXES_ORDER_REVERSED {
            (
                // buf[7..=8]   = OUTX_L_A / OUTX_H_A  → accel X
                // buf[9..=10]  = OUTY_L_A / OUTY_H_A  → accel Y
                // buf[11..=12] = OUTZ_L_A / OUTZ_H_A  → accel Z
                i16::from_le_bytes([buf[7], buf[8]]),
                i16::from_le_bytes([buf[9], buf[10]]),
                i16::from_le_bytes([buf[11], buf[12]]),
            )
        } else {
            (
                // buf[7..=8]   = OUTZ_L_A / OUTZ_H_A  → accel Z
                // buf[9..=10]  = OUTY_L_A / OUTY_H_A  → accel Y
                // buf[11..=12] = OUTX_L_A / OUTX_H_A  → accel X
                i16::from_le_bytes([buf[11], buf[12]]),
                i16::from_le_bytes([buf[9], buf[10]]),
                i16::from_le_bytes([buf[7], buf[8]]),
            )
        };

        Ok(super::Data {
            gyro: AngularVelocity3D::new::<degree_per_second>(
                f32::from(gx) * GYRO_SCALE_DPS,
                f32::from(gy) * GYRO_SCALE_DPS,
                f32::from(gz) * GYRO_SCALE_DPS,
            ),
            accel: Acceleration3D::new::<standard_gravity>(
                f32::from(ax) * ACCEL_SCALE_G,
                f32::from(ay) * ACCEL_SCALE_G,
                f32::from(az) * ACCEL_SCALE_G,
            ),
        })
    }
}

const WHO_AM_I: u8 = 0x0F;

const CTRL3: u8 = 0x12;
const CTRL3_SW_RESET: u8 = 0x01;
const CTRL3_IF_INC: u8 = 0x04;
const CTRL3_BDU: u8 = 0x40;

const CTRL4: u8 = 0x13;
const CTRL4_DRDY_PULSED: u8 = 0x02;

const CTRL6: u8 = 0x15;
// LPF1_G_BW [6:4] — UltraLight (widest, lowest latency) = 0b000
const CTRL6_LPF1_G_BW_ULTRALIGHT: u8 = 0x00;
const CTRL6_FS_G_2000DPS: u8 = 0x04;

const CTRL7: u8 = 0x16;
const CTRL7_LPF1_G_EN: u8 = 0x01;

const CTRL8: u8 = 0x17;
// HP_LPF2_XL_BW [7:5] = 0b110 → "Strong" LPF2 for accel vibration rejection
const CTRL8_HP_LPF2_XL_BW_STRONG: u8 = 0b110 << 5;
const CTRL8_FS_XL_16G: u8 = 0x03;

const CTRL9: u8 = 0x18;
const CTRL9_LPF2_XL_EN: u8 = 0x08;

// High-Accuracy ODR config — selects the frequency table used by ODR_XL/ODR_G nibbles
const HAODR_CFG: u8 = 0x62;
const HAODR_MODE1: u8 = 0x01; // table: …500 / 1000 / 2000 / 4000 / 8000 Hz

// CTRL1 — accelerometer: OP_MODE_XL = HIGH_ACCURACY (001), ODR_XL = 1000 Hz (9 in mode-1 table)
const CTRL1: u8 = 0x10;
const CTRL1_OP_MODE_XL_HIGH_ACCURACY: u8 = 0x01 << 4;
const CTRL1_ODR_XL_1000HZ: u8 = 0x09; // in HAODR mode-1

// CTRL2 — gyroscope: OP_MODE_G = HIGH_ACCURACY (001), ODR_G = 1000 Hz (9 in mode-1 table)
const CTRL2: u8 = 0x11;
const CTRL2_OP_MODE_G_HIGH_ACCURACY: u8 = 0x01 << 4;
const CTRL2_ODR_G_1000HZ: u8 = 0x09; // in HAODR mode-1

// INT1 pin control
const INT1_CTRL: u8 = 0x0D;
const INT1_DRDY_G: u8 = 0x02; // gyro DRDY → INT1 only

// First gyro output register — reading 12 bytes from here gives X/Y/Z gyro then X/Y/Z accel
const OUTX_L_G: u8 = 0x22;

// SPI read flag
const SPI_READ: u8 = 0x80;

/// Gyroscope scale: 70 mdps/LSB at ±2000 dps → degrees/s per LSB
pub const GYRO_SCALE_DPS: f32 = 0.070;

/// Accelerometer scale: 0.488 mg/LSB at ±16 g → g per LSB
pub const ACCEL_SCALE_G: f32 = 0.000488;
