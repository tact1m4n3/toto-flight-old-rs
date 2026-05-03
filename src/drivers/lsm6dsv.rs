//! LSM6DSV IMU driver
//! Ported from Betaflight's accgyro_spi_lsm6dsv16x.c
//!
//! Generic over embedded-hal-async SPI + CS pin.
//! Accel and gyro are read in a single 12-byte SPI transaction (6 gyro + 6 accel).
//! ODR: High-Accuracy mode 1 @ 1000 Hz for both sensors.
//! Full scale: ±2000 dps gyro, ±16 g accel.

use embedded_hal_async::spi::SpiDevice;

pub type Lsm6dsv16b<SPI> = Lsm6dsv<SPI, 0x71>;

#[derive(Debug, Clone, Copy, Default)]
pub struct ImuData {
    /// Angular rate in degrees/s
    pub gyro: [f32; 3],
    /// Linear acceleration in g
    pub accel: [f32; 3],
}

#[derive(Debug)]
pub enum Error<SpiE> {
    Spi(SpiE),
    WhoAmI(u8),
}

impl<SpiE> From<SpiE> for Error<SpiE> {
    fn from(e: SpiE) -> Self {
        Error::Spi(e)
    }
}

pub struct Lsm6dsv<SPI, const WHO_AM_I_REG: u8> {
    spi: SPI,
}

impl<SPI, const WHO_AM_I_REG: u8> Lsm6dsv<SPI, WHO_AM_I_REG>
where
    SPI: SpiDevice,
{
    /// Create a new driver instance wrapping an `SpiDevice`.
    ///
    /// `SpiDevice` handles CS assertion internally (use `embedded-hal-bus`
    /// `ExclusiveDevice` or similar to wrap a raw SPI + CS pin).
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Initialise the sensor.
    ///
    /// Call once after power-on. The caller must wait for the INT1 rising edge
    /// before calling `read()`.
    pub async fn init(&mut self) -> Result<(), Error<SPI::Error>> {
        // 1. Verify device identity
        let whoami = self.read_reg(WHO_AM_I).await?;
        if whoami != WHO_AM_I_REG {
            return Err(Error::WhoAmI(whoami));
        }

        // 2. Software reset (sets all registers to default)
        self.write_reg(CTRL3, CTRL3_SW_RESET).await?;
        // Poll until reset completes (bit self-clears)
        loop {
            let v = self.read_reg(CTRL3).await?;
            if v & CTRL3_SW_RESET == 0 {
                break;
            }
        }

        // 3. Enable register auto-increment + Block Data Update
        //    BDU prevents reading mismatched high/low bytes across a sample boundary.
        self.write_reg(CTRL3, CTRL3_IF_INC | CTRL3_BDU).await?;

        // 4. Select High-Accuracy ODR table 1
        //    (gives 1000 Hz when ODR nibble = 9, matching CTRL1_ODR_XL_1000HZ / CTRL2_ODR_G_1000HZ)
        self.write_reg(HAODR_CFG, HAODR_MODE1).await?;

        // 5. Accelerometer: ±16 g full scale + Strong LPF2 for vibration rejection
        self.write_reg(CTRL8, CTRL8_HP_LPF2_XL_BW_STRONG | CTRL8_FS_XL_16G)
            .await?;

        // 6. Gyroscope: ±2000 dps full scale + UltraLight LPF1 (lowest latency)
        self.write_reg(CTRL6, CTRL6_LPF1_G_BW_ULTRALIGHT | CTRL6_FS_G_2000DPS)
            .await?;

        // 7. Enable gyroscope LPF1 digital filter
        self.write_reg(CTRL7, CTRL7_LPF1_G_EN).await?;

        // 8. Enable accelerometer LPF2 digital filter
        self.write_reg(CTRL9, CTRL9_LPF2_XL_EN).await?;

        // 9. Accelerometer: High-Accuracy mode, 1000 Hz ODR
        self.write_reg(CTRL1, CTRL1_OP_MODE_XL_HIGH_ACCURACY | CTRL1_ODR_XL_1000HZ)
            .await?;

        // 10. Gyroscope: High-Accuracy mode, 1000 Hz ODR
        self.write_reg(CTRL2, CTRL2_OP_MODE_G_HIGH_ACCURACY | CTRL2_ODR_G_1000HZ)
            .await?;

        // 11. DRDY interrupt: pulsed (clears automatically, no register read needed to unlatch)
        self.write_reg(CTRL4, CTRL4_DRDY_PULSED).await?;

        // 12. Route gyro DRDY only to INT1 pin
        //     The interrupt fires when gyro data is ready; accel at the same ODR
        //     will be ready by the time we complete the SPI transaction.
        self.write_reg(INT1_CTRL, INT1_DRDY_G).await?;

        Ok(())
    }

    /// Read gyro + accel in a single 13-byte SPI transaction.
    ///
    /// Registers are contiguous: OUTX_L_G (0x22) … OUTZ_H_A (0x2D).
    /// That is 12 data bytes = 3×i16 gyro + 3×i16 accel.
    ///
    /// Call this after the INT1 rising edge has been detected.
    ///
    /// Raw integer values are converted using the datasheet scale factors:
    ///   - Gyro:  70 mdps/LSB  → degrees/s
    ///   - Accel: 0.488 mg/LSB → g
    pub async fn read(&mut self) -> Result<ImuData, Error<SPI::Error>> {
        // 1 address byte + 12 data bytes
        let mut buf = [0u8; 13];
        buf[0] = OUTX_L_G | SPI_READ;
        self.spi.transfer_in_place(&mut buf).await?;

        // buf[0]  = dummy (address echo / don't care)
        // buf[1..=2]   = OUTX_L_G / OUTX_H_G  → gyro X
        // buf[3..=4]   = OUTY_L_G / OUTY_H_G  → gyro Y
        // buf[5..=6]   = OUTZ_L_G / OUTZ_H_G  → gyro Z
        // buf[7..=8]   = OUTX_L_A / OUTX_H_A  → accel X
        // buf[9..=10]  = OUTY_L_A / OUTY_H_A  → accel Y
        // buf[11..=12] = OUTZ_L_A / OUTZ_H_A  → accel Z

        let gx = i16::from_le_bytes([buf[1], buf[2]]);
        let gy = i16::from_le_bytes([buf[3], buf[4]]);
        let gz = i16::from_le_bytes([buf[5], buf[6]]);
        let ax = i16::from_le_bytes([buf[7], buf[8]]);
        let ay = i16::from_le_bytes([buf[9], buf[10]]);
        let az = i16::from_le_bytes([buf[11], buf[12]]);

        Ok(ImuData {
            gyro: [
                gx as f32 * GYRO_SCALE_DPS,
                gy as f32 * GYRO_SCALE_DPS,
                gz as f32 * GYRO_SCALE_DPS,
            ],
            accel: [
                ax as f32 * ACCEL_SCALE_G,
                ay as f32 * ACCEL_SCALE_G,
                az as f32 * ACCEL_SCALE_G,
            ],
        })
    }

    async fn write_reg(
        &mut self,
        reg: u8,
        val: u8,
    ) -> Result<(), Error<SPI::Error>> {
        // MSB = 0 → write
        self.spi.write(&[reg & !SPI_READ, val]).await?;
        Ok(())
    }

    async fn read_reg(
        &mut self,
        reg: u8,
    ) -> Result<u8, Error<SPI::Error>> {
        let mut buf = [reg | SPI_READ, 0x00];
        self.spi.transfer_in_place(&mut buf).await?;
        Ok(buf[1])
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
