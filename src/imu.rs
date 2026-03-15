use core::cell::Cell;

use embassy_rp::{gpio, peripherals::SPI1, spi};
use embassy_sync::{
    blocking_mutex::{
        Mutex,
        raw::{CriticalSectionRawMutex, NoopRawMutex},
    },
    signal::Signal,
};
use embassy_time::{Delay, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use lsm6dsv16b_rs::asynchronous::prelude::*;
use nalgebra::{UnitQuaternion, Vector3};
use st_mems_bus::asynchronous::spi::SpiBus;
use static_cell::StaticCell;

use crate::{
    ImuPeripherals, Irqs, SystemSpawner, control::SystemAuthorityToken, sd_card, utils::{NalgebraQuatExt, NalgebraVec3Ext}
};

type Device = ExclusiveDevice<spi::Spi<'static, SPI1, spi::Async>, gpio::Output<'static>, Delay>;
type Bus = SpiBus<Device>;
type Imu = Lsm6dsv16b<Bus, Delay, lsm6dsv16b_rs::asynchronous::register::MainBank>;
type ImuError = Error<<Bus as st_mems_bus::asynchronous::BusOperation>::Error>;

const RATE: Rate = Rate::_4kHz;
const GRAVITY: Vector3<f32> = Vector3::new(0.0, 0.0, 9.81);

pub static RAW_GYRO: Mutex<CriticalSectionRawMutex, Cell<Vector3<f32>>> =
    Mutex::new(Cell::new(Vector3::ZERO));
pub static RAW_ACCEL: Mutex<CriticalSectionRawMutex, Cell<Vector3<f32>>> =
    Mutex::new(Cell::new(Vector3::ZERO));
pub static POSITION_DELTA: Mutex<CriticalSectionRawMutex, Cell<Vector3<f32>>> =
    Mutex::new(Cell::new(Vector3::ZERO));
pub static VELOCITY_DELTA: Mutex<CriticalSectionRawMutex, Cell<Vector3<f32>>> =
    Mutex::new(Cell::new(Vector3::ZERO));
pub static ORIENTATION_DELTA: Mutex<CriticalSectionRawMutex, Cell<UnitQuaternion<f32>>> =
    Mutex::new(Cell::new(UnitQuaternion::IDENTITY));

pub static CALIBRATION_REQUEST: Signal<CriticalSectionRawMutex, SystemAuthorityToken> =
    Signal::new();

static GYRO_BIAS: Mutex<CriticalSectionRawMutex, Cell<Vector3<f32>>> =
    Mutex::new(Cell::new(Vector3::ZERO));
static ACCEL_BIAS: Mutex<CriticalSectionRawMutex, Cell<Vector3<f32>>> =
    Mutex::new(Cell::new(Vector3::ZERO));

pub async fn init(spawner: SystemSpawner, p: ImuPeripherals) -> Result<(), ImuError> {
    let mut config = spi::Config::default();
    config.frequency = 8_000_000;
    let spi = spi::Spi::new(
        p.spi, p.clk_pin, p.mosi_pin, p.miso_pin, p.tx_dma, p.rx_dma, Irqs, config,
    );
    let spi_cs = gpio::Output::new(p.cs_pin, gpio::Level::High);
    let spi_dev = ExclusiveDevice::new(spi, spi_cs, Delay).unwrap(); // NOTE: can't fail
    let interrupt_pin = gpio::Input::new(p.int_pin, gpio::Pull::None);

    let mut imu = Lsm6dsv16b::new_spi(spi_dev, Delay);
    configure(&mut imu).await?;

    spawner.realtime.spawn(task(imu, interrupt_pin).unwrap());

    Ok(())
}

pub async fn calibrate(
    spawner: embassy_executor::Spawner,
) -> Result<(), CalibrationAlreadyInProgress> {
    static DONE_SIGNAL: StaticCell<Signal<NoopRawMutex, ()>> = StaticCell::new();
    let done_signal = DONE_SIGNAL.init(Signal::new());

    spawner.spawn(match calibrator(done_signal) {
        Ok(token) => token,
        Err(_) => return Err(CalibrationAlreadyInProgress),
    });

    done_signal.wait().await;

    Ok(())
}

#[derive(defmt::Format)]
pub struct CalibrationAlreadyInProgress;

#[embassy_executor::task]
async fn task(mut imu: Imu, mut interrupt_pin: gpio::Input<'static>) {
    loop {
        interrupt_pin.wait_for_rising_edge().await;

        let (unbiased_gyro, unbiased_accel) = match read(&mut imu).await {
            Ok(data) => data,
            Err(_) => {
                defmt::error!("failed to read imu");
                continue;
            }
        };

        let (prev_orientation, prev_velocity, prev_position, gyro_bias, accel_bias) =
            critical_section::with(|cs| {
                (
                    ORIENTATION_DELTA.borrow(cs).get(),
                    VELOCITY_DELTA.borrow(cs).get(),
                    POSITION_DELTA.borrow(cs).get(),
                    GYRO_BIAS.borrow(cs).get(),
                    ACCEL_BIAS.borrow(cs).get(),
                )
            });

        let gyro = unbiased_gyro - gyro_bias;
        let accel = unbiased_accel - accel_bias;

        let scaled_rotation = UnitQuaternion::from_scaled_axis(gyro * RATE.dt());
        let new_orientation = prev_orientation * scaled_rotation;
        let rot_matrix = new_orientation.to_rotation_matrix();
        let world_accel = rot_matrix.transform_vector(&accel) - GRAVITY;
        let new_position =
            prev_position + prev_velocity * RATE.dt() + 0.5 * world_accel * RATE.dt2();
        let new_velocity = prev_velocity + world_accel * RATE.dt();

        critical_section::with(|cs| {
            RAW_GYRO.borrow(cs).set(gyro);
            RAW_ACCEL.borrow(cs).set(accel);
            ORIENTATION_DELTA.borrow(cs).set(new_orientation);
            VELOCITY_DELTA.borrow(cs).set(new_velocity);
            POSITION_DELTA.borrow(cs).set(new_position);
        });
    }
}

#[embassy_executor::task]
async fn calibrator(done: &'static Signal<NoopRawMutex, ()>) {
    const CALIBRATION_SAMPLES: usize = 200;

    // TODO: Check we are not in flight

    defmt::debug!("calibration started");

    let mut accumulated_gyro: Vector3<f32> = Vector3::zeros();
    let mut accumulated_acceleration: Vector3<f32> = Vector3::zeros();
    for _ in 0..CALIBRATION_SAMPLES {
        let (gyro, accel) =
            critical_section::with(|cs| (RAW_GYRO.borrow(cs).get(), RAW_ACCEL.borrow(cs).get()));
        accumulated_gyro += gyro;
        accumulated_acceleration += accel;
        Timer::after_millis(1).await;
    }

    let gyro_bias = accumulated_gyro / CALIBRATION_SAMPLES as f32;
    let accel_bias = accumulated_acceleration / CALIBRATION_SAMPLES as f32 - GRAVITY;

    defmt::debug!("finished calibrating");
    defmt::debug!("gyro bias: {} {} {}", gyro_bias.x, gyro_bias.y, gyro_bias.z);
    defmt::debug!(
        "accel bias: {} {} {}",
        accel_bias.x,
        accel_bias.y,
        accel_bias.z
    );

    critical_section::with(|cs| {
        GYRO_BIAS.borrow(cs).set(gyro_bias);
        ACCEL_BIAS.borrow(cs).set(accel_bias);

        sd_card::config::access_mut(cs, |config| {
            config.calibration.gyro_bias = gyro_bias.into();
            config.calibration.accel_bias = accel_bias.into();
        });
    });

    done.signal(());
}

async fn configure(imu: &mut Imu) -> Result<(), ImuError> {
    // Check device ID
    let whoami = imu.device_id_get().await?;
    if whoami != ID {
        // TODO: maybe wrap error
        return Err(ImuError::UnexpectedValue);
    }

    // Restore default configuration
    imu.reset_set(Reset::RestoreCtrlRegs).await?;
    let mut rst: Reset = Reset::RestoreCtrlRegs;
    while rst != Reset::Ready {
        rst = imu.reset_get().await?;
        Timer::after_millis(10).await;
    }

    // Enable Block Data Update
    imu.block_data_update_set(1).await?;

    let mut pin_int = PinIntRoute::default();
    pin_int.drdy_g = 1;
    imu.pin_int1_route_set(&pin_int).await?;

    let odr = match RATE {
        Rate::_1kHz => Odr::Ha01_1000hz,
        Rate::_2kHz => Odr::Ha01_2000hz,
        Rate::_4kHz => Odr::Ha01_4000hz,
        Rate::_8kHz => Odr::Ha01_8000hz,
    };
    imu.gy_data_rate_set(odr).await?;
    imu.xl_data_rate_set(odr).await?;

    // Set full scale
    imu.xl_full_scale_set(XlFullScale::_16g).await?;
    imu.gy_full_scale_set(GyFullScale::_2000dps).await?;

    // Configure filtering chain
    let filt_settling_mask = FiltSettlingMask {
        drdy: 1,
        irq_xl: 1,
        irq_g: 1,
        tdm_excep_code: 0,
    };
    imu.filt_settling_mask_set(filt_settling_mask).await?;

    imu.filt_gy_lp1_set(1).await?;
    imu.filt_gy_lp1_bandwidth_set(FiltGyLp1Bandwidth::UltraLight)
        .await?;

    imu.filt_xl_lp2_set(1).await?;
    imu.filt_xl_hp_set(0).await?;
    imu.filt_xl_lp2_bandwidth_set(FiltXlLp2Bandwidth::Strong)
        .await?;

    Timer::after_millis(100).await;

    Ok(())
}

async fn read(imu: &mut Imu) -> Result<(Vector3<f32>, Vector3<f32>), ImuError> {
    // TODO: implement one read for both

    let gyro = imu.angular_rate_raw_get().await?;
    let gyro: Vector3<f32> = gyro.map(|value| from_fs2000_to_mdps(value) / 1000.0).into();

    let accel = imu.acceleration_raw_get().await?;
    let accel: Vector3<f32> = accel.map(|value| from_fs16_to_mg(value) / 1000.0).into();

    Ok((gyro, accel))
}

#[derive(Clone, Copy)]
pub enum Rate {
    _1kHz = 1000,
    _2kHz = 2000,
    _4kHz = 4000,
    _8kHz = 8000,
}

impl Rate {
    pub const fn dt(self) -> f32 {
        1.0 / (self as u16 as f32)
    }

    pub const fn dt2(self) -> f32 {
        self.dt() * self.dt()
    }
}
