use core::{
    convert::Infallible,
    sync::atomic::{AtomicBool, Ordering},
};

use embassy_rp::{gpio, peripherals::SPI1, spi};
use embassy_time::Delay;
use embedded_hal_bus::spi::{DeviceError, ExclusiveDevice};
use nalgebra::Vector3;

use crate::{
    ImuPeripherals, Irqs, SystemSpawner, drivers::lsm6dsv::{self, Lsm6dsv16b}, mutex_cells, nav_loop::ArmBlocker, persistent_config, signals, utils::{NalgebraVec3Ext, SyncWatch}
};

type Device = ExclusiveDevice<spi::Spi<'static, SPI1, spi::Async>, gpio::Output<'static>, Delay>;
type Imu = Lsm6dsv16b<Device>;
type Error = lsm6dsv::Error<DeviceError<spi::Error, Infallible>>;

const GRAVITY: Vector3<f32> = Vector3::new(0.0, 0.0, 9.81);

pub static GYRO: SyncWatch<Vector3<f32>> = SyncWatch::new(Vector3::ZERO);
pub static ACCEL: SyncWatch<Vector3<f32>> = SyncWatch::new(Vector3::ZERO);

mutex_cells! {
    GYRO_BIAS: Vector3<f32> = Vector3::ZERO;
    ACCEL_BIAS: Vector3<f32> = Vector3::ZERO;
}

pub fn get_gyro_bias() -> Vector3<f32> {
    GYRO_BIAS.lock(|cell| cell.get())
}

pub fn get_accel_bias() -> Vector3<f32> {
    ACCEL_BIAS.lock(|cell| cell.get())
}

signals! {
    INTERRUPT_SIGNAL: ();
    UNBIASED_IMU_FOR_CALIBRATION: (Vector3<f32>, Vector3<f32>);
}

pub async fn init(spawner: SystemSpawner, p: ImuPeripherals) -> Result<(), Error> {
    let mut config = spi::Config::default();
    config.frequency = 8_000_000;
    let spi = spi::Spi::new(
        p.spi, p.clk_pin, p.mosi_pin, p.miso_pin, p.tx_dma, p.rx_dma, Irqs, config,
    );
    let spi_cs = gpio::Output::new(p.cs_pin, gpio::Level::High);
    let spi_dev = ExclusiveDevice::new(spi, spi_cs, Delay).unwrap(); // NOTE: can't fail
    let interrupt_pin = gpio::Input::new(p.int_pin, gpio::Pull::None);

    let imu = Lsm6dsv16b::new(spi_dev).await?;

    // load calibration
    // NOTE: there is a possible race condition here with the calibrate function
    let gyro_bias = persistent_config::get_gyro_bias().await.unwrap_or([0.0; 3]);
    let accel_bias = persistent_config::get_accel_bias()
        .await
        .unwrap_or([0.0; 3]);
    critical_section::with(|cs| {
        GYRO_BIAS.borrow(cs).set(gyro_bias.into());
        ACCEL_BIAS.borrow(cs).set(accel_bias.into());
    });

    spawner.realtime.spawn(main_task(imu).unwrap());
    spawner
        .realtime
        .spawn(interrupt_listener_task(interrupt_pin).unwrap());

    Ok(())
}

#[embassy_executor::task]
async fn main_task(mut imu: Imu) {
    loop {
        INTERRUPT_SIGNAL.wait().await;

        let data = match imu.read().await {
            Ok(data) => data,
            Err(_) => {
                defmt::error!("failed to read imu");
                continue;
            }
        };

        let unbiased_gyro = Vector3::from(data.gyro);
        let unbiased_accel = Vector3::from(data.accel);
        UNBIASED_IMU_FOR_CALIBRATION.signal((unbiased_gyro, unbiased_accel));

        let (gyro_bias, accel_bias) =
            critical_section::with(|cs| (GYRO_BIAS.borrow(cs).get(), ACCEL_BIAS.borrow(cs).get()));

        let gyro = unbiased_gyro + gyro_bias;
        let accel = unbiased_accel + accel_bias;

        GYRO.set(gyro);
        ACCEL.set(accel);
    }
}

#[embassy_executor::task]
async fn interrupt_listener_task(mut interrupt_pin: gpio::Input<'static>) {
    loop {
        interrupt_pin.wait_for_rising_edge().await;
        INTERRUPT_SIGNAL.signal(());
    }
}

pub struct CalibrationAlreadyInProgressError;

pub async fn calibrate() -> Result<(), CalibrationAlreadyInProgressError> {
    const CALIBRATION_SAMPLES: usize = 1000;

    static CALIBRATING: AtomicBool = AtomicBool::new(false);
    if CALIBRATING.swap(true, Ordering::Relaxed) {
        return Err(CalibrationAlreadyInProgressError);
    }

    let _blocker = ArmBlocker::new();

    defmt::debug!("calibration started");

    let mut calibrator = Calibrator::new();
    for _ in 0..CALIBRATION_SAMPLES {
        let (gyro, accel) = UNBIASED_IMU_FOR_CALIBRATION.wait().await;
        calibrator.add_sample(gyro, accel);
    }
    let (gyro_bias, accel_bias) = calibrator.compute_bias();

    defmt::debug!("calibration completed");
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
    });
    persistent_config::set_gyro_bias(gyro_bias.into()).await;
    persistent_config::set_accel_bias(accel_bias.into()).await;

    CALIBRATING.store(false, Ordering::Release);
    Ok(())
}

struct Calibrator {
    accumulated_gyro: Vector3<f32>,
    accumulated_accel: Vector3<f32>,
    samples_collected: usize,
}

impl Calibrator {
    fn new() -> Self {
        Self {
            accumulated_gyro: Vector3::zeros(),
            accumulated_accel: Vector3::zeros(),
            samples_collected: 0,
        }
    }

    fn add_sample(&mut self, gyro: Vector3<f32>, accel: Vector3<f32>) {
        self.accumulated_gyro += gyro;
        self.accumulated_accel += accel;
        self.samples_collected += 1;
    }

    fn compute_bias(&self) -> (Vector3<f32>, Vector3<f32>) {
        let gyro_bias = -self.accumulated_gyro / self.samples_collected as f32;
        let accel_bias = -GRAVITY - self.accumulated_accel / self.samples_collected as f32;
        (gyro_bias, accel_bias)
    }
}
