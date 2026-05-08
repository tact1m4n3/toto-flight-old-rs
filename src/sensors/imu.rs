use core::{
    convert::Infallible,
    f32,
    sync::atomic::{AtomicBool, Ordering},
};

use ahrs::Ahrs;
use embassy_rp::{gpio, peripherals::SPI1, spi};
use embassy_time::Delay;
use embedded_hal_bus::spi::{DeviceError, ExclusiveDevice};
use nalgebra::{UnitQuaternion, Vector3};

use crate::{
    ImuPeripherals, Irqs, SystemSpawner,
    drivers::lsm6dsv::{self, Lsm6dsv16b},
    nav_loop::ArmBlocker,
    persistent_config, signals,
    utils::{NalgebraQuatExt, NalgebraVec3Ext, SyncWatch},
};

type Device = ExclusiveDevice<spi::Spi<'static, SPI1, spi::Async>, gpio::Output<'static>, Delay>;
type Imu = Lsm6dsv16b<Device>;
type Error = lsm6dsv::Error<DeviceError<spi::Error, Infallible>>;

pub static GYRO: SyncWatch<Vector3<f32>> = SyncWatch::new(Vector3::ZERO);
pub static ACCEL: SyncWatch<Vector3<f32>> = SyncWatch::new(Vector3::ZERO);

static START_CALIBRATION: AtomicBool = AtomicBool::new(false);
static LEVEL: AtomicBool = AtomicBool::new(false);

signals! {
    INTERRUPT_SIGNAL: ();
}

pub fn start_calibration() {
    START_CALIBRATION.store(true, Ordering::Relaxed);
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

    spawner.realtime.spawn(main_task(imu).unwrap());
    spawner
        .realtime
        .spawn(interrupt_listener_task(interrupt_pin).unwrap());

    Ok(())
}

#[embassy_executor::task]
async fn main_task(mut imu: Imu) {
    let mut gyro_bias: Vector3<f32> = persistent_config::get_gyro_bias()
        .await
        .unwrap_or([0.0; 3])
        .into();
    let mut accel_bias: Vector3<f32> = persistent_config::get_accel_bias()
        .await
        .unwrap_or([0.0; 3])
        .into();

    loop {
        INTERRUPT_SIGNAL.wait().await;

        let should_start_calibration = START_CALIBRATION.load(Ordering::Relaxed);
        if should_start_calibration && let Ok(_arm_guard) = ArmBlocker::new() {
            match calibrate(&mut imu).await {
                Ok(biases) => (gyro_bias, accel_bias) = biases,
                Err(_) => defmt::error!("calibration failed"),
            }
            START_CALIBRATION.store(false, Ordering::Relaxed);
        };

        let data = match imu.read().await {
            Ok(data) => data,
            Err(_) => {
                defmt::error!("failed to read imu");
                continue;
            }
        };

        let unbiased_gyro = Vector3::from(data.gyro);
        let unbiased_accel = Vector3::from(data.accel);

        let gyro = (unbiased_gyro - gyro_bias) * f32::consts::PI / 180.0;
        let accel = (unbiased_accel - accel_bias) * 9.80665;

        GYRO.set(gyro);
        ACCEL.set(accel);
    }
}

async fn calibrate(imu: &mut Imu) -> Result<(Vector3<f32>, Vector3<f32>), Error> {
    const CALIBRATION_SAMPLES: usize = 500;

    defmt::debug!("calibration started");

    let mut calibrator = Calibrator::new();
    for _ in 0..CALIBRATION_SAMPLES {
        INTERRUPT_SIGNAL.wait().await;
        let data = imu.read().await?;
        calibrator.add_sample(data.gyro.into(), data.accel.into());
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

    persistent_config::set_gyro_bias(gyro_bias.into()).await;
    persistent_config::set_accel_bias(accel_bias.into()).await;

    Ok((gyro_bias, accel_bias))
}

#[embassy_executor::task]
async fn interrupt_listener_task(mut interrupt_pin: gpio::Input<'static>) {
    loop {
        interrupt_pin.wait_for_rising_edge().await;
        INTERRUPT_SIGNAL.signal(());
    }
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
        self.accumulated_accel += accel - Vector3::new(0.0, 0.0, 1.0);
        self.samples_collected += 1;
    }

    fn compute_bias(&self) -> (Vector3<f32>, Vector3<f32>) {
        let gyro_bias = self.accumulated_gyro / self.samples_collected as f32;
        let accel_bias = self.accumulated_accel / self.samples_collected as f32;
        (gyro_bias, accel_bias)
    }
}
