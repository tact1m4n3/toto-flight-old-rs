use core::convert::Infallible;

use embassy_rp::{gpio, peripherals::SPI1, spi};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::{Delay, Instant, Timer};
use embedded_hal_bus::spi::{DeviceError, ExclusiveDevice};
use nalgebra::Vector3;
use static_cell::StaticCell;

use crate::{
    ImuPeripherals, Irqs, SystemSpawner, black_box,
    drivers::lsm6dsv::{self, Lsm6dsv16b},
    mutex_cells, persistent_config,
    utils::NalgebraVec3Ext,
};

type Device = ExclusiveDevice<spi::Spi<'static, SPI1, spi::Async>, gpio::Output<'static>, Delay>;
type Imu = Lsm6dsv16b<Device>;
type Error = lsm6dsv::Error<DeviceError<spi::Error, Infallible>>;

const GRAVITY: Vector3<f32> = Vector3::new(0.0, 0.0, 9.81);

mutex_cells! {
    GYRO: Vector3<f32> = Vector3::ZERO;
    ACCEL: Vector3<f32> = Vector3::ZERO;

    GYRO_BIAS: Vector3<f32> = Vector3::ZERO;
    ACCEL_BIAS: Vector3<f32> = Vector3::ZERO;
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

    let mut imu = Lsm6dsv16b::new(spi_dev);
    imu.init().await?;

    spawner
        .realtime
        .spawn(read_task(imu, interrupt_pin).unwrap());

    Ok(())
}

pub fn calibrate(spawner: embassy_executor::Spawner) -> Result<(), CalibrationAlreadyInProgress> {
    spawner.spawn(calibrator().map_err(|_| CalibrationAlreadyInProgress)?);
    Ok(())
}

#[derive(defmt::Format)]
pub struct CalibrationAlreadyInProgress;

#[embassy_executor::task]
async fn read_task(mut imu: Imu, interrupt_pin: gpio::Input<'static>) {
    // load calibration
    let gyro_bias = persistent_config::get_gyro_bias().await.unwrap_or([0.0; 3]);
    let accel_bias = persistent_config::get_accel_bias()
        .await
        .unwrap_or([0.0; 3]);
    critical_section::with(|cs| {
        GYRO_BIAS.borrow(cs).set(gyro_bias.into());
        ACCEL_BIAS.borrow(cs).set(accel_bias.into());
    });

    let spawner = unsafe { embassy_executor::Spawner::for_current_executor().await };
    static INTERRUPT_SIGNAL: StaticCell<Signal<NoopRawMutex, ()>> = StaticCell::new();
    let interrupt_signal = INTERRUPT_SIGNAL.init(Signal::new());
    spawner.spawn(interrupt_task(interrupt_pin, interrupt_signal).unwrap());

    loop {
        interrupt_signal.wait().await;

        let data = match imu.read().await {
            Ok(data) => data,
            Err(_) => {
                defmt::error!("failed to read imu");
                continue;
            }
        };

        let (gyro_bias, accel_bias) =
            critical_section::with(|cs| (GYRO_BIAS.borrow(cs).get(), ACCEL_BIAS.borrow(cs).get()));

        let gyro = Vector3::from(data.gyro) + gyro_bias;
        let accel = Vector3::from(data.accel) + accel_bias;

        critical_section::with(|cs| {
            GYRO.borrow(cs).set(gyro);
            ACCEL.borrow(cs).set(accel);
        });

        let timestamp = Instant::now();
        black_box::GYRO.signal((gyro, timestamp));
        black_box::ACCEL.signal((accel, timestamp));
    }
}

#[embassy_executor::task]
async fn interrupt_task(
    mut interrupt_pin: gpio::Input<'static>,
    interrupt_signal: &'static Signal<NoopRawMutex, ()>,
) {
    loop {
        interrupt_pin.wait_for_rising_edge().await;
        interrupt_signal.signal(());
    }
}

#[embassy_executor::task]
async fn calibrator() {
    const CALIBRATION_SAMPLES: usize = 1000;

    // TODO: acquire arm lock

    defmt::debug!("calibration started");

    let mut accumulated_gyro: Vector3<f32> = Vector3::zeros();
    let mut accumulated_acceleration: Vector3<f32> = Vector3::zeros();
    for _ in 0..CALIBRATION_SAMPLES {
        let (gyro, accel) =
            critical_section::with(|cs| (GYRO.borrow(cs).get(), ACCEL.borrow(cs).get()));
        accumulated_gyro += gyro;
        accumulated_acceleration += accel;
        Timer::after_millis(1).await;
    }

    let gyro_bias = -accumulated_gyro / CALIBRATION_SAMPLES as f32;
    let accel_bias = -GRAVITY - accumulated_acceleration / CALIBRATION_SAMPLES as f32;

    defmt::debug!("finished calibrating");
    defmt::debug!("gyro bias: {} {} {}", gyro_bias.x, gyro_bias.y, gyro_bias.z);
    defmt::debug!(
        "accel bias: {} {} {}",
        accel_bias.x,
        accel_bias.y,
        accel_bias.z
    );

    critical_section::with(|cs| {
        // we add the new bias to the old one because the new bias is calculated based on the
        // already biased readings
        GYRO_BIAS
            .borrow(cs)
            .set(GYRO_BIAS.borrow(cs).get() + gyro_bias);
        ACCEL_BIAS
            .borrow(cs)
            .set(ACCEL_BIAS.borrow(cs).get() + accel_bias);
    });

    persistent_config::set_gyro_bias(gyro_bias.into()).await;
    persistent_config::set_accel_bias(accel_bias.into()).await;
}
