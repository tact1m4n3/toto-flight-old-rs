use core::{
    convert::Infallible,
    f32,
    sync::atomic::{AtomicBool, Ordering},
};

use embassy_rp::{gpio, peripherals::SPI1, spi};
use embassy_time::Delay;
use embedded_hal_bus::spi::{DeviceError, ExclusiveDevice};
use nalgebra::{Matrix3, Rotation3, Vector3};

use crate::{
    ImuPeripherals, Irqs, SystemSpawner,
    drivers::lsm6dsv::{self, Lsm6dsv16b},
    nav_loop::ArmBlocker,
    persistent_config, signals,
    utils::{NalgebraVec3Ext, SyncWatch},
};

type Device = ExclusiveDevice<spi::Spi<'static, SPI1, spi::Async>, gpio::Output<'static>, Delay>;
type Imu = Lsm6dsv16b<Device>;
type Error = lsm6dsv::Error<DeviceError<spi::Error, Infallible>>;

pub static GYRO: SyncWatch<Vector3<f32>> = SyncWatch::new(Vector3::ZERO);
pub static ACCEL: SyncWatch<Vector3<f32>> = SyncWatch::new(Vector3::ZERO);

static START_CALIBRATION: AtomicBool = AtomicBool::new(false);

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
    let mut imu_rot: Matrix3<f32> = match persistent_config::get_imu_orientation().await {
        Some(raw) => Matrix3::from_column_slice(&raw),
        None => Matrix3::identity(),
    };

    loop {
        INTERRUPT_SIGNAL.wait().await;

        let data = match imu.read().await {
            Ok(data) => data,
            Err(_) => {
                defmt::error!("failed to read imu");
                continue;
            }
        };

        let gyro = imu_rot * Vector3::from(data.gyro) - gyro_bias;
        let accel = imu_rot * Vector3::from(data.accel) - accel_bias;

        let gyro = gyro * f32::consts::PI / 180.0; // to rad/s from dps
        let accel = accel * 9.80665; // to m/s^2 from g

        critical_section::with(|cs| {
            GYRO.set(cs, gyro);
            ACCEL.set(cs, accel);
        });

        let should_start_calibration = START_CALIBRATION.load(Ordering::Relaxed);
        if should_start_calibration && let Ok(_arm_guard) = ArmBlocker::new() {
            match calibrate(&mut imu).await {
                Ok((new_gyro_bias, new_accel_bias, new_imu_rot)) => {
                    gyro_bias = new_gyro_bias;
                    accel_bias = new_accel_bias;
                    imu_rot = new_imu_rot;
                }
                Err(_) => defmt::error!("calibration failed"),
            }
            START_CALIBRATION.store(false, Ordering::Relaxed);
        };
    }
}

// TODO: better accelerometer calibration, currently we don't calculate any bias
// or scale for it
async fn calibrate(imu: &mut Imu) -> Result<(Vector3<f32>, Vector3<f32>, Matrix3<f32>), Error> {
    const CALIBRATION_SAMPLES: usize = 500;
    const CALIBRATION_SAMPLE_WEIGHT: f32 = 1.0 / CALIBRATION_SAMPLES as f32;

    defmt::info!("calibration started");

    let mut gyro_mean = Vector3::zeros();
    let mut accel_mean = Vector3::zeros();

    for _ in 0..CALIBRATION_SAMPLES {
        INTERRUPT_SIGNAL.wait().await;
        let data = imu.read().await?;
        gyro_mean += Vector3::from(data.gyro) * CALIBRATION_SAMPLE_WEIGHT;
        accel_mean += Vector3::from(data.accel) * CALIBRATION_SAMPLE_WEIGHT;
    }

    let board_forward_angle = persistent_config::get_board_forward_angle()
        .await
        .unwrap_or(0.0)
        .to_radians();

    let up = accel_mean.normalize();
    let down = -up;

    let reference = if down.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };

    let board_rotation = Rotation3::new(down * board_forward_angle);

    let east = board_rotation * down.cross(&reference).normalize();
    let north = east.cross(&down).normalize();
    let rot = Matrix3::from_columns(&[north, east, down]);

    let gyro_bias = rot * gyro_mean;
    // In NED the accelerometer reads -1g in the z direction.
    let accel_bias = rot * accel_mean + Vector3::z();

    defmt::info!("calibration completed");
    defmt::info!("gyro bias: {} {} {}", gyro_bias.x, gyro_bias.y, gyro_bias.z);
    defmt::info!(
        "accel bias: {} {} {}",
        accel_bias.x,
        accel_bias.y,
        accel_bias.z
    );

    persistent_config::set_gyro_bias(gyro_bias.into()).await;
    persistent_config::set_accel_bias(accel_bias.into()).await;
    persistent_config::set_imu_orientation(*rot.as_slice().as_array::<9>().unwrap()).await;

    Ok((gyro_bias, accel_bias, rot))
}

#[embassy_executor::task]
async fn interrupt_listener_task(mut interrupt_pin: gpio::Input<'static>) {
    loop {
        interrupt_pin.wait_for_rising_edge().await;
        INTERRUPT_SIGNAL.signal(());
    }
}
