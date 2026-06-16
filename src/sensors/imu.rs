use embassy_sync::signal::Signal;
use nalgebra::{Matrix3, Rotation3, Vector3};
use uom::si::{acceleration::standard_gravity, angle::radian};

use crate::{
    RawMutex, config,
    control::ArmBlocker,
    drivers::imu::{Data as ImuSample, ImuSensor},
    math::{Acceleration3D, AngularVelocity3D, Vector3Ext},
    utils::SyncWatch,
};

static TAKE_SAMPLE_SIGNAL: Signal<RawMutex, ()> = Signal::new();

static CALIBRATE: Signal<RawMutex, ()> = Signal::new();
static IMU_RAW_SAMPLE: Signal<RawMutex, ImuSample> = Signal::new();

pub static DATA: SyncWatch<RawMutex, Data> = SyncWatch::new(Data {
    gyro: AngularVelocity3D::ZEROS,
    accel: Acceleration3D::ZEROS,
});

pub fn start_calibration() {
    CALIBRATE.signal(());
}

pub async fn task_interrupt_listener<P: embedded_hal_async::digital::Wait>(mut interrupt_pin: P) {
    loop {
        interrupt_pin.wait_for_rising_edge().await.ok();
        TAKE_SAMPLE_SIGNAL.signal(());
    }
}

pub async fn task_read<I: ImuSensor>(mut imu: I) {
    let cfg = config::INITIAL.get().await;

    loop {
        TAKE_SAMPLE_SIGNAL.wait().await;

        let Ok(data) = imu.read().await else {
            warn!("failed to read imu");
            continue;
        };

        IMU_RAW_SAMPLE.signal(data);

        DATA.set(Data {
            gyro: AngularVelocity3D::from_base_unit(
                cfg.board_orientation * data.gyro.as_base_unit(),
            ) - cfg.imu_calibration.gyro_bias,
            accel: Acceleration3D::from_base_unit(
                cfg.board_orientation
                    * data
                        .accel
                        .as_base_unit()
                        .component_mul(&cfg.imu_calibration.accel_scale),
            ) - cfg.imu_calibration.accel_bias,
        });
    }
}

pub async fn task_calibrate() {
    loop {
        CALIBRATE.wait().await;
        if let Ok(_guard) = ArmBlocker::new() {
            easy_calibration().await;
        } else {
            warn!("ignoring imu calibration request because drone is armed");
        }
    }
}

// TODO: move orientation calibration someplace else and add accelerometer calibration
async fn easy_calibration() {
    const SAMPLE_COUNT: usize = 1000;

    let cfg = config::INITIAL.get().await;

    let mut gyro_accum: AngularVelocity3D = AngularVelocity3D::ZEROS;
    let mut accel_accum: Acceleration3D = Acceleration3D::ZEROS;

    for _ in 0..SAMPLE_COUNT {
        let sample = IMU_RAW_SAMPLE.wait().await;
        gyro_accum += sample.gyro;
        accel_accum += sample.accel;
    }

    let gyro_mean = gyro_accum / SAMPLE_COUNT as f32;
    let accel_mean = accel_accum / SAMPLE_COUNT as f32;

    let down = accel_mean.get::<standard_gravity>().normalize();

    let reference = if down.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };

    let board_rotation = Rotation3::new(down * cfg.board_forward_angle.get::<radian>());

    let east = board_rotation * down.cross(&reference).normalize();
    let north = east.cross(&down).normalize();
    let board_orientation = Matrix3::from_columns(&[north, east, down]);

    let gyro_bias = AngularVelocity3D::from_base_unit(board_orientation * gyro_mean.as_base_unit());

    config::UpdateItem::ImuCalibration(config::ImuCalibration {
        gyro_bias,
        ..cfg.imu_calibration
    })
    .commit()
    .await
    .ok();
    config::UpdateItem::BoardOrientation(board_orientation)
        .commit()
        .await
        .ok();
}

#[derive(Clone, Copy)]
pub struct Data {
    /// (Should be) Filtered gyro reading, NED, in the board's frame
    pub gyro: AngularVelocity3D,
    /// (Should be) Filtered accelerometer reading, NED, in the board's frame
    pub accel: Acceleration3D,
}
