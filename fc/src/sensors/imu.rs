use embassy_sync::signal::Signal;
use nalgebra::{Matrix3, Rotation3, Vector3};
use uom::si::angle::radian;

use crate::{
    RawMutex, config,
    control::ArmBlocker,
    drivers::imu::{Data as ImuSample, ImuSensor},
    math::{Acceleration3D, AngularVelocity3D, Vector3Ext},
    utils::SyncWatch,
};

pub static DATA: SyncWatch<RawMutex, Data> = SyncWatch::new();

#[derive(Clone, Copy)]
pub struct Data {
    /// (Should be) Filtered gyro reading, FRD, in the body frame
    pub gyro: AngularVelocity3D,
    /// (Should be) Filtered accelerometer reading, FRD, in the body frame
    pub accel: Acceleration3D,
}

static TAKE_SAMPLE_SIGNAL: Signal<RawMutex, ()> = Signal::new();

static CALIBRATE: Signal<RawMutex, ()> = Signal::new();
static IMU_RAW_SAMPLE: Signal<RawMutex, ImuSample> = Signal::new();

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

        let ImuSample {
            mut gyro,
            mut accel,
        } = data;

        gyro -= cfg.imu_calibration.gyro_bias;
        gyro = gyro.transform(|v| cfg.imu_to_body_transform * v);

        accel -= cfg.imu_calibration.accel_bias;
        accel = accel.transform(|v| v.component_mul(&cfg.imu_calibration.accel_scale));
        accel = accel.transform(|v| cfg.imu_to_body_transform * v);

        DATA.set(Data { gyro, accel });
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
    const SAMPLE_COUNT: usize = 100;

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

    // In FRD, stationary accelerometer measures -Gravity (Up).
    // Negating it gives the actual Down vector.
    let down = -accel_mean.as_base_unit().normalize();

    // Default front reference is the sensor's X axis. If the board is pitched ~90 degrees
    // (nose pointing straight down or straight up), X is too close to the Down vector.
    // Fall back to -Z (Board Top) as the forward reference.
    let front_ref = if down.x.abs() < 0.9 {
        Vector3::x()
    } else {
        -Vector3::z()
    };

    // In FRD, Down x Front = East
    let unrotated_right = down.cross(&front_ref).normalize();

    // Apply the yaw offset around the Z (Down) axis
    let board_rotation = Rotation3::new(down * cfg.board_forward_angle.get::<radian>());
    let right = board_rotation * unrotated_right;

    // In FRD, Right x Down = Front
    let front = right.cross(&down).normalize();

    // Matrix3::from_columns([north, east, down]) creates a transform
    // FROM the Body Frame TO the Sensor Frame.
    let body_to_imu_transform = Matrix3::from_columns(&[front, right, down]);

    // Transpose the transform to get the inverse, which is FROM the Sensor Frame TO the Body Frame.
    let imu_to_body_transform = body_to_imu_transform.transpose();

    let gyro_bias = gyro_mean;

    config::UpdateItem::ImuCalibration(config::ImuCalibration {
        gyro_bias,
        ..cfg.imu_calibration
    })
    .commit()
    .await
    .ok();
    config::UpdateItem::ImuToBodyTransform(imu_to_body_transform)
        .commit()
        .await
        .ok();
    crate::REBOOT.signal(());
}
