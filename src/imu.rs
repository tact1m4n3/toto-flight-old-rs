use embassy_sync::signal::Signal;
use nalgebra::{Matrix3, Rotation3, Vector3};

use crate::{
    RawMutex,
    config::{self, Config},
    control::ArmBlocker,
    drivers::imu::{Data, ImuSensor},
    utils::{NalgebraVec3Ext, SyncWatch},
};

pub static GYRO: SyncWatch<RawMutex, Vector3<f32>> = SyncWatch::new(Vector3::ZEROS);
pub static ACCEL: SyncWatch<RawMutex, Vector3<f32>> = SyncWatch::new(Vector3::ZEROS);

static TAKE_SAMPLE_SIGNAL: Signal<RawMutex, ()> = Signal::new();

static CALIBRATE: Signal<RawMutex, ()> = Signal::new();
static IMU_RAW_SAMPLE: Signal<RawMutex, Data> = Signal::new();

pub fn start_calibration() {
    CALIBRATE.signal(());
}

pub async fn task_interrupt_listener<P: embedded_hal_async::digital::Wait>(mut interrupt_pin: P) {
    loop {
        interrupt_pin.wait_for_rising_edge().await.ok();
        TAKE_SAMPLE_SIGNAL.signal(());
    }
}

pub async fn task_read<I: ImuSensor>(cfg: &'static Config, mut imu: I) {
    loop {
        TAKE_SAMPLE_SIGNAL.wait().await;

        let data = match imu.read().await {
            Ok(data) => data,
            Err(_) => {
                defmt::error!("failed to read imu");
                continue;
            }
        };

        IMU_RAW_SAMPLE.signal(data);

        let mut gyro = cfg.board_orientation * data.gyro - cfg.imu_calibration.gyro_bias;
        let mut accel = cfg.board_orientation
            * data.accel.component_mul(&cfg.imu_calibration.accel_scale)
            - cfg.imu_calibration.accel_bias;

        gyro *= core::f32::consts::PI / 180.0; // to rad/s from dps
        accel *= 9.80665; // to m/s^2 from g

        GYRO.set(gyro);
        ACCEL.set(accel);
    }
}

pub async fn task_calibrate(cfg: &'static Config) {
    loop {
        CALIBRATE.wait().await;
        if let Ok(_guard) = ArmBlocker::new() {
            easy_calibration(cfg).await;
        }
    }
}

// TODO: move orientation calibration someplace else and add accelerometer calibration
async fn easy_calibration(cfg: &'static Config) {
    const SAMPLE_COUNT: usize = 1000;

    let mut gyro_accum = Vector3::ZEROS;
    let mut accel_accum = Vector3::ZEROS;

    for _ in 0..SAMPLE_COUNT {
        let sample = IMU_RAW_SAMPLE.wait().await;
        gyro_accum += sample.gyro;
        accel_accum += sample.accel;
    }

    let gyro_mean = gyro_accum / SAMPLE_COUNT as f32;
    let accel_mean = accel_accum / SAMPLE_COUNT as f32;

    let board_forward_angle = cfg.board_forward_angle;

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
    let board_orientation = Matrix3::from_columns(&[north, east, down]);

    let gyro_bias = board_orientation * gyro_mean;

    config::UPDATE_CONFIG
        .send(config::ConfigItem::ImuCalibration(config::ImuCalibration {
            gyro_bias,
            ..cfg.imu_calibration
        }))
        .await;
    config::UPDATE_CONFIG
        .send(config::ConfigItem::BoardOrientation(board_orientation))
        .await;
}
