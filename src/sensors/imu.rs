use core::cell::RefCell;

use embassy_sync::{blocking_mutex::Mutex, signal::Signal};
use nalgebra::{Matrix3, Rotation3, Vector3};

use crate::{
    RawMutex, config,
    control::ArmBlocker,
    drivers::imu::{Data as ImuSample, ImuSensor},
    utils::NalgebraVec3Ext,
};

static TAKE_SAMPLE_SIGNAL: Signal<RawMutex, ()> = Signal::new();

static CALIBRATE: Signal<RawMutex, ()> = Signal::new();
static IMU_RAW_SAMPLE: Signal<RawMutex, ImuSample> = Signal::new();

pub static DATA: Mutex<RawMutex, RefCell<Data>> = Mutex::new(RefCell::new(Data {
    gyro: Vector3::ZEROS,
    accel: Vector3::ZEROS,
}));

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

        let mut gyro = cfg.board_orientation * data.gyro - cfg.imu_calibration.gyro_bias;
        let mut accel = cfg.board_orientation
            * data.accel.component_mul(&cfg.imu_calibration.accel_scale)
            - cfg.imu_calibration.accel_bias;

        gyro *= core::f32::consts::PI / 180.0; // to rad/s from dps
        accel *= 9.80665; // to m/s^2 from g

        DATA.lock(|state| {
            let mut state = state.borrow_mut();
            state.gyro = gyro;
            state.accel = accel;
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
    pub gyro: Vector3<f32>,
    pub accel: Vector3<f32>,
}
