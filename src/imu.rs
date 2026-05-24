use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::signal::Signal;
use nalgebra::{Matrix3, Rotation3, Vector3};

use crate::{
    RawMutex, config,
    drivers::imu::{Data, ImuSensor},
    utils::{NalgebraVec3Ext, SyncWatch},
};

const CALIBRATION_SAMPLES: usize = 1000;

pub static GYRO: SyncWatch<RawMutex, Vector3<f32>> = SyncWatch::new(Vector3::ZEROS);
pub static ACCEL: SyncWatch<RawMutex, Vector3<f32>> = SyncWatch::new(Vector3::ZEROS);

static START_CALIBRATION: AtomicBool = AtomicBool::new(false);
static TAKE_SAMPLE_SIGNAL: Signal<RawMutex, ()> = Signal::new();

pub fn start_calibration() {
    START_CALIBRATION.store(true, Ordering::Relaxed);
}

pub async fn task_interrupt_listener<P: embedded_hal_async::digital::Wait>(mut interrupt_pin: P) {
    loop {
        interrupt_pin.wait_for_rising_edge().await.ok();
        TAKE_SAMPLE_SIGNAL.signal(());
    }
}

pub async fn task_read<I: ImuSensor>(mut imu: I) {
    let mut calibration = config::IMU_CALIBRATION.get();
    let mut orientation = config::BOARD_ORIENTATION.get();

    let mut calibrator: Option<Calibrator> = None;

    loop {
        TAKE_SAMPLE_SIGNAL.wait().await;

        let data = match imu.read().await {
            Ok(data) => data,
            Err(_) => {
                defmt::error!("failed to read imu");
                continue;
            }
        };

        if START_CALIBRATION.swap(false, Ordering::Relaxed) && calibrator.is_none() {
            calibrator = Some(Calibrator::new());
        }

        if let Some(mut calibrator) = calibrator.take() {
            calibrator.add_sample(&data);
            if calibrator.sample_count >= CALIBRATION_SAMPLES {

            }
        }

        let gyro = orientation * data.gyro - calibration.gyro_bias;
        let accel = orientation * data.accel.component_mul(&calibration.accel_scale)
            - calibration.accel_bias;

        let gyro = gyro * core::f32::consts::PI / 180.0; // to rad/s from dps
        let accel = accel * 9.80665; // to m/s^2 from g

        GYRO.set(gyro);
        ACCEL.set(accel);
    }
}

struct Calibrator {
    sample_count: usize,
    gyro_accum: Vector3<f32>,
    accel_accum: Vector3<f32>,
}

impl Calibrator {
    pub fn new() -> Self {
        Self {
            sample_count: 0,
            gyro_accum: Vector3::ZEROS,
            accel_accum: Vector3::ZEROS,
        }
    }

    pub fn add_sample(&mut self, sample: &Data) {
        self.gyro_accum += sample.gyro;
        self.accel_accum += sample.accel;
        self.sample_count += 1;
    }
}

async fn calibrate<I: ImuSensor>(
    imu: &mut I,
    calibration: &mut config::ImuCalibration,
    orientation: &mut Matrix3<f32>,
) {
    const CALIBRATION_SAMPLE_WEIGHT: f32 = 1.0 / CALIBRATION_SAMPLES as f32;

    defmt::info!("calibration started");

    let mut gyro_mean = Vector3::zeros();
    let mut accel_mean = Vector3::zeros();

    for _ in 0..CALIBRATION_SAMPLES {
        let data = IMU_SAMPLE.wait().await;
        gyro_mean += Vector3::from(data.gyro) * CALIBRATION_SAMPLE_WEIGHT;
        accel_mean += Vector3::from(data.accel) * CALIBRATION_SAMPLE_WEIGHT;
    }

    let board_forward_angle = config::BOARD_FORWARD_ANGLE.get();

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

    defmt::info!("imu and orientation easy completed");
    defmt::info!(
        "gyro bias: x: {}, y: {}, z: {}",
        gyro_bias.x,
        gyro_bias.y,
        gyro_bias.z
    );
    let (roll, pitch, yaw) = Rotation3::from_matrix(&board_orientation).euler_angles();
    defmt::info!(
        "orientation: roll: {}d, pitch: {}d, yaw: {}d",
        roll.to_degrees(),
        pitch.to_degrees(),
        yaw.to_degrees()
    );

    config::IMU_CALIBRATION.set(config::ImuCalibration {
        gyro_bias,
        accel_bias: Vector3::ZEROS,
        accel_scale: Vector3::ONES,
    });
    config::BOARD_ORIENTATION.set(board_orientation);
}
