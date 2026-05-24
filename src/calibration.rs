use embassy_sync::signal::Signal;
use nalgebra::{Matrix3, Rotation3, Vector3};

use crate::{RawMutex, config, drivers, utils::NalgebraVec3Ext};

static IMU_AND_ORIENTATION_EASY_CALIBRATION_START: Signal<RawMutex, ()> = Signal::new();
static IMU_SAMPLE: Signal<RawMutex, drivers::imu::Data> = Signal::new();

pub fn start_imu_and_orientation_easy_calibration() {
    IMU_AND_ORIENTATION_EASY_CALIBRATION_START.signal(());
}

// TODO: better accelerometer calibration, currently we don't calculate any bias
// or scale for it

async fn imu_and_orientation_easy_calibration() {
}
