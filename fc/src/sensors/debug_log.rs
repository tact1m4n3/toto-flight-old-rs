use embassy_time::{Duration, Ticker};
use uom::si::{
    acceleration::standard_gravity, angle::degree, angular_velocity::degree_per_second,
    electric_charge::milliampere_hour, electric_current::ampere, electric_potential::volt,
    length::meter, velocity::meter_per_second,
};

use super::{battery, fusion, gps, imu};

pub async fn task() {
    const PERIOD: Duration = Duration::from_hz(1);

    let mut ticker = Ticker::every(PERIOD);
    loop {
        ticker.next().await;

        let battery = battery::DATA.get();
        info!(
            "battery: voltage={=f32}V current={=f32}A used={=f32}mAh",
            battery.voltage.get::<volt>(),
            battery.current.get::<ampere>(),
            battery.capacity_used.get::<milliampere_hour>(),
        );

        let imu = imu::DATA.get();
        let gyro = imu.gyro.get::<degree_per_second>();
        let accel = imu.accel.get::<standard_gravity>();
        info!(
            "imu: gyro=({=f32}, {=f32}, {=f32})deg/s accel=({=f32}, {=f32}, {=f32})g",
            gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z,
        );

        let gps = gps::DATA.get();
        info!(
            "gps: fix={} sats={} hdop={=f32} lat={=f32} lon={=f32} alt={=f32}m spd={=f32}m/s heading={=f32}deg vspd={=f32}m/s",
            fix_type_name(gps.fix_type),
            gps.num_sat,
            gps.hdop,
            gps.latitude.get::<degree>() as f32,
            gps.longitude.get::<degree>() as f32,
            gps.altitude.get::<meter>(),
            gps.velocity_over_ground.get::<meter_per_second>(),
            gps.heading.get::<degree>(),
            gps.vertical_velocity.get::<meter_per_second>(),
        );

        let attitude = fusion::ATTITUDE.get();
        let (roll, pitch, yaw) = attitude.euler_angles();
        info!(
            "attitude: roll={=f32}deg pitch={=f32}deg yaw={=f32}deg",
            roll.to_degrees(),
            pitch.to_degrees(),
            yaw.to_degrees(),
        );
    }
}

fn fix_type_name(fix_type: gps::FixType) -> &'static str {
    match fix_type {
        gps::FixType::NoFix => "NoFix",
        gps::FixType::DeadReckoningOnly => "DeadReckoningOnly",
        gps::FixType::Fix2D => "Fix2D",
        gps::FixType::Fix3D => "Fix3D",
        gps::FixType::GPSPlusDeadReckoning => "GPSPlusDeadReckoning",
        gps::FixType::TimeOnlyFix => "TimeOnlyFix",
        gps::FixType::Undefined => "Undefined",
    }
}
