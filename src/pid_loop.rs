use core::ops::RangeInclusive;

use embassy_rp::pwm::{self, PwmOutput, SetDutyCycle};
use embassy_time::{Duration, Ticker};
use fixed::FixedU16;
use nalgebra::Vector3;

use crate::{
    PwmOutputsPeripherals, SystemSpawner,
    nav_loop::{self, Command},
    persistent_config,
    sensors::imu,
    utils::{NalgebraVec3Ext, SyncWatch},
};

pub static MOTOR_OUTPUT: SyncWatch<MotorOutput> = SyncWatch::new(MotorOutput {
    throttle: 1000,
    left_elevon: 1500,
    right_elevon: 1500,
});

#[derive(Clone, Copy)]
pub struct MotorOutput {
    pub throttle: u16,
    pub left_elevon: u16,
    pub right_elevon: u16,
}

impl MotorOutput {
    fn apply(self, output_pwms: &mut OutputPwms) {
        output_pwms.throttle.set_duty_cycle(self.throttle).ok();
        output_pwms
            .left_elevon
            .set_duty_cycle(self.left_elevon)
            .ok();
        output_pwms
            .right_elevon
            .set_duty_cycle(self.right_elevon)
            .ok();
    }
}

pub fn init(spawner: SystemSpawner, p: PwmOutputsPeripherals) {
    // Make pwm counter increment once every microsecond
    let pwm_divider = embassy_rp::clocks::clk_sys_freq() / 1_000_000;
    // The frequency of the pwm signal is 50Hz.
    let pwm_top = 20_000;

    let mut config = pwm::Config::default();
    config.top = pwm_top;
    config.divider = FixedU16::from_num(pwm_divider);

    let (Some(throttle_pwm), Some(left_elevon_pwm)) = pwm::Pwm::new_output_ab(
        p.slice_esc_and_left_elevon,
        p.esc,
        p.left_elevon,
        config.clone(),
    )
    .split() else {
        unreachable!()
    };

    let (None, Some(right_elevon_pwm)) =
        pwm::Pwm::new_output_b(p.slice_right_elevon, p.right_elevon, config.clone()).split()
    else {
        unreachable!()
    };

    spawner.realtime.spawn(
        task(OutputPwms {
            throttle: throttle_pwm,
            left_elevon: left_elevon_pwm,
            right_elevon: right_elevon_pwm,
        })
        .unwrap(),
    );
}

const FREQ_HZ: u64 = 500;
const FREQ_DT: f32 = 1.0 / FREQ_HZ as f32;

struct OutputPwms {
    throttle: PwmOutput<'static>,
    left_elevon: PwmOutput<'static>,
    right_elevon: PwmOutput<'static>,
}

#[embassy_executor::task]
async fn task(mut output_pwms: OutputPwms) {
    let mixer_settings = MixerSettings {
        left_elevon: OutputMixerSettings {
            mid_point: 1500,
            range: 1000..=2000,
            gain: Vector3::new(0.5, 0.5, 0.0),
        },
        right_elevon: OutputMixerSettings {
            mid_point: 1500,
            range: 1000..=2000,
            gain: Vector3::new(-0.5, 0.5, 0.0),
        },
    };

    let mut rate_controller = RateController {
        roll_controller: SingleAxisRateController::new(RatePidSettings {
            kp: persistent_config::get_roll_rate_kp().await.unwrap_or(7.0),
            ki: persistent_config::get_roll_rate_ki().await.unwrap_or(10.0),
            kff: persistent_config::get_roll_rate_kff().await.unwrap_or(50.0),
            i_term_max_error: 165.0,
        }),
        pitch_controller: SingleAxisRateController::new(RatePidSettings {
            kp: persistent_config::get_pitch_rate_kp().await.unwrap_or(7.0),
            ki: persistent_config::get_pitch_rate_ki().await.unwrap_or(10.0),
            kff: persistent_config::get_pitch_rate_kff()
                .await
                .unwrap_or(90.0),
            i_term_max_error: 165.0,
        }),
        yaw_controller: SingleAxisRateController::new(RatePidSettings {
            kp: persistent_config::get_yaw_rate_kp().await.unwrap_or(0.0),
            ki: persistent_config::get_yaw_rate_ki().await.unwrap_or(0.0),
            kff: persistent_config::get_yaw_rate_kff().await.unwrap_or(0.0),
            i_term_max_error: 165.0,
        }),
    };

    let mut ticker = Ticker::every(Duration::from_hz(FREQ_HZ));

    loop {
        ticker.next().await;

        let gyro = imu::GYRO.get();
        let command = nav_loop::COMMAND.get();

        match command {
            // TODO: should we set idle to a fixed value, or keep things this way?
            Command::Idle => ManualControl::IDLE
                .mix(&mixer_settings)
                .apply(&mut output_pwms),

            Command::Manual { throttle, throw } => ManualControl { throttle, throw }
                .mix(&mixer_settings)
                .apply(&mut output_pwms),

            Command::Rate { throttle, rate } => RateControl { throttle, rate }
                .pid(&mut rate_controller, gyro, FREQ_DT)
                .mix(&mixer_settings)
                .apply(&mut output_pwms),
        }
    }
}

struct RateControl {
    throttle: u16,
    rate: Vector3<f32>,
}

impl RateControl {
    fn pid(
        self,
        rate_controller: &mut RateController,
        gyro: Vector3<f32>,
        dt: f32,
    ) -> ManualControl {
        let Self { throttle, rate } = self;
        ManualControl {
            throttle,
            throw: rate_controller.update(rate, gyro, dt),
        }
    }
}

struct ManualControl {
    throttle: u16,
    throw: Vector3<f32>,
}

impl ManualControl {
    const IDLE: Self = Self {
        throttle: 1000,
        throw: Vector3::ZERO,
    };

    fn mix(self, mixer_settings: &MixerSettings) -> MotorOutput {
        let Self { throttle, throw } = self;

        let left_elevon = mixer_settings.left_elevon.mid_point
            + throw.dot(&mixer_settings.left_elevon.gain) as i16;
        let right_elevon = mixer_settings.right_elevon.mid_point
            + throw.dot(&mixer_settings.right_elevon.gain) as i16;

        let left_elevon = left_elevon.clamp(
            *mixer_settings.left_elevon.range.start(),
            *mixer_settings.left_elevon.range.end(),
        ) as u16;

        let right_elevon = right_elevon.clamp(
            *mixer_settings.right_elevon.range.start(),
            *mixer_settings.right_elevon.range.end(),
        ) as u16;

        MotorOutput {
            throttle,
            left_elevon,
            right_elevon,
        }
    }
}

struct MixerSettings {
    left_elevon: OutputMixerSettings,
    right_elevon: OutputMixerSettings,
}

struct OutputMixerSettings {
    mid_point: i16,
    range: RangeInclusive<i16>,
    gain: Vector3<f32>,
}

struct RatePidSettings {
    kff: f32,
    kp: f32,
    ki: f32,
    i_term_max_error: f32,
}

struct SingleAxisRateController {
    settings: RatePidSettings,
    i_term: f32,
}

impl SingleAxisRateController {
    fn new(settings: RatePidSettings) -> Self {
        Self {
            settings,
            i_term: 0.0,
        }
    }

    fn update(&mut self, target_rate: f32, current_rate: f32, dt: f32) -> f32 {
        let error = target_rate - current_rate;

        let ff_term = self.settings.kff * target_rate;
        let p_term = self.settings.kp * error;
        if error.abs() <= self.settings.i_term_max_error {
            self.i_term += self.settings.ki * error * dt;
        }

        ff_term + p_term + self.i_term
    }
}

struct RateController {
    roll_controller: SingleAxisRateController,
    pitch_controller: SingleAxisRateController,
    yaw_controller: SingleAxisRateController,
}

impl RateController {
    fn update(
        &mut self,
        target_rate: Vector3<f32>,
        current_rate: Vector3<f32>,
        dt: f32,
    ) -> Vector3<f32> {
        Vector3::new(
            self.roll_controller
                .update(target_rate.x, current_rate.x, dt),
            self.pitch_controller
                .update(target_rate.y, current_rate.y, dt),
            self.yaw_controller
                .update(target_rate.z, current_rate.z, dt),
        )
    }
}
