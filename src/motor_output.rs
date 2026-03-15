use embassy_rp::pwm::{self, PwmOutput, SetDutyCycle};
use embassy_time::{Duration, Ticker};
use fixed::FixedU16;
use nalgebra::Vector3;

use crate::{
    PwmOutputsPeripherals, SystemSpawner, control, imu,
    utils::{NEG_ONE_ONE_TO_US, ZERO_ONE_TO_US},
};

pub const PID_LOOP_FREQ_HZ: usize = 500;
pub const PID_LOOP_FREQ_DT: f32 = 1.0 / PID_LOOP_FREQ_HZ as f32;

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
        task(Outputs {
            throttle_pwm,
            left_elevon_pwm,
            right_elevon_pwm,
        })
        .unwrap(),
    );
}

#[embassy_executor::task]
async fn task(mut outputs: Outputs) {
    const LEFT_ELEVON_MIXER: Vector3<f32> = Vector3::new(0.5, 0.5, 0.0);
    const RIGHT_ELEVON_MIXER: Vector3<f32> = Vector3::new(-0.5, 0.5, 0.0);

    const THROTTLE_IDLE_US: u16 = 1000;
    const LEFT_ELEVON_IDLE_US: u16 = 1500;
    const RIGHT_ELEVON_IDLE_US: u16 = 1500;

    let mut rate_controller = RateController {
        roll_controller: SingleAxisRateController::new(RatePidSettings {
            kp: 0.1,
            ki: 0.01,
            kff: 0.5,
            i_term_max_error: 10.0,
        }),
        pitch_controller: SingleAxisRateController::new(RatePidSettings {
            kp: 0.1,
            ki: 0.01,
            kff: 10.0,
            i_term_max_error: 10.0,
        }),
        yaw_controller: SingleAxisRateController::new(RatePidSettings {
            kp: 0.1,
            ki: 0.01,
            kff: 10.0,
            i_term_max_error: 10.0,
        }),
    };

    let mut ticker = Ticker::every(Duration::from_hz(PID_LOOP_FREQ_HZ as u64));

    loop {
        ticker.next().await;

        let (gyro, target) = critical_section::with(|cs| {
            (
                imu::RAW_GYRO.borrow(cs).get(),
                control::TARGET.borrow(cs).get(),
            )
        });

        let Some(target) = target else {
            outputs
                .throttle_pwm
                .set_duty_cycle(THROTTLE_IDLE_US)
                .ok();
            outputs
                .left_elevon_pwm
                .set_duty_cycle(LEFT_ELEVON_IDLE_US)
                .ok();
            outputs
                .right_elevon_pwm
                .set_duty_cycle(RIGHT_ELEVON_IDLE_US)
                .ok();
            continue;
        };

        let pids = rate_controller.update(&target.rate, &gyro, PID_LOOP_FREQ_DT);

        let left_elevon_output = LEFT_ELEVON_MIXER.dot(&pids).clamp(-1.0, 1.0);
        let right_elevon_output = RIGHT_ELEVON_MIXER.dot(&pids).clamp(-1.0, 1.0);

        outputs
            .throttle_pwm
            .set_duty_cycle(ZERO_ONE_TO_US.map(target.throttle) as u16)
            .ok();
        outputs
            .left_elevon_pwm
            .set_duty_cycle(NEG_ONE_ONE_TO_US.map(left_elevon_output) as u16)
            .ok();
        outputs
            .right_elevon_pwm
            .set_duty_cycle(NEG_ONE_ONE_TO_US.map(right_elevon_output) as u16)
            .ok();
    }
}

struct Outputs {
    pub throttle_pwm: PwmOutput<'static>,
    pub left_elevon_pwm: PwmOutput<'static>,
    pub right_elevon_pwm: PwmOutput<'static>,
}

pub struct RatePidSettings {
    pub kff: f32,
    pub kp: f32,
    pub ki: f32,
    pub i_term_max_error: f32,
}

pub struct SingleAxisRateController {
    settings: RatePidSettings,
    i_term: f32,
}

impl SingleAxisRateController {
    pub fn new(settings: RatePidSettings) -> Self {
        Self {
            settings,
            i_term: 0.0,
        }
    }

    pub fn update(&mut self, target_rate: f32, current_rate: f32, dt: f32) -> f32 {
        let error = target_rate - current_rate;

        let ff_term = self.settings.kff * target_rate;
        let p_term = self.settings.kp * error;
        if error.abs() <= self.settings.i_term_max_error {
            self.i_term += self.settings.ki * error * dt;
        }

        ff_term + p_term + self.i_term
    }
}

pub struct RateController {
    pub roll_controller: SingleAxisRateController,
    pub pitch_controller: SingleAxisRateController,
    pub yaw_controller: SingleAxisRateController,
}

impl RateController {
    pub fn update(&mut self, target: &Vector3<f32>, gyro: &Vector3<f32>, dt: f32) -> Vector3<f32> {
        Vector3::new(
            self.roll_controller.update(target.x, gyro.x, dt),
            self.pitch_controller.update(target.y, gyro.y, dt),
            self.yaw_controller.update(target.z, gyro.z, dt),
        )
    }
}
