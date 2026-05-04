use embassy_rp::pwm::{self, PwmOutput, SetDutyCycle};
use embassy_time::{Duration, Ticker};
use fixed::FixedU16;
use nalgebra::Vector3;

use crate::{PwmOutputsPeripherals, SystemSpawner, nav_loop, sensors::imu, utils::{NEG_ONE_ONE_TO_US, SyncWatch, ZERO_ONE_TO_US}};

pub static MOTOR_OUTPUT: SyncWatch<MotorOutput> = SyncWatch::new(MotorOutput::IDLE);

#[derive(Clone, Copy)]
pub struct MotorOutput {
    pub throttle: u16,
    pub left_elevon: u16,
    pub right_elevon: u16,
}

impl MotorOutput {
    pub const IDLE: MotorOutput = MotorOutput {
        throttle: 1000,
        left_elevon: 1500,
        right_elevon: 1500,
    };
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

#[embassy_executor::task]
async fn task(mut output_pwms: OutputPwms) {
    const LEFT_ELEVON_MIXER: Vector3<f32> = Vector3::new(0.5, 0.5, 0.0);
    const RIGHT_ELEVON_MIXER: Vector3<f32> = Vector3::new(-0.5, 0.5, 0.0);

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

    let mut ticker = Ticker::every(Duration::from_hz(FREQ_HZ));

    loop {
        ticker.next().await;

        let gyro = imu::GYRO.get();
        let target = nav_loop::TARGET.get();

        if let Some(target) = target {
            let pids = rate_controller.update(&target.rate, &gyro, FREQ_DT);

            let motor_output = MotorOutput {
                throttle: ZERO_ONE_TO_US.map(target.throttle) as u16,
                left_elevon: NEG_ONE_ONE_TO_US.map(LEFT_ELEVON_MIXER.dot(&pids)) as u16,
                right_elevon: NEG_ONE_ONE_TO_US.map(RIGHT_ELEVON_MIXER.dot(&pids)) as u16,
            };

            output_pwms.apply(motor_output);
        } else {
            output_pwms.apply(MotorOutput::IDLE);
        }
    }
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

struct OutputPwms {
    throttle: PwmOutput<'static>,
    left_elevon: PwmOutput<'static>,
    right_elevon: PwmOutput<'static>,
}

impl OutputPwms {
    fn apply(&mut self, output: MotorOutput) {
        self.throttle.set_duty_cycle(output.throttle).ok();
        self.left_elevon.set_duty_cycle(output.left_elevon).ok();
        self.right_elevon
            .set_duty_cycle(output.right_elevon)
            .ok();
    }
}
