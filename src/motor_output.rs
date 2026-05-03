use embassy_rp::pwm::{self, PwmOutput, SetDutyCycle};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use fixed::FixedU16;

use crate::{PwmOutputsPeripherals, SystemSpawner};

pub static OUTPUT: Signal<CriticalSectionRawMutex, Output> = Signal::new();

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

#[embassy_executor::task]
async fn task(mut output_pwms: OutputPwms) {
    output_pwms.apply(&Output::IDLE);
    loop {
        let output = OUTPUT.wait().await;
        output_pwms.apply(&output);
    }
}

#[derive(Clone, Copy)]
pub struct Output {
    pub throttle: u16,
    pub left_elevon: u16,
    pub right_elevon: u16,
}

impl Output {
    pub const IDLE: Output = Output {
        throttle: 1000,
        left_elevon: 1500,
        right_elevon: 1500,
    };
}

struct OutputPwms {
    throttle: PwmOutput<'static>,
    left_elevon: PwmOutput<'static>,
    right_elevon: PwmOutput<'static>,
}

impl OutputPwms {
    fn apply(&mut self, outputs_us: &Output) {
        self.throttle.set_duty_cycle(outputs_us.throttle).ok();
        self.left_elevon.set_duty_cycle(outputs_us.left_elevon).ok();
        self.right_elevon
            .set_duty_cycle(outputs_us.right_elevon)
            .ok();
    }
}
