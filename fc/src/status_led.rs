use embassy_time::{Duration, Timer};
use smart_leds::colors;

use crate::{control, drivers::led_strip::LedStrip};

// TODO: support led strips (maybe display the battery level)

pub async fn task<L: LedStrip>(mut led: L) {
    loop {
        let status = control::status();

        let color = match status.arm_state {
            control::ArmState::Disarmed => colors::DARK_GREEN,
            control::ArmState::ArmFailed => colors::DARK_BLUE,
            control::ArmState::Armed => colors::RED,
        };

        let delay = if status.failsafe {
            Duration::from_millis(200)
        } else {
            Duration::from_millis(500)
        };

        led.write(&[color]).await;
        Timer::after(delay).await;
        led.write(&[colors::BLACK]).await;
        Timer::after(delay).await;
    }
}
