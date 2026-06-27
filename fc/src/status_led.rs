use embassy_time::{Duration, Timer};
use smart_leds::{RGB8, SmartLedsWriteAsync, colors};

use crate::control;

// TODO: support led strips (maybe display the battery level)

pub async fn task<L: SmartLedsWriteAsync<Color = RGB8>>(mut led: L) {
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

        led.write([color]).await.ok();
        Timer::after(delay).await;
        led.write([colors::BLACK]).await.ok();
        Timer::after(delay).await;
    }
}
