use embassy_time::{Duration, Timer};
use smart_leds::{RGB8, SmartLedsWriteAsync, colors};

use crate::control;

// TODO: support led strips (maybe display the battery level)

pub async fn task<L: SmartLedsWriteAsync<Color = RGB8>>(led: L) {
    let mut blinky = Blinky { led };
    loop {
        let status = control::SHARED_STATE.lock(|state| {
            let state = state.borrow();
            Status {
                arm_state: state.arm_state(),
                failsafe: state.in_failsafe(),
            }
        });
        blinky.blink(&status).await;
    }
}

struct Status {
    arm_state: control::ArmState,
    failsafe: bool,
}

struct Blinky<L: SmartLedsWriteAsync<Color = RGB8>> {
    led: L,
}

impl<L: SmartLedsWriteAsync<Color = RGB8>> Blinky<L> {
    async fn blink(&mut self, status: &Status) {
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

        self.led.write([color]).await.ok();
        Timer::after(delay).await;
        self.led.write([colors::BLACK]).await.ok();
        Timer::after(delay).await;
    }
}
