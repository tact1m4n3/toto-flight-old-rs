use embassy_time::{Duration, Instant};

pub struct Tracker {
    last_tick_ticks: u32,
    idle_ticks: u32,
    last_tick_instant: Instant,
}

impl Tracker {
    pub fn new() -> Self {
        Self {
            last_tick_ticks: 0,
            idle_ticks: 0,
            last_tick_instant: Instant::now(),
        }
    }

    pub fn on_idle(&mut self, cp: &mut cortex_m::Peripherals) {
        let mut before: u32 = 0;
        let mut after: u32 = 0;
        critical_section::with(|_| {
            before = cp.DWT.cyccnt.read();
            cortex_m::asm::wfe();
            after = cp.DWT.cyccnt.read();
        });
        self.idle_ticks += after.wrapping_sub(before);

        let now_instant = Instant::now();
        if now_instant - self.last_tick_instant > Duration::from_secs(1) {
            let ticks_now = cp.DWT.cyccnt.read();
            let tick_diff = ticks_now.wrapping_sub(self.last_tick_ticks);

            if tick_diff != 0 {
                let cpu_usage =
                    100u64.saturating_sub(u64::from(self.idle_ticks) * 100 / u64::from(tick_diff));
                info!("cpu usage: {}%", cpu_usage);
            }

            self.last_tick_instant = now_instant;
            self.last_tick_ticks = ticks_now;
            self.idle_ticks = 0;
        }
    }
}
