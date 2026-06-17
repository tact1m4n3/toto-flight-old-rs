/// Inspired by the dshot-pio crate.
/// TODO: support up to 4 motors control per pio instance
use embassy_rp::{
    Peri,
    interrupt::typelevel::Binding,
    pio::{Config, Instance, InterruptHandler, Pio, PioPin, ShiftConfig, ShiftDirection::Left},
};
use embassy_time::{Duration, Timer};

use super::SpinDirection;

#[derive(Clone, Copy)]
pub enum Speed {
    Dshot150,
    Dshot300,
    Dshot600,
    Dshot1200,
}

impl Speed {
    // TODO: maybe compute a fixed point and keep decimals
    fn get_clk_div(self) -> u16 {
        let raw = match self {
            Speed::Dshot150 => 150,
            Speed::Dshot300 => 300,
            Speed::Dshot600 => 600,
            Speed::Dshot1200 => 1200,
        };

        (embassy_rp::clocks::clk_sys_freq() / (raw * 8000)) as u16
    }
}

#[repr(u16)]
#[derive(Clone, Copy)]
pub enum Command {
    Stop = 0,

    Beep1 = 1,
    Beep2 = 2,
    Beep3 = 3,
    Beep4 = 4,
    Beep5 = 5,

    SpinDirectionNormal = 20,
    SpinDirectionReversed = 21,

    ThrottleMin = 48,
}

pub const THROTTLE_MIN: u16 = 48;
pub const THROTTLE_MAX: u16 = 2047;

pub struct DshotPio<'a, PIO: Instance> {
    pio_instance: Pio<'a, PIO>,
}

impl<'a, PIO: Instance> DshotPio<'a, PIO> {
    pub fn new(
        pio: Peri<'a, PIO>,
        irq: impl Binding<PIO::Interrupt, InterruptHandler<PIO>>,
        pin0: Peri<'a, impl PioPin>,
        speed: Speed,
    ) -> DshotPio<'a, PIO> {
        // Define program
        let dshot_pio_program = pio::pio_asm!(
            "set pindirs, 1",
            "entry:"
            "   pull"
            "   out null 16"
            "   set x 15"
            "loop:"
            "   set pins 1"
            "   out y 1"
            "   jmp !y zero"
            "   nop [2]"
            "one:" // 6 and 2
            "   set pins 0"
            "   jmp x-- loop"
            "   jmp reset"
            "zero:" // 3 and 5
            "   set pins 0 [3]"
            "   jmp x-- loop"
            "   jmp reset"
            "reset:" // Blank frame
            "   nop [31]"
            "   nop [31]"
            "   nop [31]"
            "   jmp entry [31]"
        );

        // Configure program
        let mut cfg = Config::default();
        let mut pio = Pio::new(pio, irq);
        cfg.use_program(&pio.common.load_program(&dshot_pio_program.program), &[]);
        cfg.clock_divider = speed.get_clk_div().into();

        cfg.shift_in = ShiftConfig {
            auto_fill: true,
            direction: Default::default(),
            threshold: 32,
        };

        cfg.shift_out = ShiftConfig {
            auto_fill: Default::default(),
            direction: Left,
            threshold: Default::default(),
        };

        // Set pins and enable all state machines
        let pin0 = pio.common.make_pio_pin(pin0);
        cfg.set_set_pins(&[&pin0]);
        pio.sm0.set_config(&cfg);
        pio.sm0.set_enable(true);

        // Return struct of 1 configured DShot state machine
        DshotPio { pio_instance: pio }
    }
}

impl<'a, PIO: embassy_rp::pio::Instance> super::MotorOutput<1> for DshotPio<'a, PIO> {
    fn disarm(&mut self) {
        let dshot_out = u32::from(encode(Command::Stop as u16, false));
        self.pio_instance.sm0.tx().push(dshot_out);
    }

    fn write(&mut self, us: &[u16; 1]) {
        // map 1000..=2000 -> 48..=2047
        let us = u32::from(us[0]);
        // and-ing by 0x3FF ensures no overflow can happen on the multiplication
        // in case us is very big (it shouldn't be)
        let command = (((us.saturating_sub(1000) & 0x3FF) * 0x7FF0) >> 14) as u16
            + Command::ThrottleMin as u16;
        let dshot_out = u32::from(encode(command, false));
        self.pio_instance.sm0.tx().push(dshot_out);
    }

    async fn beep(&mut self) {
        self.pio_instance
            .sm0
            .tx()
            .wait_push(u32::from(encode(Command::Beep1 as u16, false)))
            .await;
        Timer::after(Duration::from_millis(265)).await; // must sleep at least 260 millis before the next command
    }

    async fn set_spin_direction(&mut self, direction: SpinDirection) {
        let command = match direction {
            SpinDirection::Normal => Command::SpinDirectionNormal,
            SpinDirection::Reversed => Command::SpinDirectionReversed,
        };
        for _ in 0..6 {
            self.pio_instance
                .sm0
                .tx()
                .wait_push(u32::from(encode(command as u16, false)))
                .await;
        }
    }
}

fn encode(command: u16, telemetry: bool) -> u16 {
    let packet = (command << 1) | (telemetry as u16);
    let crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;

    ((packet & 0xFFF) << 4) | crc
}
