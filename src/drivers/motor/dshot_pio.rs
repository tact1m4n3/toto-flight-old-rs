/// Inspired by the dshot-pio crate.
/// TODO: support up to 4 motors control per pio instance
use embassy_rp::{
    Peri,
    interrupt::typelevel::Binding,
    pio::{Config, Instance, InterruptHandler, Pio, PioPin, ShiftConfig, ShiftDirection::Left},
};
use embassy_time::{Duration, Timer};

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
        match self {
            Speed::Dshot150 => Self::compute(150) as u16,
            Speed::Dshot300 => Self::compute(300) as u16,
            Speed::Dshot600 => Self::compute(600) as u16,
            Speed::Dshot1200 => Self::compute(1200) as u16,
        }
    }

    fn compute(raw: u32) -> u32 {
        embassy_rp::clocks::clk_sys_freq() / (raw * 8 * 1000)
    }
}

#[repr(u16)]
#[derive(Clone, Copy)]
pub enum Beep {
    _1 = 1,
    _2 = 2,
    _3 = 3,
    _4 = 4,
    _5 = 5,
}

#[repr(u16)]
#[derive(Clone, Copy)]
pub enum SpinDirection {
    Normal = 20,
    Reversed = 21,
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

    pub async fn set_direction(&mut self, direction: SpinDirection) {
        for _ in 0..6 {
            self.pio_instance
                .sm0
                .tx()
                .wait_push(u32::from(encode(direction as u16, false)))
                .await;
        }
    }

    pub async fn beep(&mut self, beep: Beep) {
        self.pio_instance
            .sm0
            .tx()
            .wait_push(u32::from(encode(beep as u16, false)))
            .await;
        Timer::after(Duration::from_millis(265)).await; // must sleep at least 260 millis before the next command
    }
}

impl<'a, PIO: embassy_rp::pio::Instance> super::MotorOutput<1> for DshotPio<'a, PIO> {
    fn write(&mut self, us: &[u16; 1]) {
        // map 1000..=2000 -> 48..=2047
        let us = u32::from(us[0]);
        // and-ing by 0x3FF ensures no overflow can happen on the multiplication
        // in case us is very big (it shouldn't be)
        let command = (((us.saturating_sub(1000) & 0x3FF) * 0x7FF0) >> 14) + 48;
        let dshot_out = u32::from(encode(command as u16, false));
        self.pio_instance.sm0.tx().push(dshot_out);
    }
}

fn encode(command: u16, telemetry: bool) -> u16 {
    // value = 100000101100
    // (>>4) = 000010000010 # right shift value by 4
    // (^) = 100010101110 # XOR with value
    // (>>8) = 000000001000 # right shift value by 8
    // (^) = 100010100110 # XOR with previous XOR
    // (0x0F) = 000000001111 # Mask 0x0F
    // (&) = 000000000110 # CRC

    let crc = {
        let a = (command >> 4) ^ command;
        let b = (a >> 8) ^ a;
        b & 0xF
    };

    ((command & 0x7FF) << 5) | (u16::from(telemetry) << 4) | crc
}
