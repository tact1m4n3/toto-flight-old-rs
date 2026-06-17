use embedded_hal::pwm::SetDutyCycle;

pub mod dshot_pio;

#[derive(Clone, Copy)]
pub enum SpinDirection {
    Normal,
    Reversed,
}

pub trait MotorOutput<const N: usize> {
    fn disarm(&mut self);
    fn write(&mut self, us: &[u16; N]);
    async fn beep(&mut self) {}
    async fn set_spin_direction(&mut self, direction: SpinDirection) {
        _ = direction;
    }
}

pub trait ServoOutput<const N: usize> {
    fn write(&mut self, us: &[u16; N]);
}

// NOTE: this assumes that duty increments each microsecond
impl<const N: usize, T: SetDutyCycle> MotorOutput<N> for [T; N] {
    fn disarm(&mut self) {
        for pwm_output in self.iter_mut() {
            pwm_output.set_duty_cycle(1000).ok();
        }
    }

    fn write(&mut self, us: &[u16; N]) {
        for (pwm_output, us) in self.iter_mut().zip(us) {
            pwm_output.set_duty_cycle(*us).ok();
        }
    }
}

// NOTE: this assumes that duty increments each microsecond
impl<const N: usize, T: SetDutyCycle> ServoOutput<N> for [T; N] {
    fn write(&mut self, us: &[u16; N]) {
        for (pwm_output, us) in self.iter_mut().zip(us) {
            pwm_output.set_duty_cycle(*us).ok();
        }
    }
}
