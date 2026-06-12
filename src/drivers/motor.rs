// pub mod dshot;

pub mod dshot_pio;

use embedded_hal::pwm::SetDutyCycle;

pub trait MotorOutput<const N: usize> {
    fn write(&mut self, us: &[u16; N]);
}

pub trait ServoOutput<const N: usize> {
    fn write(&mut self, us: &[u16; N]);
}

// NOTE: this assumes that duty increments each microsecond
impl<const N: usize, T: SetDutyCycle> MotorOutput<N> for [T; N] {
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
