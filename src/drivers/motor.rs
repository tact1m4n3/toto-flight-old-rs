use embedded_hal::pwm::SetDutyCycle;

pub trait Motor {
    type Error;
    // TODO: What should happen in case of errors.
    fn write(&mut self, us: u16);
}

impl<T: SetDutyCycle> Motor for T {
    type Error = T::Error;

    fn write(&mut self, us: u16) {
        self.set_duty_cycle(us).ok();
    }
}
