use dw3000::block;
use embedded_hal::timer::CountDown;
use rppal::{gpio::OutputPin as rppalPin, hal::Timer};
use std::time::Duration;

#[derive(Debug)]
pub struct RpiSync {
    pub gpio: rppalPin,
    timer: Timer,
}

impl RpiSync {
    pub fn new(gpio: rppalPin) -> Self {
        Self {
            gpio,
            timer: Timer::new(),
        }
    }

    pub fn sync(&mut self) -> Result<(), &'static str> {
        self.gpio.set_high();
        self.timer.start(Duration::from_nanos(32));
        block!(self.timer.wait()).map_err(|_| "Timer failed")?;
        self.gpio.set_low();
        Ok(())
    }
}
