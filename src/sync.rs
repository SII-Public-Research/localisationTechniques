use dw3000::block;
use embedded_hal::timer::CountDown;
use rppal::{gpio::OutputPin as rppalPin, hal::Timer};
use std::{arch::asm, time::Duration};

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

    pub fn sync_instant(&mut self) -> Result<(), &'static str> {
        self.gpio.set_high();
        self.gpio.set_low();
        Ok(())
    }

    pub fn sync_assembly(&mut self) -> Result<(), &'static str> {
        self.gpio.set_high();
        unsafe {
            // we need to wait 26 ns (1/38.4Mhz)
            asm!("nop", "nop", "nop", "nop", "nop", "nop", "nop"); // at 250 MHz, it's 6-7 clk cnt
            asm!("nop", "nop", "nop", "nop"); // at 400 Mhz, this is 10~11 clk cnt
        }
        self.gpio.set_low();
        Ok(())
    }
}
