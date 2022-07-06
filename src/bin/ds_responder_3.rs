// `block_on` blocks the current thread until the provided future has run to completion. Other executors provide more complex behavior, like scheduling
// multiple futures onto the same thread.
use futures::executor::block_on;

use rppal::gpio::{Gpio, OutputPin as rppalOutputPin};
use rppal::hal::Timer;
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};

use std::io::{self, Write as stdWrite};
use std::thread;
use std::time::Duration;

use raspberry_client::uwb_sensor::*;
use raspberry_client::{error::Error, ok_or_panic};

use dw3000::{hl, Ready};

use embedded_hal::{
    blocking::spi::{Transfer, Write},
    digital::v2::OutputPin,
    timer::CountDown,
};

async fn async_main() {
    let mut uwbsensor = init();
    loop {
        println!("\nNew measure");
        let _timer = Timer::new();
        uwbsensor = match rtt_ds_responder_3(uwbsensor, Timeout::new(_timer, Duration::from_millis(5000)), 16385) {
            Ok(sensor) => {
                print!("OK");
                io::stdout().flush().unwrap();
                sensor
            }
            Err((mut sensor, _e)) => {
                sensor.distance = 100_000.0;
                print!("KO");
                io::stdout().flush().unwrap();
                sensor
            }
        };
    }
}

fn main() {
    block_on(async_main());
}


fn init() -> UWBSensor<Spi, OutputPin, Ready> 
{

    /******************************************************* */
	/************        BASIC CONFIGURATION      ********** */
	/******************************************************* */
    let spi = Spi::new(Bus::Spi1, SlaveSelect::Ss0, 4_500_000, Mode::Mode0).unwrap();
    let gpio = Gpio::new().unwrap();
    let cs = gpio.get(16).unwrap().into_output();

    /****************************************************** */
	/*****                DW3000 RESET              ******* */
	/****************************************************** */

    let mut reset = gpio
        .get(4)
        .expect("Failed to set up RESET PIN")
        .into_output();
    reset.set_low();
    reset.set_high();

    thread::sleep(Duration::from_millis(500));

    // create an UWBSensor
    let mut uwbsensor = ok_or_panic(UWBSensor::new(spi, cs),"Failed to create an UWBSensor object");
    uwbsensor.id = 1;
    uwbsensor.dw3000.set_address(PAN_ID, ADD_S_ANCH1).expect("Erreur set adress");

    println!("Init OK");
    uwbsensor
}
