
use rppal::gpio::{Gpio, OutputPin};
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};

use std::io::{self, Write};

use raspberry_client::ok_or_panic;
use raspberry_client::uwb_sensor::*;

use dw3000::hl;

fn main() {
    let mut uwbsensor = init();

    loop {
        println!("\nNew measure");
        uwbsensor = match rtt_ss_responder(uwbsensor, OptionTimeout::None) {
            Ok(sensor) => {
                println!("OK");
                io::stdout().flush().unwrap();
                sensor
            }
            Err((mut sensor, _e)) => {
                sensor.distance = 100_000.0;
                println!("KO");
                io::stdout().flush().unwrap();
                sensor
            }
        };
    }
}


fn init() -> UWBSensor<Spi, OutputPin, hl::Ready> {
   
    /******************************************************* */
	/************        BASIC CONFIGURATION      ********** */
	/******************************************************* */

    let spi = Spi::new(Bus::Spi1, SlaveSelect::Ss0, 4_500_000, Mode::Mode0)
        .expect("Failed to configure the spi");
    let gpio = Gpio::new()
        .expect("Failed to configure GPIO");
    let cs = gpio
        .get(16)
        .expect("Failed to set up CS PIN")
        .into_output();

    /****************************************************** */
	/*****                DW3000 RESET              ******* */
	/****************************************************** */

    let mut reset = gpio
        .get(4)
        .expect("Failed to set up RESET PIN")
        .into_output();
    reset.set_low();
    reset.set_high();

    /****************************************************** */
	/*********        UWBsensor CONFIGURATION      ******** */
	/****************************************************** */

    let uwbsensor = ok_or_panic(
        UWBSensor::new(spi, cs),
        "Failed to create an UWBSensor object",
    );
    println!("Init OK");
    
    uwbsensor
}
