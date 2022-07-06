
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



pub fn init() -> UWBSensor<Spi, OutputPin, Ready> 
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
