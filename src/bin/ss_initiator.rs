
use rppal::gpio::{Gpio, OutputPin};
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};

use std::io::{self, Write};
use std::thread;
use std::time::Duration;

use dw3000::hl::Ready;
use localisationtechniques::{
    rtt_ss_algorithms::*,
    uwb_basics::*,
    tools::*,
    experiment_file::*,
    ok_or_panic,
    scanf,
};

use chrono::*;

// Returns current time HH:MM:SS
async fn get_time() -> String {
    let local: DateTime<Local> = Local::now(); // e.g. `2014-11-28T21:45:59.324310806+09:00`
    let local = local.time().format("%H:%M:%S").to_string(); // format("%Y-%m-%dT%H:%M")
    local
}

#[tokio::main]
async fn main()  {

    // Creates and prepare csv to export data
    println!("Creating file for the experimentation");
    let mut file = create_experiment_file()
        .expect("Failed to create the file for the experiment");
    write_to_experiment_file("Time; D1 row;\n", &mut file)
        .expect("Failed to format the file for the experiment");
    println!("File is created and ready for the experimentation");
    
    let mut uwbsensor = init();

    loop {
        let nb_measure: u32;
        let mut nb_current_measure = 0;

        scanf!(nb_measure,
            "How many measure are needed ? : ",
            "Invalid number, try again : "
        );

        while nb_current_measure < nb_measure {
            println!("\nNew measure");
            uwbsensor = match rtt_ss_inititor(uwbsensor, OptionTimeout::Some(Timeout::new(500))) {
                Ok(sensor) => {
                    println!("OK");
                    println!("Distance = {}", sensor.distance);
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

            if uwbsensor.distance < 100_000.0 {
                nb_current_measure += 1;
                let time = get_time().await; 
                let texte = time + ";"
                    + &uwbsensor.distance.to_string() + ";\n";

                write_to_experiment_file(&texte, &mut file)
                    .expect("Failed to format the file for the experiment");
            }
            thread::sleep(Duration::from_millis(25));
        }
    }
}


fn init() -> UWBSensor<Spi, OutputPin, Ready>
{ 
    /******************************************************* */
	/************        BASIC CONFIGURATION      ********** */
	/******************************************************* */

    let spi = Spi::new(Bus::Spi1, SlaveSelect::Ss0, 500_000, Mode::Mode0)
        .expect("Failed to configure the spi");
    let gpio = Gpio::new()
        .expect("Failed to configure GPIO");
    let cs= gpio.get(16).unwrap().into_output();

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
