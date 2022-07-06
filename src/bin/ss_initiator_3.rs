
use rppal::gpio::{Gpio, OutputPin};
use rppal::hal::Timer;
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};

use std::io::{self, Write};
use std::thread;
use std::time::Duration;

use dw3000::hl;
use raspberry_client::{ok_or_panic, scanf};
use raspberry_client::uwb_sensor::*;

use raspberry_client::experiment_file::*;
use chrono::prelude::*;



// Returns current time HH:MM:SS
async fn get_time() -> String {
    let local: DateTime<Local> = Local::now(); // e.g. `2014-11-28T21:45:59.324310806+09:00`
    let local = local.time().format("%H:%M:%S").to_string(); // format("%Y-%m-%dT%H:%M")
    local
}

#[tokio::main]
async fn main() {
    // Creates and prepare csv to export data
    println!("Creating file for the experimentation");
    let mut file = create_experiment_file()
        .expect("Failed to create the file for the experiment");
    write_to_experiment_file("Time; D1 row; D2 row; D3 row;\n", &mut file)
        .expect("Failed to format the file for the experiment");
    println!("File is created and ready for the experimentation");
 
    
    let (mut uwbsensor1, mut uwbsensor2, mut uwbsensor3) = init_3();
    loop {
        let _timer = Timer::new();
        let nb_measure: u32;
        let mut nb_current_measure = 0;

        scanf!(nb_measure,
            "How many measure are needed ? : ",
            "Invalid number, try again : "
        );
              
        while nb_current_measure < nb_measure {
            println!("\nNew measure");
            uwbsensor1 = match rtt_ss_inititor(uwbsensor1, Timeout::new(_timer, Duration::from_millis(500))) {
                Ok(sensor) => {
                    println!("OK");
                    println!("Distance 1 = {}", sensor.distance);
                    io::stdout().flush().unwrap();
                    sensor
                }
                Err((mut sensor, _e)) => {
                    sensor.distance = 100_000.;
                    print!("KO");
                    io::stdout().flush().unwrap();
                    sensor
                }
            };
            thread::sleep(Duration::from_millis(25));

            uwbsensor2 = match rtt_ss_inititor(uwbsensor2, Timeout::new(_timer, Duration::from_millis(500))) {
                Ok(sensor) => {
                    println!("OK");
                    println!("Distance 2 = {}", sensor.distance);
                    io::stdout().flush().unwrap();
                    sensor
                }
                Err((mut sensor, _e)) => {
                    sensor.distance = 100_000.;
                    print!("KO");
                    io::stdout().flush().unwrap();
                    sensor
                }
            };
            thread::sleep(Duration::from_millis(25));

            uwbsensor3 = match rtt_ss_inititor(uwbsensor3, Timeout::new(_timer, Duration::from_millis(500))) {
                Ok(sensor) => {
                    println!("OK");
                    println!("Distance 3= {}", sensor.distance);
                    io::stdout().flush().unwrap();
                    sensor
                }
                Err((mut sensor, _e)) => {
                    sensor.distance = 100_000.;
                    print!("KO");
                    io::stdout().flush().unwrap();
                    sensor
                }
            };


            if uwbsensor1.distance < 100_000.0 && uwbsensor3.distance < 100_000.0 && uwbsensor2.distance < 100_000.0 {
                nb_current_measure += 1;
                let time = get_time().await; // on récupère l'heure actuelle
                let texte = time + ";"
                    + &uwbsensor1.distance.to_string() + ";"
                    + &uwbsensor2.distance.to_string() + ";"
                    + &uwbsensor3.distance.to_string() + ";\n";
                    
                write_to_experiment_file(&texte, &mut file)
                    .expect("Failed to format the file for the experiment");
            }
            thread::sleep(Duration::from_millis(25));
        }
    }
}


pub fn init_3() -> (UWBSensor<Spi, OutputPin, hl::Ready>, UWBSensor<Spi, OutputPin, hl::Ready>, UWBSensor<Spi, OutputPin, hl::Ready>) 
{
    /******************************************************* */
	/************        BASIC CONFIGURATION      ********** */
	/******************************************************* */
    let spi1 = Spi::new(Bus::Spi1, SlaveSelect::Ss0, 4_500_000, Mode::Mode0).unwrap();
    let spi2 = Spi::new(Bus::Spi1, SlaveSelect::Ss1, 4_500_000, Mode::Mode0).unwrap();
    let spi3 = Spi::new(Bus::Spi1, SlaveSelect::Ss2, 4_500_000, Mode::Mode0).unwrap();
    
    /* /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\  */
	/* An unknown problem, for the moment, forces us to select 3 different CS from the default ones to make it work */
    /* /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\ /!\  */
    let gpio = Gpio::new().unwrap();
    let cs0 = gpio.get(23).unwrap().into_output();
    let cs1 = gpio.get(27).unwrap().into_output();
    let cs2 = gpio.get(22).unwrap().into_output();

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
    let mut uwbsensor1 = ok_or_panic(UWBSensor::new(spi1, cs0),"Failed to create an UWBSensor object");
    let mut uwbsensor2 = ok_or_panic(UWBSensor::new(spi2, cs1),"Failed to create an UWBSensor object");
    let mut uwbsensor3 = ok_or_panic(UWBSensor::new(spi3, cs2),"Failed to create an UWBSensor object");

    uwbsensor1.id = 1;
    uwbsensor2.id = 2;
    uwbsensor3.id = 3;

    uwbsensor1.dw3000.set_address(PAN_ID, ADD_S_ANCH1).expect("Erreur set adress");
    uwbsensor2.dw3000.set_address(PAN_ID, ADD_S_ANCH2).expect("Erreur set adress");
    uwbsensor3.dw3000.set_address(PAN_ID, ADD_S_ANCH3).expect("Erreur set adress");

    println!("Init OK");

    return (uwbsensor1, uwbsensor2, uwbsensor3)
}
