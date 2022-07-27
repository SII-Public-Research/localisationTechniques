use std::io::{self, Write};

use localisationtechniques::{
    experiment_file::*, ok_or_panic, rtt_ds_algorithms::*, scanf, tools::*, uwb_basics::*,
};

use chrono::prelude::*;

use rppal::gpio::{Gpio, OutputPin};
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};
use std::time::Duration;

use dw3000::hl::Ready;

use std::thread;

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
    let mut file = create_experiment_file().expect("Failed to create the file for the experiment");
    write_to_experiment_file("Time; D1 row; D1 filtered;\n", &mut file)
        .expect("Failed to format the file for the experiment");
    println!("File is created and ready for the experimentation");

    let mut uwbsensor = init();

    // A start measure is needed for initiatlise IIR filter
    uwbsensor = match rtt_ds_initiator(uwbsensor, OptionTimeout::Some(Timeout::new(500))).await {
        Ok(mut sensor1) => {
            sensor1.previous_filtered_distance = sensor1.distance;
            sensor1
        }
        Err((_sensor1, _e)) => {
            panic!("1st measure failed");
        }
    };

    loop {
        let trash_measure = 25; // 25 measure to stabilize antenna
        let nb_measure: u32;
        let mut nb_current_measure = 0;
        let mut nb_mean = 0.0;
        let mut d1_mean = 0.0;

        scanf!(
            nb_measure,
            "How many measure are needed ? (25 will be trashed) : ",
            "Invalid number, try again : "
        );
        println!(
            "Starting {} measures, the first {} are thrown away",
            nb_measure, trash_measure
        );

        while nb_current_measure < nb_measure {
            println!("\nNew measure");
            uwbsensor =
                match rtt_ds_initiator(uwbsensor, OptionTimeout::Some(Timeout::new(500))).await {
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

            if uwbsensor.distance < 100_000.0 {
                if nb_current_measure >= trash_measure {
                    d1_mean += uwbsensor.filtered_distance;
                    nb_mean += 1.0;
                }
                nb_current_measure += 1;
                let time = get_time().await;
                let text = time
                    + ";"
                    + &uwbsensor.distance.to_string()
                    + ";"
                    + &uwbsensor.filtered_distance.to_string()
                    + ";\n";

                write_to_experiment_file(&text, &mut file)
                    .expect("Failed to format the file for the experiment");
            }

            thread::sleep(Duration::from_millis(10));
        }

        d1_mean = d1_mean / nb_mean;
        println!("Mean distance : {}\n", d1_mean);
    }
}

fn init() -> UWBSensor<Spi, OutputPin, Ready> {
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
    let mut uwbsensor = ok_or_panic(
        UWBSensor::new(spi, cs),
        "Failed to create an UWBSensor object",
    );
    uwbsensor.id = 1;
    uwbsensor
        .dw3000
        .set_address(PAN_ID, ADD_S_ANCH1)
        .expect("Erreur set adress");

    println!("Init OK");
    uwbsensor
}
