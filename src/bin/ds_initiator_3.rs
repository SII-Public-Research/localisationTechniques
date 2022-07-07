use std::io::{self, Write};

use localisationtechniques::{
    rtt_ds_algorithms::*,
    uwb_basics::*,
    tools::*,
    experiment_file::*,
    ok_or_panic,
    scanf,
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
    let mut file = create_experiment_file()
        .expect("Failed to create the file for the experiment");
    write_to_experiment_file("Time; D1 row; D2 row; D3 row; D1 filtered; D2 filtered; D3 filtered;\n", &mut file)
        .expect("Failed to format the file for the experiment");
    println!("File is created and ready for the experimentation");

    let (mut uwbsensor1, mut uwbsensor2, mut uwbsensor3) = init_3();

    // A start measure is needed for initiatlise IIR filter
    (uwbsensor1, uwbsensor2, uwbsensor3) =
    match rtt_ds_initiator_3(uwbsensor1, uwbsensor2, uwbsensor3, OptionTimeout::Some(Timeout::new(500))).await {
        Ok((mut sensor1, mut sensor2, mut sensor3)) => {
            sensor1.previous_distance_filtre = sensor1.distance;
            sensor2.previous_distance_filtre = sensor2.distance;
            sensor3.previous_distance_filtre = sensor3.distance;
            (sensor1, sensor2, sensor3)
        }
        Err((sensor1, sensor2, sensor3, _e)) => {
            println!("1st measure failed");
            (sensor1, sensor2, sensor3)
        }
    };

    loop {
        let trash_measure = 25; // 25 measure to stabilize antenna
        let nb_measure: u32;
        let mut nb_current_measure = 0;
        let mut nb_mean = 0.0;
        let mut d1_mean = 0.0;
        let mut d2_mean = 0.0;
        let mut d3_mean = 0.0;

        scanf!(nb_measure,
            "How many measure are needed ? (25 will be trashed) : ",
            "Invalid number, try again : "
        );
        println!("Starting {} measures, the first {} are thrown away", nb_measure, trash_measure);  

        while nb_current_measure < nb_measure {
            println!("\nNew measure");
            (uwbsensor1, uwbsensor2, uwbsensor3) = match rtt_ds_initiator_3(uwbsensor1, uwbsensor2, uwbsensor3, OptionTimeout::Some(Timeout::new(500))).await {
                Ok(sensors) => {
                    println!("OK");
                    io::stdout().flush().unwrap();
                    sensors
                }
                Err((mut sensor1, mut sensor2, mut sensor3, _e)) => {
                    sensor1.distance = 100_000.0;
                    sensor2.distance = 100_000.0;
                    sensor3.distance = 100_000.0;
                    println!("KO");
                    io::stdout().flush().unwrap();
                    (sensor1, sensor2, sensor3)
                }
            };

            if uwbsensor1.distance < 100_000.0 && uwbsensor2.distance < 100_000.0 && uwbsensor3.distance <100_000.0 {
                if nb_current_measure >= trash_measure {
                    d1_mean += uwbsensor1.distance_filtre;
                    d2_mean += uwbsensor2.distance_filtre;
                    d3_mean += uwbsensor3.distance_filtre;
                    nb_mean += 1.0;
                }
                nb_current_measure += 1;
                let time = get_time().await;
                let text = time + ";"
                    + &uwbsensor1.distance.to_string() + ";"
                    + &uwbsensor2.distance.to_string() + ";"
                    + &uwbsensor3.distance.to_string() + ";"
                    + &uwbsensor1.distance_filtre.to_string() + ";"
                    + &uwbsensor2.distance_filtre.to_string() + ";"
                    + &uwbsensor3.distance_filtre.to_string() + ";\n";

                write_to_experiment_file(&text, &mut file)
                    .expect("Failed to format the file for the experiment");
            }

            thread::sleep(Duration::from_millis(10));
        };

        d1_mean = d1_mean / nb_mean;
        d2_mean = d2_mean / nb_mean;
        d3_mean = d3_mean / nb_mean;
        println!("Mean distances : \n D1 : {}\n D2 : {}\n D3 : {}\n", d1_mean, d2_mean, d3_mean);
    }   

}


fn init_3() -> (UWBSensor<Spi, OutputPin, Ready>, UWBSensor<Spi, OutputPin, Ready>, UWBSensor<Spi, OutputPin, Ready>)
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



