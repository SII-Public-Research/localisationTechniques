use std::io::{self, Write};

use raspberry_client::uwb_sensor::*;
use raspberry_client::{ok_or_panic, scanf};
use raspberry_client::experiment_file::*;

use chrono::prelude::*;

use rppal::gpio::{Gpio, OutputPin};
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};
use rppal::hal::Timer;
use std::time::Duration;

use dw3000::hl;

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
    write_to_experiment_file("Time; D1 row; D1 filtered;\n", &mut file)
        .expect("Failed to format the file for the experiment");
    println!("File is created and ready for the experimentation");

    let mut uwbsensor = init();

    // A start measure is needed for initiatlise IIR filter
    uwbsensor = match rtt_ds_initiator(uwbsensor).await {
        Ok(mut sensor1) => {
            sensor1.previous_distance_filtre = sensor1.distance;
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

        scanf!(nb_measure,
            "How many measure are needed ? (25 will be trashed) : ",
            "Invalid number, try again : "
        );
        println!("Starting {} measures, the first {} are thrown away", nb_measure, trash_measure);  

        while nb_current_measure < nb_measure {
            println!("\nNew measure");
            uwbsensor = match rtt_ds_initiator(uwbsensor).await {
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
                    d1_mean += uwbsensor.distance_filtre;
                    nb_mean += 1.0;
                }
                nb_current_measure += 1;
                let time = get_time().await;
                let text = time + ";"
                    + &uwbsensor.distance.to_string() + ";"
                    + &uwbsensor.distance_filtre.to_string() + ";\n";

                write_to_experiment_file(&text, &mut file)
                    .expect("Failed to format the file for the experiment");
            }

            thread::sleep(Duration::from_millis(10));
        };

        d1_mean = d1_mean / nb_mean;
        println!("Mean distance : {}\n", d1_mean);
    }   

}

async fn rtt_ds_initiator(
    mut uwb: UWBSensor<Spi, OutputPin, hl::Ready>
) -> Result<
    UWBSensor<Spi, OutputPin, hl::Ready>,
    (UWBSensor<Spi, OutputPin, hl::Ready>, &'static str)> 
{
    let mut data_anch1 = UwbRData {
        addr_sender: Some(dw3000::mac::Address::Short(dw3000::mac::PanId(0x111), dw3000::mac::ShortAddress(0x0))),
        data: [0; 6],
        r_time: 0,
    };
        
    println!("STEP 1 : Sending first ping...");
    uwb = ok_or_panic(uwb.uwb_send(&[0], None), "fail");


    /************************************/
    /* Blocking reception on the module */
    /************************************/
    println!("STEP 2 : Waiting measurement request on the anchor...");

    let _timer = Timer::new();
    let rcv_timeout: u64 = 100; // millis

    let mut uwb = match uwb.uwb_receive(Timeout::new(_timer, Duration::from_millis(rcv_timeout)), &mut data_anch1) {
        Ok(mut sensor) => {
            sensor.timing_data[1] = data_anch1.r_time;
            if data_anch1.data[0] == 1099511627775  { // 2^40 - 1 
                sensor.error = 1;  
            }
            sensor
        }
        Err((mut sensor, _e)) => {
            sensor.timing_data[1] = 0;  
            println!("Anchor - Error");
            sensor.error = 2;
            sensor
        }
    };

    if uwb.timing_data[1] == 0 {
        uwb.error = 0;
        return Err((uwb, "fail"))
    }


    /***************************/
    /* Offset module responses */
    /***************************/
    println!("STEP 3 : Offset module response...");

    // The buffer is empty because the tag does not need timestamps
    // The final computation is made in each anchor with T2 and T3
    let buff: [u8;10] = [0;10];
    let mut delay1 = 0;
    uwb = ok_or_panic(calc_delay_send(uwb, &mut delay1), "fail");
    uwb = ok_or_panic(uwb.uwb_send(&buff, Some(delay1)), "fail");


    /************************************/
    /* Blocking reception on the module */
    /************************************/
    println!("STEP 4 : Waiting for an answer on the anchor...");

    let mut uwb = match uwb.uwb_receive(Timeout::new(_timer, Duration::from_millis(rcv_timeout)), &mut data_anch1) {
        Ok(mut sensor) => {
            if data_anch1.data[0] == 1099511627775  { // 2^40 - 1 
                sensor.error = 1;  
            } else {
                sensor.timing_data[0] = data_anch1.data[0];  // T1
                sensor.timing_data[3] = data_anch1.data[1];  // T4
                sensor.timing_data[4] = data_anch1.data[4];  // T5
                sensor.timing_data[5] = data_anch1.r_time;   // T6
            }
            sensor
        }
        Err((mut sensor, _e)) => {
            print!("Receiving : Error");
            sensor.error = 2;
            sensor
        }
    };


    /**************************/
    /* Distances calculaction */
    /**************************/
    println!("STEP 5 : Distance calculation...");

    uwb = ok_or_panic(
        calc_distance_double(uwb),
        "Distance calculation failed",
    );
    uwb = ok_or_panic(
        filtre_iir(uwb),
        "Filter failed",
    );

    println!("Distance = {}", uwb.distance);
    println!("Filtered Distance = {}", uwb.distance_filtre);

    Ok(uwb)
}


fn init() -> UWBSensor<Spi, OutputPin, hl::Ready> {

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



