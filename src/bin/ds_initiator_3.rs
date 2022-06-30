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
    write_to_experiment_file("Time; D1 row; D2 row; D3 row; D1 filtered; D2 filtered; D3 filtered;\n", &mut file)
        .expect("Failed to format the file for the experiment");
    println!("File is created and ready for the experimentation");

    let (mut uwbsensor1, mut uwbsensor2, mut uwbsensor3) = init();

    // A start measure is needed for initiatlise IIR filter
    (uwbsensor1, uwbsensor2, uwbsensor3) =
    match rtt_ds_initiator_3(uwbsensor1, uwbsensor2, uwbsensor3).await {
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
            (uwbsensor1, uwbsensor2, uwbsensor3) = match rtt_ds_initiator_3(uwbsensor1, uwbsensor2, uwbsensor3).await {
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

async fn rtt_ds_initiator_3(
    mut uwb1: UWBSensor<Spi, OutputPin, hl::Ready>, 
    uwb2: UWBSensor<Spi, OutputPin, hl::Ready>, 
    uwb3: UWBSensor<Spi, OutputPin, hl::Ready> 
) -> Result<
    (UWBSensor<Spi, OutputPin, hl::Ready>, UWBSensor<Spi, OutputPin, hl::Ready>, UWBSensor<Spi, OutputPin, hl::Ready>),
    (UWBSensor<Spi, OutputPin, hl::Ready>, UWBSensor<Spi, OutputPin, hl::Ready>, UWBSensor<Spi, OutputPin, hl::Ready>,
    &'static str)> 
{
    let mut data_anch1 = UwbRData {
        addr_sender: Some(dw3000::mac::Address::Short(dw3000::mac::PanId(0x111), dw3000::mac::ShortAddress(0x0))),
        data: [0; 6],
        r_time: 0,
    };
    let mut data_anch2 = UwbRData {
        addr_sender: Some(dw3000::mac::Address::Short(dw3000::mac::PanId(0x111), dw3000::mac::ShortAddress(0x0))),
        data: [0; 6],
        r_time: 0,
    };
    let mut data_anch3 = UwbRData {
        addr_sender: Some(dw3000::mac::Address::Short(dw3000::mac::PanId(0x111), dw3000::mac::ShortAddress(0x0))),
        data: [0; 6],
        r_time: 0,
    };

        
    println!("STEP 1 : Sending first ping...");
    uwb1 = ok_or_panic(uwb1.uwb_send(&[0], None), "fail");


    /****************************************************/
    /* Asynchronous blocking reception on the 3 modules */
    /****************************************************/
    println!("STEP 2 : Waiting measurement request on all 3 anchors...");

    let _timer = Timer::new();
    let rcv_timeout: u64 = 5000; // millis

    let blocking_task1 = tokio::task::spawn_blocking(move || {
        let uwb1 = match uwb1.uwb_receive(Timeout::new(_timer, Duration::from_millis(rcv_timeout)), &mut data_anch1) {
            Ok(mut sensor) => {
                sensor.timing_data[1] = data_anch1.r_time;
                if data_anch1.data[0] == 1099511627775  { // 2^40 - 1 
                    sensor.error = 1;  
                }
                sensor
            }
            Err((mut sensor, _e)) => {
                sensor.timing_data[1] = 0;  
                println!("Anchor 1 - Error");
                sensor.error = 2;
                sensor
            }
        };
        uwb1
    });
    let blocking_task2 = tokio::task::spawn_blocking(move || {
        let uwb2 = match uwb2.uwb_receive(Timeout::new(_timer, Duration::from_millis(rcv_timeout)), &mut data_anch2) {
            Ok(mut sensor) => {
                sensor.timing_data[1] = data_anch2.r_time;
                if data_anch1.data[0] == 1099511627775  { // 2^40 - 1 
                    sensor.error = 1;  
                }
                sensor
            }
            Err((mut sensor, _e)) => {
                sensor.timing_data[1] = 0;
                println!("Anchor 2 - Error");
                sensor.error = 2;
                sensor
            }
        };
        uwb2
    });
    let blocking_task3 = tokio::task::spawn_blocking(move || {
        let uwb3 = match uwb3.uwb_receive(Timeout::new(_timer, Duration::from_millis(rcv_timeout)), &mut data_anch3) {
            Ok(mut sensor) => {
                sensor.timing_data[1] = data_anch3.r_time;
                if data_anch1.data[0] == 1099511627775  { // 2^40 - 1 
                    sensor.error = 1;  
                }
                sensor
            }
            Err((mut sensor, _e)) => {
                sensor.timing_data[1] = 0;
                println!("Anchor 3 - Error");
                sensor.error = 2;
                sensor
            }
        };
        uwb3
    });

    let mut uwb1 = blocking_task1.await.unwrap();
    let mut uwb2 = blocking_task2.await.unwrap();
    let mut uwb3 = blocking_task3.await.unwrap();

    if uwb1.timing_data[1] == 0 || uwb2.timing_data[1] == 0 || uwb3.timing_data[1] == 0 {
        uwb1.error=0;
        uwb2.error=0;
        uwb3.error=0;
        return Err((uwb1, uwb2, uwb3, "fail"))
    }


    /*******************************************/
    /* Offset module responses for each module */
    /*******************************************/
    println!("STEP 3 : Offset module responses...");

    // The buffer is empty because the tag does not need timestamps
    // The final computation is made in each anchor with T2 and T3
    let buff: [u8;10] = [0;10];

    let mut delay1 = 0;
    uwb1 = ok_or_panic(calc_delay_send(uwb1, &mut delay1), "fail");
    uwb1 = ok_or_panic(uwb1.uwb_send(&buff, Some(delay1)), "fail");

    let mut delay2 = 0;
    uwb2 = ok_or_panic(calc_delay_send(uwb2, &mut delay2), "fail");
    uwb2 = ok_or_panic(uwb2.uwb_send(&buff, Some(delay2)), "fail");

    let mut delay3 = 0;
    uwb3 = ok_or_panic(calc_delay_send(uwb3, &mut delay3), "fail");
    uwb3 = ok_or_panic(uwb3.uwb_send(&buff, Some(delay3)), "fail");


    /****************************************************/
    /* Asynchronous blocking reception on the 3 modules */
    /****************************************************/
    println!("STEP 4 : Waiting for an answer on each anchor...");

    let blocking_task1 = tokio::task::spawn_blocking(move || {
        let uwb1 = match uwb1.uwb_receive(Timeout::new(_timer, Duration::from_millis(rcv_timeout)), &mut data_anch1) {
            Ok(mut sensor) => {
                if data_anch1.data[0] == 1099511627775  { // 2^40 - 1 
                    sensor.error = 1;  
                }
                else {
                    sensor.timing_data[0] = data_anch1.data[0];  // T1
                    sensor.timing_data[3] = data_anch1.data[1];  // T4
                    sensor.timing_data[4] = data_anch1.data[4];  // T5
                    sensor.timing_data[5] = data_anch1.r_time;   // T6
                }
                sensor
            }
            Err((mut sensor, _e)) => {
                print!("Anchor 1 - Error");
                sensor.error = 2;
                sensor
            }
        };
        uwb1
    });
    let blocking_task2 = tokio::task::spawn_blocking(move || {
        let uwb2 = match uwb2.uwb_receive(Timeout::new(_timer, Duration::from_millis(rcv_timeout)), &mut data_anch2) {
            Ok(mut sensor) => {
                if data_anch1.data[0] == 1099511627775  { // 2^40 - 1 
                    sensor.error = 1;  
                }
                else {
                    sensor.timing_data[0] = data_anch2.data[0];  // T1
                    sensor.timing_data[3] = data_anch2.data[2];  // T4
                    sensor.timing_data[4] = data_anch2.data[4];  // T5
                    sensor.timing_data[5] = data_anch2.r_time;   // T6
                }
                sensor
            }
            Err((mut sensor, _e)) => {
                print!("Anchor 2 - Error");
                sensor.error = 2;
                sensor
            }
        };
        uwb2
    });
    let blocking_task3 = tokio::task::spawn_blocking(move || {
        let uwb3 = match uwb3.uwb_receive(Timeout::new(_timer, Duration::from_millis(rcv_timeout)), &mut data_anch3) {
            Ok(mut sensor) => {
                if data_anch1.data[0] == 1099511627775  { // 2^40 - 1 
                    sensor.error = 1;  
                }
                else {
                    sensor.timing_data[0] = data_anch3.data[0];  // T1
                    sensor.timing_data[3] = data_anch3.data[3];  // T4
                    sensor.timing_data[4] = data_anch3.data[4];  // T5
                    sensor.timing_data[5] = data_anch3.r_time;   // T6
                }
                sensor
            }
            Err((mut sensor, _e)) => {
                print!("Anchor 3 - Timeout");
                sensor.error = 2;
                sensor
            }
        };
        uwb3
    });

    let mut uwb1 = blocking_task1.await.unwrap();
    let mut uwb2 = blocking_task2.await.unwrap();
    let mut uwb3 = blocking_task3.await.unwrap();

    
    /**************************/
    /* Distances calculaction */
    /**************************/
    println!("STEP 5 : Distances calculations...");

    uwb1 = ok_or_panic(
        calc_distance_double(uwb1),
        "Distance calculations anchor 1 failed",
    );
    uwb1 = ok_or_panic(
        filtre_iir(uwb1),
        " Distance filter anchor 01 failed",
    );

    uwb2 = ok_or_panic(
        calc_distance_double(uwb2),
        "Distance calculations anchor 2 failed",
    );
    uwb2 = ok_or_panic(
        filtre_iir(uwb2),
        " Distance filter anchor 01 failed",
    );
    
    uwb3
     = ok_or_panic(
        calc_distance_double(uwb3),
        "Distance calculations anchor 3 failed",
    );
    uwb3 = ok_or_panic(
        filtre_iir(uwb3),
        " Distance filter anchor 01 failed",
    );

    println!("Anchor 1 - Row distance = {}", uwb1.distance);
    println!("Anchor 2 - Row distance = {}", uwb2.distance);
    println!("Anchor 3 - Row distance = {}", uwb3.distance);

    println!("Anchor 1 - Filtered distance = {}", uwb1.distance_filtre);
    println!("Anchor 2 - Filtered distance = {}", uwb2.distance_filtre);
    println!("Anchor 3 - Filtered distance = {}", uwb3.distance_filtre);

    Ok((uwb1, uwb2, uwb3))
}


fn init() -> (UWBSensor<Spi, OutputPin, hl::Ready>, UWBSensor<Spi, OutputPin, hl::Ready>, UWBSensor<Spi, OutputPin, hl::Ready>) 
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



