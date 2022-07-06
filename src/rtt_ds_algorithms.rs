use crate::{
    error::Error,
    tools::*,
    uwb_basics::*
};

use dw3000::{
    block,
    hl::{self, DW3000},
    time::Instant,
    Config, Ready, Uninitialized,
};

use embedded_hal::{
    blocking::spi::{Transfer, Write},
    timer::CountDown,
};
use embedded_timeout_macros::block_timeout;

use rppal::gpio::{Gpio, OutputPin};
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};
use rppal::hal::Timer;
use std::time::Duration;


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


pub fn rtt_ds_responder<TIM, X, SPI, CS>(
    mut sensor: UWBSensor<SPI, CS, Ready>,
    timer: Option<Timeout<TIM, X>>,
    ant_delay: u64,
) -> Result<UWBSensor<SPI, CS, Ready>, (UWBSensor<SPI, CS, Ready>, Error<SPI, CS>)>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
    TIM: CountDown + Copy,
    X: Into<TIM::Time> + Copy,
{
    sensor.id = 5;

    let mut data_anch1 = UwbRData {
        addr_sender: Some(dw3000::mac::Address::Short(
            dw3000::mac::PanId(0x111),
            dw3000::mac::ShortAddress(0x0),
        )),
        data: [0; 6],
        r_time: 0,
    };

    println!("STEP 1 : Waiting ping...");
    sensor = match sensor.uwb_receive(Timeout::none(), &mut data_anch1) {
        Ok(sensor) => sensor,
        Err((sensor, e)) => {
            println!("Erreur");
            return Err((sensor, e));
        }
    };

    /*****************************************************************/
    /* Initialization of the measurement, sending of the first frame */
    /*****************************************************************/
    println!("STEP 2 : Requesting new measurement...");

    let mut buff: [u8; 25] = [0; 25];
    sensor = sensor.uwb_send(&buff, None)?; // T1 is stored at the same time
    
    let mut uwb_data_r = UwbRData {
        addr_sender: Some(dw3000::mac::Address::Short(dw3000::mac::PanId(0x111), dw3000::mac::ShortAddress(0x0))),
        data: [0; 6],
        r_time: 0,
    };


    /******************************************/
    /* Waiting for reception on the 3 modules */
    /******************************************/
    println!("STEP 3 : Wainting answer for the anchor...");

    sensor = sensor.uwb_receive(timer, &mut uwb_data_r)?;
    if uwb_data_r.data[0] == 1099511627775 {
        // 2^40 - 1
        println!("Error on Anchor !");
        return Err((sensor, Error::ModuleAnchor));
    } else {
        match uwb_data_r.addr_sender {
            Some(ADD_ANCHOR1) => {
                sensor.timing_data[1] = uwb_data_r.r_time; // T4 anchor 1
            }
            _ => {
                println!("Receiving from unknown anchor !");
                sensor = send_erreur(sensor)?;
                return Err((sensor, Error::ModuleInconnu));
            }
        };
    }
    
    /*******************************************/
    /* Offset module responses for each module */
    /*******************************************/
    println!("STEP 4 : Final offset response...");

    let delay_rx_tx = (sensor.timing_data[1] + (15000 * 63898)) % 1_0995_1162_7776 as u64;
    sensor.timing_data[4] = ((delay_rx_tx >> 9) << 9) + ant_delay ;
    
    convert_u64_u8(&sensor.timing_data, &mut buff);
    sensor = sensor.uwb_send(&buff, Some(delay_rx_tx))?;

    println!("Exchange procedure terminated on Tag side");
    Ok(sensor)
}


async pub fn rtt_ds_initiator_3(
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


pub fn rtt_ds_responder_3<TIM, X, SPI, CS>(
    mut sensor: UWBSensor<SPI, CS, Ready>,
    timer: Option<Timeout<TIM, X>>,
    ant_delay: u64,
) -> Result<UWBSensor<SPI, CS, Ready>, (UWBSensor<SPI, CS, Ready>, Error<SPI, CS>)>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
    TIM: CountDown + Copy,
    X: Into<TIM::Time> + Copy,
{
    sensor.id = 5;
    let mut nb_anchor = 0;

    let mut data_anch1 = UwbRData {
        addr_sender: Some(dw3000::mac::Address::Short(
            dw3000::mac::PanId(0x111),
            dw3000::mac::ShortAddress(0x0),
        )),
        data: [0; 6],
        r_time: 0,
    };

    println!("STEP 1 : Waiting ping...");
    sensor = match sensor.uwb_receive(Timeout::none(), &mut data_anch1) {
        Ok(t) => t,
        Err((t, e)) => {
            println!("Erreur");
            return Err((t, e));
        }
    };

    // Waiting for the other module tu run the 3 receiving threads
    thread::sleep(Duration::from_millis(20));

    /*****************************************************************/
    /* Initialization of the measurement, sending of the first frame */
    /*****************************************************************/
    println!("STEP 2 : Requesting new measurement...");

    let mut buff: [u8; 25] = [0; 25];
    sensor = sensor.uwb_send(&buff, None)?; // T1 is stored at the same time

    let mut uwb_data_r = UwbRData {         // structure dans laquelle sont stockées les données de la fonction uwb_reception
        addr_sender: Some(dw3000::mac::Address::Short(dw3000::mac::PanId(0x111), dw3000::mac::ShortAddress(0x0))),
        data: [0; 6],
        r_time: 0,
    };


    /******************************************/
    /* Waiting for reception on the 3 modules */
    /******************************************/
    println!("STEP 3 : Wainting answer for each anchor...");

    while nb_anchor < 7 { // 3 bits for 3 anchors flag
        sensor = sensor.uwb_receive(timer, &mut uwb_data_r)?;

        // Checking the content before assigning the value to the correct timestamp
        if uwb_data_r.data[0] == 1099511627775 { // 2^40 - 1
            println!("Error on Anchor !");
            return Err((sensor, Error::ModuleAnchor));
        } else {
            match uwb_data_r.addr_sender {
                Some(ADD_ANCHOR1) => {
                    sensor.timing_data[1] = uwb_data_r.r_time; // T4 anchor 1
                    nb_anchor = nb_anchor + 1; // bit 1 -> first flag
                }
                Some(ADD_ANCHOR2) => {
                    sensor.timing_data[2] = uwb_data_r.r_time; // T4 anchor 2
                    nb_anchor = nb_anchor + 2; // bit 2 -> second flag
                }
                Some(ADD_ANCHOR3) => {
                    sensor.timing_data[3] = uwb_data_r.r_time; // T4 anchor 3
                    nb_anchor = nb_anchor + 4; // bit 3 -> third flag
                }
                _ => {
                    println!("Receiving from unknown anchor !");
                    sensor = send_erreur(sensor)?;
                    return Err((sensor, Error::ModuleInconnu));
                }
            };
        }
    }

    
    /*******************************************/
    /* Offset module responses for each module */
    /*******************************************/
    println!("STEP 4 : Final offset response...");

    let delay_rx_tx = (sensor.timing_data[3] + (100000 * 63898)) % 1_0995_1162_7776 as u64;
    sensor.timing_data[4] = ((delay_rx_tx >> 9) << 9) + ant_delay ;
    
    convert_u64_u8(&sensor.timing_data, &mut buff);
    sensor = sensor.uwb_send(&buff, Some(delay_rx_tx))?;
    
    println!("Exchange procedure terminated on Tag side");
    Ok(sensor)
}