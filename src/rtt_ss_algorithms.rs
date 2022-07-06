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
    digital::v2::OutputPin,
};



pub fn rtt_ss_inititor <SPI, CS>(
    mut sensor: UWBSensor<SPI, CS, Ready>,
    timeout: OptionTimeout,
) -> Result<UWBSensor<SPI, CS, Ready>, (UWBSensor<SPI, CS, Ready>, Error<SPI, CS>)>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{

    // SENDING FIRST MESSAGE
    println!("STEP 1 : Sending ping...");

    let buffer = [0; 25];
    sensor = sensor.uwb_send(&buffer, None)?; // ???? gestion des erreur


    // RECEIVING T2 AND T3
    println!("STEP 2 : Waiting for an answer...");

    let mut uwb_data_r = UwbRData {
        addr_sender: Some(dw3000::mac::Address::Short(dw3000::mac::PanId(0x111), dw3000::mac::ShortAddress(0x0))),
        data: [0; 6],
        r_time: 0,
    };

    sensor = sensor.uwb_receive(timeout, &mut uwb_data_r)?; // ???? gestion des erreur
    sensor.timing_data[1] = uwb_data_r.data[0];     // Stocke T2
    sensor.timing_data[2] = uwb_data_r.data[1];     // Stocke T3
    sensor.timing_data[3] = uwb_data_r.r_time;      // Stocke T4
    

    // Distance calculation
    println!("STEP 3 : Distance calculation...");

    sensor = calc_distance_simple(sensor)?;         // Calcule de la distance en m
    Ok(sensor)
}


pub fn rtt_ss_responder <SPI, CS>(
    mut sensor: UWBSensor<SPI, OutputPin, Ready>,
    timeout: OptionTimeout,
) -> Result<UWBSensor<SPI, CS, Ready>, (UWBSensor<SPI, CS, Ready>, Error<SPI, CS>)>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: ,
{
    // RECEIVING THE FIRST MESSAGE
    println!("STEP 1 : Waiting ping...");

    let mut uwb_data_r = UwbRData {
        addr_sender: Some(dw3000::mac::Address::Short(dw3000::mac::PanId(0x111), dw3000::mac::ShortAddress(0x0))),
        data: [0; 6],
        r_time: 0,
    };

    sensor = sensor.uwb_receive(timeout, &mut uwb_data_r)?; // ???? gestion des erreur
    sensor.timing_data[0] = uwb_data_r.r_time;   // T2

    // CALCULATE SENDING DELAY 
    let delay_rx_tx = sensor.timing_data[0] + (10000 * 63898) as u64;
    sensor.timing_data[1] = ((delay_rx_tx >> 9) << 9) + sensor.ant_delay_tx ;  // T3

    // SENDING T2 AND T3
    println!("STEP 2 : Offset response...");
    
    let mut buffer: [u8; 25] = [0; 25];
    convert_u64_u8(&sensor.timing_data, &mut buffer);
    sensor = sensor.uwb_send(&buffer, Some(delay_rx_tx))?; // ???? gestion des erreur

    Ok(sensor)
}
