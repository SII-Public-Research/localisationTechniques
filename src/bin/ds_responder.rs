// `block_on` blocks the current thread until the provided future has run to completion. Other executors provide more complex behavior, like scheduling
// multiple futures onto the same thread.
use futures::executor::block_on;

use rppal::gpio::{Gpio, OutputPin as rppalOutputPin};
use rppal::hal::Timer;
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};

use std::io::{self, Write as stdWrite};
use std::time::Duration;

use raspberry_client::uwb_sensor::*;
use raspberry_client::{error::Error, ok_or_panic};

use dw3000::{hl, Ready};

use embedded_hal::{
    blocking::spi::{Transfer, Write},
    digital::v2::OutputPin,
    timer::CountDown,
};

async fn async_main() {
    let mut uwbsensor = init();
    loop {
        println!("\nNew measure");
        let _timer = Timer::new();
        uwbsensor = match rtt_ds_responder(uwbsensor, Timeout::new(_timer, Duration::from_millis(5000)), 16385) {
            Ok(sensor) => {
                print!("OK");
                io::stdout().flush().unwrap();
                sensor
            }
            Err((mut sensor, _e)) => {
                sensor.distance = 100_000.0;
                print!("KO");
                io::stdout().flush().unwrap();
                sensor
            }
        };
    }
}

fn main() {
    block_on(async_main());
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

fn init() -> UWBSensor<Spi, rppalOutputPin, hl::Ready> {
    /******************************************************* */
    /************        BASIC CONFIGURATION      ********** */
    /******************************************************* */

    let spi = Spi::new(Bus::Spi1, SlaveSelect::Ss0, 4_500_000, Mode::Mode0)
        .expect("Failed to configure the spi");
    let gpio = Gpio::new().expect("Failed to configure GPIO");
    let cs = gpio.get(16).expect("Failed to set up CS PIN").into_output();

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
    //uwbsensor.dw3000.set_antenna_delay(4416, 16500).expect("Fail set antenna delay");
    println!("Init OK");
    uwbsensor
}
