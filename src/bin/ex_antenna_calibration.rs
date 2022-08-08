use futures::executor::block_on;

use rppal::gpio::{Gpio, OutputPin};
use rppal::hal::Timer;
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};

use std::{
    io::{self, Write},
    thread,
    time::Duration,
};

use localisationtechniques::{ok_or_panic, rtt_ds_algorithms::*, tools::*, uwb_basics::*};
use localisationtechniques::sync::RpiSync;
use dw3000::{hl,ll,Ready};


fn main()
{
    let (mut uwbsensor, mut sync)=init();

  

    ok_or_panic(uwbsensor.dw3000.ll().otp_cfg().modify(|_, w| w.otp_man(1)), "fail");
    ok_or_panic(uwbsensor.dw3000.ll().otp_addr().modify(|_, w| w.otp_addr(0x0B)), "fail");
    ok_or_panic(uwbsensor.dw3000.ll().otp_cfg().modify(|_, w| w.otp_read(1)), "fail");
    
    let antenna_delay = ok_or_panic(uwbsensor.dw3000.ll().otp_rdata().read(),"caca").value();
    println!("antenna delay = {}",antenna_delay);

    

}


fn init() -> (UWBSensor<Spi, OutputPin, Ready>,RpiSync)

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
    let mut uwbsensor = ok_or_panic(
        UWBSensor::new(spi, cs),
        "Failed to create an UWBSensor object",
    );

     // SYNC
     let gpio_sync = gpio.get(24).unwrap().into_output();
     let sync = RpiSync::new(gpio_sync);

    (uwbsensor,sync)
}
