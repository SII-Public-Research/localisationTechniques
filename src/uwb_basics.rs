use crate::{error::Error, tools::*, *};
use dw3000::{
    block,
    hl::{self, DW3000},
    time::Instant,
    Config, Ready, Uninitialized,
};

use embedded_hal::{
    blocking::spi::{Transfer, Write},
    digital::v2::OutputPin,
    timer::CountDown,
};
use embedded_timeout_macros::block_timeout;



// ajouter une decription de ce qu'est PanId et ShortAddress
pub const PAN_ID : dw3000::mac::PanId = dw3000::mac::PanId(0x111);

pub const ADD_S_ANCH1: dw3000::mac::ShortAddress = dw3000::mac::ShortAddress(0x124);
pub const ADD_S_ANCH2: dw3000::mac::ShortAddress = dw3000::mac::ShortAddress(0x152);
pub const ADD_S_ANCH3: dw3000::mac::ShortAddress = dw3000::mac::ShortAddress(0x282);

pub const ADD_ANCHOR1: dw3000::mac::Address = dw3000::mac::Address::Short(dw3000::mac::PanId(0x111), dw3000::mac::ShortAddress(0x124));
pub const ADD_ANCHOR2: dw3000::mac::Address = dw3000::mac::Address::Short(dw3000::mac::PanId(0x111), dw3000::mac::ShortAddress(0x152));
pub const ADD_ANCHOR3: dw3000::mac::Address = dw3000::mac::Address::Short(dw3000::mac::PanId(0x111), dw3000::mac::ShortAddress(0x282));

pub const ADD_ERROR: dw3000::mac::Address = dw3000::mac::Address::Short(dw3000::mac::PanId(0x0), dw3000::mac::ShortAddress(0x0));

// Structure to store specifics data from the Message received (See Message<'l> struct on DW3000 crate)
#[derive(Copy, Clone, Debug)]
pub struct UwbRData {
    pub addr_sender: Option<dw3000::mac::Address>,
    pub data: [u64; 6],
    pub r_time: u64,
}


#[derive(Copy, Clone, Debug)]
pub struct UWBSensor<SPI, CS, STATE> {
    pub dw3000: DW3000<SPI, CS, STATE>,
    pub connection: bool,                       // indique si le anchor et la tag sont connectés
    pub timing_data: [u64; 10],                 // stocke les temps envoyés
    pub distance: f64,                          // stocke la distance en m
    pub addr_dest: dw3000::mac::ShortAddress,   // stocke l'adresse du destinataire
    pub id: u64,                                // stocke id du module UWB
    pub ant_delay_tx: u64,                      // stocke le delai de l'antenne TX
    pub error: u8,                              // stocke le type d'erreur
    pub distance_filtre: f64,                   // stocke la valeur de la distance filtré en m (utilisé par le filtre IIR)          
    pub previous_distance_filtre: f64,          // stocke la valeur de la distance précédente filtré en m (utilisé par le filtre IIR)
}


impl<SPI, CS> UWBSensor<SPI, CS, Uninitialized>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
    pub fn new(spi: SPI, cs: CS) -> Result<UWBSensor<SPI, CS, Ready>, dw3000::Error<SPI, CS>> {
        let uwb_sensor = UWBSensor {
            dw3000: DW3000::new(spi, cs).init()?.config(Config::default())?,
            connection: false,
            timing_data: [0; 10],
            distance: -1,
            addr_dest: dw3000::mac::ShortAddress(0x0),
            id: 0,
            ant_delay_tx: 16500,
            error: 0, 
            distance_filtre: 0.0,
            previous_distance_filtre: 0.0, 
        };

        Ok(uwb_sensor)
    }
}

impl<SPI, CS> UWBSensor<SPI, CS, Ready>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
    // Basic function to wait for a reception. To be used with or without a Timeout (no timeout = wait forever)
    pub fn uwb_receive(
        mut self,
        timeout: OptionTimeout,
        uwb_data_r: &mut UwbRData,
    ) -> Result<Self, (Self, Error<SPI, CS>)>
    {
        let mut receiving = ok_or_panic(
            self.dw3000.receive(Config::default()),
            "error when finish receiving, reset must be needed !",
        );

        let mut buffer = [0; 1024]; // buffer to receive message
        let result = match timeout {
            OptionTimeout::Some(mut timeout) => {   
                timeout.timer.start(timeout.count);                                                     // With timeout
                match block_timeout!(&mut timeout.timer, receiving.r_wait(&mut buffer)) {
                    Ok(t) => t,
                    Err(e) => {
                        self.dw3000 = ok_or_panic(
                            receiving.finish_receiving(),
                            "error when finish receiving, reset must be needed !",
                        );
                        return Err((self, Error::TimeoutError(e)));
                    }
                }
            }
            OptionTimeout::None => {                                                                   // Without timeout
                match block!(receiving.r_wait(&mut buffer)) {                          
                    Ok(t) => t,
                    Err(e) => {
                        self.dw3000 = ok_or_panic(
                            receiving.finish_receiving(),
                            "error when finish receiving, reset must be needed !",
                        );
                        return Err((self, Error::Dw3000Error(e)));
                    }
                }
            }
        };

        self.dw3000 = ok_or_panic(
            receiving.finish_receiving(),
            "error when finish receiving, reset must be needed !",
        );

        uwb_data_r.addr_sender = result.frame.header.source;        // Stocke l'addresse du module ayant envoyé la trame de données pour pouvoir vérifier qu'il ne s'agit pas d'un message d'un module inconnu
        uwb_data_r.r_time = result.rx_time.value();                 // Stocke la valeur du timer lorsque la trame de données a été reçue
        
        let mut nb_data = 0;
        while nb_data <  (result.frame.payload.len() - 2) / 5 {     // Converti les données reçues (u8) en u64 quelque soit la taille de payload
            uwb_data_r.data[nb_data] = convert_u8_u64(result.frame.payload, nb_data);
            nb_data = nb_data + 1;
        }
        
        Ok(self)
    }


    // Basic function to send a message. To be used with or without a Delay (no timeout = wait forever)
    pub fn uwb_send(
        mut self, 
        buffer: &[u8],
        delay: Option<u64>,
    ) -> Result<UWBSensor<SPI, CS, Ready>, (Self, Error<SPI, CS>)> 
    {
        let mut sending = match delay {
            Some(n) => { 
                let sending = match self
                .dw3000
                .send(buffer, hl::SendTime::Delayed(Instant::new(n).unwrap()), Config::default()) {
                    Ok(it) => it,
                    Err(_err) => panic!("error when finish sending, reset must be needed !"),
                };
                sending
            }
            None => {
                let sending = match self
                    .dw3000
                    .send(buffer, hl::SendTime::Now, Config::default()) {
                Ok(it) => it,
                Err(_err) => panic!("error when finish sending, reset must be needed !"),
                };
                sending
            }
        };

        if ok_or_panic(sending.ll().sys_status().read(), "testfail").hpdwarn() == 1 {
            println!("HPDWARN Error")
        }

        self.timing_data[0] = match block!(sending.s_wait()) {
            Ok(t) => t,
            Err(e) => {
                println!("s_wait Error !");
                self.dw3000 = match sending.finish_sending() {
                    Ok(it) => it,
                    Err(_err) => panic!("error when finish sending, reset must be needed !"),
                };
                return Err((self, Error::Dw3000Error(e)))
            },
        }.value();
        
        self.dw3000 = match sending.finish_sending() {
            Ok(it) => it,
            Err(_err) => panic!("error when finish sending, reset must be needed !"),
        };


        Ok(self)
    }

}

















///////////   FONCTION DOUBLE SIDED TARGET WITH 3 ANCHORS /////////////

pub fn calc_delay_send <SPI, CS> (mut sensor: UWBSensor<SPI, CS, Ready>, delay: &mut u64) 
    -> Result<UWBSensor<SPI, CS, Ready>, (UWBSensor<SPI, CS, Ready>, Error<SPI, CS>)> 
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
    *delay = (sensor.timing_data[1] + (sensor.id * 50000 * 63898)) % 1_0995_1162_7776 as u64;
    //println!("delay send = {}", sensor.id * 100000 * 63898);  //us //GHz
    if sensor.id == 5 {
        sensor.timing_data[4] = ((*delay >> 9) << 9) + sensor.ant_delay_tx;
    }
    else {
        sensor.timing_data[2] = ((*delay >> 9) << 9) + sensor.ant_delay_tx;
        //println!("T3 estimé = {}", sensor.timing_data[2]);
    }
    //sensor.timing_data[4] =  sensor.timing_data[4] << 9 + 16348; 

    Ok(sensor)
} 

pub fn send_erreur<SPI,CS>(mut sensor:UWBSensor<SPI,CS,Ready>)
 -> Result<UWBSensor<SPI, CS, Ready>, (UWBSensor<SPI, CS, Ready>, Error<SPI, CS>)> 
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{

    let buff: [u8;10] = [255;10];
    sensor = sensor.uwb_send(&buff, Some(300000*63898))?;

    Ok(sensor)
}



/******************************************************* */
/************      FONCTION - SIMPLE SIDED    ********** */
/******************************************************* */

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
    mut sensor: UWBSensor<SPI, CS, Ready>,
    timeout: OptionTimeout,
) -> Result<UWBSensor<SPI, CS, Ready>, (UWBSensor<SPI, CS, Ready>, Error<SPI, CS>)>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
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

pub fn calc_distance_simple <SPI, CS> (mut sensor: UWBSensor<SPI, CS, Ready>) 
    -> Result<UWBSensor<SPI, CS, Ready>, (UWBSensor<SPI, CS, Ready>, Error<SPI, CS>)> 
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
	let f: f64 = 1.0/499200000.0/128.0;
    let s_light: f64 = 299792458.0; // vitesse de la lumière 10⁹m/s

	// let clockoffset: f32 = ll().cia_diag_0().read().coe_ppm() as f32/(1<<26) as f32;
    let tround: f64 = (sensor.timing_data[3] - sensor.timing_data[0]) as f64;
    let treply: f64 = (sensor.timing_data[2] - sensor.timing_data[1]) as f64;

	// let tick_tof = (tround-treply*(1.0-clockoffset)) / 2 as f32;
	let tick_tof: f64 = (tround-treply) / 2 as f64;

	let tof: f64 = tick_tof * f ;

	sensor.distance = s_light * tof; // calcul pour obtenir une distance en m  
    Ok(sensor)
}


pub fn calc_distance_double <SPI, CS> (mut sensor: UWBSensor<SPI, CS, Ready>) 
    -> Result<UWBSensor<SPI, CS, Ready>, (UWBSensor<SPI, CS, Ready>, Error<SPI, CS>)> 
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
    let f: f64 = 1.0/499200000.0/128.0;
    let s_light: f64 = 299792458.0; // speed of light m/s

    let tround1: f64 = (sensor.timing_data[3] - sensor.timing_data[0]) as f64;  // T4-T1
    let treply1: f64 = (sensor.timing_data[2] - sensor.timing_data[1]) as f64;  // T3-T2
    let tround2: f64 = (sensor.timing_data[5] - sensor.timing_data[2]) as f64;  // T6-T3
    let treply2: f64 = (sensor.timing_data[4] - sensor.timing_data[3]) as f64;  // T5-T4

    let tick_tof: f64 = (((tround1 as u128 * tround2 as u128) - (treply1 as u128 * treply2 as u128 )) / ((tround1 + treply1 + tround2 + treply2) as u128)) as f64;
    let tof_sec: f64 = tick_tof * f;

    let distance = tof_sec * s_light;
    if distance < 10000.0 { // protection contre les fausse valeures
        sensor.distance = distance;
    } else {
        sensor.distance = sensor.distance;
    }
    
    Ok(sensor)
}
