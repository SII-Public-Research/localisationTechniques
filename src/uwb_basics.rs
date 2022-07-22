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
// use rppal::gpio::OutputPin as rppalPin;

// PanId : Personal Area Network Identifier, all modules are on the same network
pub const PAN_ID: dw3000::mac::PanId = dw3000::mac::PanId(0x111);

// ShortAddress : address assigned to the module, used to identify the device in the PAN.
pub const ADD_S_ANCH1: dw3000::mac::ShortAddress = dw3000::mac::ShortAddress(0x124);
pub const ADD_S_ANCH2: dw3000::mac::ShortAddress = dw3000::mac::ShortAddress(0x152);
pub const ADD_S_ANCH3: dw3000::mac::ShortAddress = dw3000::mac::ShortAddress(0x282);

// Address : full adress
pub const ADD_ANCHOR1: dw3000::mac::Address = dw3000::mac::Address::Short(PAN_ID, ADD_S_ANCH1);
pub const ADD_ANCHOR2: dw3000::mac::Address = dw3000::mac::Address::Short(PAN_ID, ADD_S_ANCH2);
pub const ADD_ANCHOR3: dw3000::mac::Address = dw3000::mac::Address::Short(PAN_ID, ADD_S_ANCH3);

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
    pub connection: bool,       // indicates if the anchor and tag are connected
    pub timing_data: [u64; 10], // store received timestamps
    pub distance: f64,          // store distance in meters
    pub addr_dest: dw3000::mac::ShortAddress, // store the receiver destinataire
    pub id: u64,                // store the id of the UWB module
    pub ant_delay_tx: u64,      // store TX antenna delay
    pub error: u8,              // store error type
    pub filtered_distance: f64, // store the filtered distance in m (used by IIR filter)
    pub previous_filtered_distance: f64, // store the previous filtered distance in m (used by IIR filter)
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
            distance: -1.0,
            addr_dest: dw3000::mac::ShortAddress(0x0),
            id: 0,
            ant_delay_tx: 16500,
            error: 0,
            filtered_distance: 0.0,
            previous_filtered_distance: 0.0,
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
    ) -> Result<Self, (Self, Error<SPI, CS>)> {
        let mut receiving = ok_or_panic(
            self.dw3000.receive(Config::default()),
            "error when finish receiving, reset must be needed !",
        );

        let mut buffer = [0; 1024]; // buffer to receive message
        let result = match timeout {
            OptionTimeout::Some(mut timeout) => {
                timeout.timer.start(timeout.count); // With timeout
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
            OptionTimeout::None => {
                // Without timeout
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

        uwb_data_r.addr_sender = result.frame.header.source; // Stores the address of the module that sent the data frame in order to check that it is not a message from an unknown module
        uwb_data_r.r_time = result.rx_time.value(); // Stores the timer value when the data frame has been received

        let mut nb_data = 0;
        while nb_data < (result.frame.payload.len() - 2) / 5 {
            // Convert received data (u8) to u64 wrt the payload size
            uwb_data_r.data[nb_data] = convert_u8_u64(result.frame.payload, nb_data);
            nb_data += 1;
        }

        Ok(self)
    }

    // Basic function to send a message. To be used with a send_time which is actually a timestamp at which the message is send
    // WARNING :    send_time has to be well adjusted, so that the timestamp is not passed while the code is executing.
    //              In this case, we will wait for a full clock round to reach the timestamp in question.
    pub fn uwb_send(
        mut self,
        buffer: &[u8],
        send_time: Option<u64>,
    ) -> Result<UWBSensor<SPI, CS, Ready>, (Self, Error<SPI, CS>)> {
        let mut sending = match send_time {
            Some(n) => {
                let sending = match self.dw3000.send(
                    buffer,
                    hl::SendTime::Delayed(Instant::new(n).unwrap()),
                    Config::default(),
                ) {
                    Ok(it) => it,
                    Err(_err) => panic!("error when finish sending, reset must be needed !"),
                };
                sending
            }
            None => {
                let sending = match self
                    .dw3000
                    .send(buffer, hl::SendTime::Now, Config::default())
                {
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
                return Err((self, Error::Dw3000Error(e)));
            }
        }
        .value();

        self.dw3000 = match sending.finish_sending() {
            Ok(it) => it,
            Err(_err) => panic!("error when finish sending, reset must be needed !"),
        };

        Ok(self)
    }
}

// calc_delay_send : Computes the timestamp when the response will be sent
pub fn calc_delay_send<SPI, CS>(
    mut sensor: UWBSensor<SPI, CS, Ready>,
    delay: &mut u64,
) -> Result<UWBSensor<SPI, CS, Ready>, (UWBSensor<SPI, CS, Ready>, Error<SPI, CS>)>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
    // delay = (reception_time + id * 50ms * clock_scale) % max_clcok_value
    *delay = (sensor.timing_data[1] + (sensor.id * 100_000 * 63898)) % 1_0995_1162_7776_u64;
    if sensor.id == 5 {
        sensor.timing_data[4] = ((*delay >> 9) << 9) + sensor.ant_delay_tx;
    } else {
        sensor.timing_data[2] = ((*delay >> 9) << 9) + sensor.ant_delay_tx;
    }

    Ok(sensor)
}

pub fn send_erreur<SPI, CS>(
    mut sensor: UWBSensor<SPI, CS, Ready>,
) -> Result<UWBSensor<SPI, CS, Ready>, (UWBSensor<SPI, CS, Ready>, Error<SPI, CS>)>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
    let buff: [u8; 10] = [255; 10];
    sensor = sensor.uwb_send(&buff, Some(300000 * 63898))?;

    Ok(sensor)
}

// calc_distance_simple : Compute the distance for simple_sided algorithm
pub fn calc_distance_simple<SPI, CS>(
    mut sensor: UWBSensor<SPI, CS, Ready>,
) -> Result<UWBSensor<SPI, CS, Ready>, (UWBSensor<SPI, CS, Ready>, Error<SPI, CS>)>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
    let f: f64 = 1.0 / 499200000.0 / 128.0;
    let s_light: f64 = 299792458.0; // speed of light 10⁹m/s

    let tround: f64 = (sensor.timing_data[3] - sensor.timing_data[0]) as f64;
    let treply: f64 = (sensor.timing_data[2] - sensor.timing_data[1]) as f64;

    let tick_tof: f64 = (tround - treply) / 2_f64;

    let tof: f64 = tick_tof * f;

    sensor.distance = s_light * tof; // distance in meters
    Ok(sensor)
}

// calc_distance_double : Compute the distance for double_sided algorithm
pub fn calc_distance_double<SPI, CS>(
    mut sensor: UWBSensor<SPI, CS, Ready>,
) -> Result<UWBSensor<SPI, CS, Ready>, (UWBSensor<SPI, CS, Ready>, Error<SPI, CS>)>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
    let f: f64 = 1.0 / 499200000.0 / 128.0;
    let s_light: f64 = 299792458.0; // speed of light 10⁹m/s

    let tround1: f64 = (sensor.timing_data[3] - sensor.timing_data[0]) as f64; // T4 - T1
    let treply1: f64 = (sensor.timing_data[2] - sensor.timing_data[1]) as f64; // T3 - T2
    let tround2: f64 = (sensor.timing_data[5] - sensor.timing_data[2]) as f64; // T6 - T3
    let treply2: f64 = (sensor.timing_data[4] - sensor.timing_data[3]) as f64; // T5 - T4

    let tof_tick: f64 = (((tround1 as u128 * tround2 as u128)
        - (treply1 as u128 * treply2 as u128))
        / ((tround1 + treply1 + tround2 + treply2) as u128)) as f64;
    let tof_sec: f64 = tof_tick * f;

    let distance = tof_sec * s_light;

    // protection against wrong values
    // e.g. One of the clocks ends its cycle during a measurement
    if distance < 10000.0 {
        sensor.distance = distance;
    }

    Ok(sensor)
}

// iir_filter : to be used on the distances measured by UWBSensor
pub fn iir_filter<SPI, CS>(
    mut sensor: UWBSensor<SPI, CS, Ready>,
) -> Result<UWBSensor<SPI, CS, Ready>, (UWBSensor<SPI, CS, Ready>, Error<SPI, CS>)>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
    let a: f64 = 0.96; // constraint coefficient: 0 no filter, 1 total restriction
    let filtered_distance: f64 =
        ((1.0 - a) * sensor.distance) + (a * sensor.previous_filtered_distance);
    sensor.previous_filtered_distance = filtered_distance;
    sensor.filtered_distance = filtered_distance;
    Ok(sensor)
}
