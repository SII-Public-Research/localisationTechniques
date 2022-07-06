use rppal::hal::Timer;
use std::time::Duration;




// This is a structure to be used with uwb_basics functions.
#[derive(Copy, Clone, Debug)]
pub struct Timeout
{
    pub timer: Timer,
    pub count: Duration,
}

impl Timeout
{
    pub fn new(count: u64) -> Self {
        Timeout {
            timer: Timer::new(),
            count: Duration::from_millis(count),
        }
    }
}

pub enum OptionTimeout
{
    Some(Timeout),
    None,
}


// convert [u8] to u64, used with receive and send and receive 
pub fn convert_u8_u64(u8_array: &[u8], nb_data: usize) -> u64 {
    let j = nb_data*5;
    ((u8_array[j] as u64) << (8 * 4))
        + ((u8_array[j+1] as u64) << (8 * 3))
        + ((u8_array[j+2] as u64) << (8 * 2))
        + ((u8_array[j+3] as u64) << (8))
        + ((u8_array[j+4] as u64))
}

// // convert u64 to [u8], used with receive and send and receive 
pub fn convert_u64_u8(u64time: &[u64], u8_array: &mut [u8; 25]) {
    for i in 0..5 {
        let j = i*5;
        u8_array[j] = (u64time[i] >>(8 * 4)) as u8;
        u8_array[j+1] = (u64time[i] >> (8 * 3)) as u8;
        u8_array[j+2] = (u64time[i] >> (8 * 2)) as u8;
        u8_array[j+3] = (u64time[i] >> (8 * 1)) as u8;
        u8_array[j+4] = (u64time[i] >> (8 * 0)) as u8;
    }
}


// IIR filtre, to be used on the distances measured by UWBSensor
pub fn filtre_iir<SPI, CS>(
    mut sensor: UWBSensor<SPI, CS, Ready>,
) -> Result<UWBSensor<SPI, CS, Ready>, (UWBSensor<SPI, CS, Ready>, Error<SPI, CS>)>
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
    let a: f64 = 0.96;
    let distance_filtre: f64 = ((1.0 - a) * sensor.distance) + (a * sensor.previous_distance_filtre);
    sensor.previous_distance_filtre = distance_filtre;
    sensor.distance_filtre = distance_filtre;
    Ok(sensor)
}
