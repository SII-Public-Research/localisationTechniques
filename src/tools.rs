use rppal::hal::Timer;
use std::time::Duration;

// This is a structure to be used with uwb_basics functions.
#[derive(Copy, Clone, Debug)]
pub struct Timeout
{
    pub timer: Timer, // millis
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
#[derive(Copy, Clone, Debug)]
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
