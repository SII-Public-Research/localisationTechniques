#![allow(clippy::type_complexity)]

pub mod error;
pub mod experiment_file;
pub mod rtt_ds_algorithms;
pub mod rtt_ss_algorithms;
pub mod sync;
pub mod tools;
pub mod uwb_basics;

pub fn ok_or_panic<T, E>(result: Result<T, E>, panic_info: &str) -> T {
    match result {
        Ok(it) => it,
        Err(_err) => panic!("{}", panic_info),
    }
}

#[macro_export]
macro_rules! scanf {
    ($data_buf:expr, $wlc_msg:literal, $err_msg:literal) => {
        println!($wlc_msg);
        loop {
            let mut buffer = String::new();
            io::stdin()
                .read_line(&mut buffer)
                .expect("Failed to read line");
            match buffer.trim().parse() {
                Ok(num) => {
                    $data_buf = num;
                    break;
                }
                Err(_) => {
                    println!($err_msg);
                    continue;
                }
            };
        }
    };
}
