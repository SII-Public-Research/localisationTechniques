use std::error::Error;
// includes for time (day + hour)
use chrono::prelude::*;
// include for files
use std::fs::File;
use std::io::prelude::*;

// This function create a file depending on the current date and time
pub fn create_experiment_file() -> Result<File, Box<dyn Error>> {
    let local: DateTime<Local> = Local::now(); // e.g. `2014-11-28T21:45:59.324310806+09:00`
    let local = local.format("%Y-%m-%dT%Hh%M").to_string(); // you cant put a char ':' in this path string, OK CHEF !!
                                                            // otherwise you cant pull the git on windows
    let local = "dataset/".to_string() + &local; //il faut un dossier dataset sinon ca plante instant'
    let file = File::create(local)?;
    Ok(file)
}

// This function write a distance data to the file, with correct format
pub fn write_to_experiment_file(str: &str, file: &mut File) -> Result<(), Box<dyn Error>> {
    let buf = str.as_bytes();
    file.write_all(buf)?;

    Ok(())
}
