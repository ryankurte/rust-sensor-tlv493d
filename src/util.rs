
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use structopt::StructOpt;
use simplelog::{LevelFilter, SimpleLogger, TermLogger, TerminalMode};
use log::{info, debug};

use sensor_tlv493d::{Tlv493d, Mode};
use linux_embedded_hal::I2cdev;


/// Connect to a TLV493D device and read out values
#[derive(PartialEq, Clone, Debug, StructOpt)]
pub struct Options {

    #[structopt(long, default_value = "/dev/i2c-1")]
    /// Linux I2C device
    pub i2c_dev: String,

    #[structopt(long, default_value = "400")]
    /// I2C device baud in kHz
    pub i2c_baud_khz: u32,

    #[structopt(long, parse(try_from_str=u8_from_hex), default_value = "5e")]
    /// Device I2C address in hex 
    pub i2c_addr: u8,

    #[structopt(long, default_value = "info")]
    /// Configure app logging levels (warn, info, debug, trace)
    pub log_level: LevelFilter,
}

fn u8_from_hex(s: &str) -> Result<u8, std::num::ParseIntError> {
    u8::from_str_radix(s, 16)
}

fn main() -> Result<(), anyhow::Error> {
    let running = Arc::new(AtomicBool::new(true));

    // Load options
    let opts = Options::from_args();

    // Setup logging
    let log_config = simplelog::ConfigBuilder::new().build();
    if let Err(_e) = TermLogger::init(opts.log_level, log_config.clone(), TerminalMode::Mixed) {
        SimpleLogger::init(opts.log_level, log_config).unwrap();
    }
    
    info!("Connecting to sensor: 0x{:02x} on bus: {}", opts.i2c_addr, opts.i2c_dev);

    // Connect to I2C
    let i2c = I2cdev::new(opts.i2c_dev)?;

    // Create sensor
    let mut sensor = Tlv493d::new(i2c, opts.i2c_addr, Mode::Master)?;

    // Setup exit handler
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");

    debug!("Running");

    // Read from sensor
    while running.load(Ordering::SeqCst) {
        
        let v = sensor.read()?;

        info!("{:03.04?}", v);

        std::thread::sleep(Duration::from_secs(1));
    }

    // Disable sensor before exit
    sensor.configure(Mode::Disabled)?;

    Ok(())
}
