
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use structopt::StructOpt;
use simplelog::{LevelFilter, SimpleLogger, TermLogger, TerminalMode};
use log::{info, debug};

use sensor_tlv493d::Tlv493d;
use linux_embedded_hal::I2cdev;

/// Connect to a TLV493D device and read out values
#[derive(PartialEq, Clone, Debug, StructOpt)]
pub struct Options {

    #[structopt(long = "log-level", default_value = "/dev/i2c-1")]
    /// Linux I2C device
    pub i2c_dev: String,

    #[structopt(long = "log-level", default_value = "ADDRESS_BASE")]
    /// Device I2C address    
    pub i2c_addr: u8,


    #[structopt(long = "log-level", default_value = "info")]
    /// Configure app logging levels (warn, info, debug, trace)
    pub log_level: LevelFilter,
}


fn main() -> Result<(), Box<dyn std::error::Error>> {
    let running = Arc::new(AtomicBool::new(true));

    // Load options
    let opts = Options::from_args();

    // Setup logging
    let log_config = simplelog::ConfigBuilder::new().build();
    if let Err(_e) = TermLogger::init(opts.log_level, log_config.clone(), TerminalMode::Mixed) {
        SimpleLogger::init(opts.log_level, log_config).unwrap();
    }
 
    debug!("Connecting to I2C device: {}", opts.i2c_dev);

    // Connect to I2C
    let i2c = I2cdev::new(opts.i2c_dev)?;

    debug!("Connecting to sensor: 0x{:02x}", opts.i2c_addr);

    // Create sensor
    let mut sensor = Tlv493d::new(i2c, opts.i2c_addr)?;

    // Setup exit handler
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");


    debug!("Running");

    // Read from sensor
    while running.load(Ordering::SeqCst) {
        
        let v = sensor.read()?;

        info!("{:?}", v);
    }

    Ok(())
}
