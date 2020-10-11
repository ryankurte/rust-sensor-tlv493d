
#![no_std]

/// Rust tlv493d 3-DoF I2C Hall Effect Sensor Driver
///
/// Copyright 2020 Ryan Kurte
///
/// - https://www.infineon.com/dgdl/Infineon-TLV493D-A1B6-DataSheet-v01_10-EN.pdf?fileId=5546d462525dbac40152a6b85c760e80
/// - https://www.infineon.com/dgdl/Infineon-TLV493D-A1B6_3DMagnetic-UM-v01_03-EN.pdf?fileId=5546d46261d5e6820161e75721903ddd


use core::marker::PhantomData;

use log::{debug};
use bitflags::bitflags;
use embedded_hal::blocking::i2c;

#[cfg(feature = "std")]
extern crate std;

pub struct Tlv493d<I2c, E> {
    i2c: I2c,
    addr: u8,
    _e: PhantomData<E>,
}

/// Base address for Tlv493d, bottom bit set during power up
/// based on value of SDA.
pub const ADDRESS_BASE: u8 = 0b1011110;

/// Read registers for the Tlv493d
pub enum ReadRegisters {
    Rx      = 0x00, // X direction flux (Bx[11..4])
    By      = 0x01, // Y direction flux (By[11..4])
    Bz      = 0x02, // X direction flux (Bz[11..4])
    Temp    = 0x03, // Temperature high bits, frame counter, channel, (Temp[11..8] | FRM[1..0] | CH[1..0])
    Bx2     = 0x04, // Lower X and Y flux (Bx[3..0] | Bx[3..0])
    Bz2     = 0x05, // Flags + Lower Z flux (Reserved | T | FF | PD | Bz[3..0])
    Temp2   = 0x06, // Temperature low bits (Temp[7..0])
    
    FactSet1 = 0x07,
    FactSet2 = 0x08,
    FactSet3 = 0x09,
}

/// Write registers for the Tlv493d
pub enum WriteRegisters {
    Res     = 0x00, // Reserved
    Mode1   = 0x01, // Mode 1 register (P | IICAddr[1..0] | Reserved[1..0] | INT | FAST | LOW)
    Res2    = 0x02, // Reserved
    Mode2   = 0x03, // Mode 2 register (T | LP | PT | Reserved[4..0])
}

/// TLV493D Measurement values
#[derive(Debug, PartialEq, Clone)]
pub struct Values {
    x: f32,     // X axis magnetic flux (mT)
    y: f32,     // Y axis magnetic flux (mT)
    z: f32,     // Z axis magnetic flux (mT)
    temp: f32,  // Device temperature (C)
}

bitflags! {
    /// Device Mode1 register
    pub struct Mode1: u8 {
        const PARITY = 0b1000_0000;     // Parity of configuration map, must be calculated prior to write command
        const I2C_ADDR_1 = 0b0100_0000;     // Set I2C address top bit in bus configuration
        const I2C_ADDR_0 = 0b0010_0000;     // Set I2C address bottom bit in bus configuration
        const IRQ_EN     = 0b0000_0100;      // Enable read-complete interrupts
        const FAST       = 0b0000_0010;      // Enable fast mode (must be disabled for power-down)
        const LOW        = 0b0000_0001;      // Low power mode
    }
}

bitflags! {
    /// Device Mode2 register
    pub struct Mode2: u8 {
        const TEMP_DISABLE   = 0b1000_0000;     // DISABLE temperature measurement
        const LOW_POW_PERIOD = 0b0100_0000;     // Set low power period ("0": 100ms, "1": 12ms)
        const PARITY_TEST_EN = 0b0010_0000;     // Enable / Disable parity test
    }
}


/// Tlv493d related errors
#[derive(Debug)]
#[cfg_attr(feature = "std", derive(thiserror::Error))] 
pub enum Error<E: core::fmt::Debug> {
    // No device found with specified i2c bus and address
    #[cfg_attr(feature = "std", error("No device found with specified i2c bus and address"))] 
    NoDevice,

    // Underlying I2C device error
    #[cfg_attr(feature = "std", error("Underlying I2C device error"))] 
    I2c(E),
}

impl <I2c, E> Tlv493d<I2c, E>
where 
    I2c: i2c::Read<Error = E> + i2c::Write<Error = E> + i2c::WriteRead<Error = E>,
    E: core::fmt::Debug,
{
    /// Create a new TLV493D instance
    pub fn new(i2c: I2c, addr: u8) -> Result<Self, Error<E>> {
        debug!("New Tlv493d with address: {}", addr);
        
        // Construct object
        let mut s = Self {
            i2c, addr, _e: PhantomData,
        };
       
        // Test read from device
        let mut b = [0u8; 1];
        let _ = s.i2c.read(s.addr, &mut b[..]).map_err(Error::I2c)?;

        // Return object
        Ok(s)
    }

    /// Read raw values from the sensor
    pub fn read_raw(&mut self) -> Result<[i16; 4], Error<E>> {
        let mut v = [0i16; 4];

        // Read data from device
        let mut b = [0u8; 7];
        self.i2c.write_read(self.addr, &[0x00], &mut b[..]).map_err(Error::I2c)?;
        
        // Convert to values
        v[0] = (b[0] as i16) << 8 | ((b[4] & 0xF0) >> 4) as i16;
        v[1] = (b[1] as i16) << 8 | ((b[4] & 0x0F)) as i16;
        v[2] = (b[2] as i16) << 8 | ((b[5] & 0x0F)) as i16;
        v[3] = ((b[3] & 0x00F0) as i16) << 8 | (b[6] as i16 & 0xFF);

        debug!("Read data {:02x?} values: {:?}", b, v);

        Ok(v)
    }

    /// Read and convert values from the sensor
    pub fn read(&mut self) -> Result<Values, Error<E>> {
        let raw = self.read_raw()?;

        Ok(Values {
            x: raw[0] as f32 * 0.098f32,
            y: raw[1] as f32 * 0.098f32,
            z: raw[2] as f32 * 0.098f32,
            temp: raw[3] as f32 * 1.1f32,
        })
    }
}

