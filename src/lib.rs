#![no_std]

/// Rust tlv493d 3-DoF I2C Hall Effect Sensor Driver
///
/// Copyright 2020 Ryan Kurte
///
/// - https://www.infineon.com/dgdl/Infineon-TLV493D-A1B6-DataSheet-v01_10-EN.pdf?fileId=5546d462525dbac40152a6b85c760e80
/// - https://www.infineon.com/dgdl/Infineon-TLV493D-A1B6_3DMagnetic-UM-v01_03-EN.pdf?fileId=5546d46261d5e6820161e75721903ddd
use core::fmt::Debug;
use core::marker::PhantomData;

use bitflags::bitflags;
use embedded_hal::blocking::{delay, i2c};
use log::debug;

#[cfg(feature = "std")]
extern crate std;


pub struct Tlv493d<I2c, I2cErr, Delay, DelayErr> {
    i2c: I2c,
    delay: Delay,
    addr: u8,
    initial: [u8; 10],
    last_frm: u8,
    _e_i2c: PhantomData<I2cErr>,
    _e_delay: PhantomData<DelayErr>,
}

/// Base address for Tlv493d, bottom bit set during power up
/// based on value of SDA.
pub const ADDRESS_BASE: u8 = 0b1011110;

/// Read registers for the Tlv493d
pub enum ReadRegisters {
    Rx = 0x00,    // X direction flux (Bx[11..4])
    By = 0x01,    // Y direction flux (By[11..4])
    Bz = 0x02,    // X direction flux (Bz[11..4])
    Temp = 0x03, // Temperature high bits, frame counter, channel, (Temp[11..8] | FRM[1..0] | CH[1..0])
    Bx2 = 0x04,  // Lower X and Y flux (Bx[3..0] | Bx[3..0])
    Bz2 = 0x05,  // Flags + Lower Z flux (Reserved | T | FF | PD | Bz[3..0])
    Temp2 = 0x06, // Temperature low bits (Temp[7..0])

    FactSet1 = 0x07,
    FactSet2 = 0x08,
    FactSet3 = 0x09,
}

/// Write registers for the Tlv493d
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WriteRegisters {
    Res = 0x00,   // Reserved
    Mode1 = 0x01, // Mode 1 register (P | IICAddr[1..0] | Reserved[1..0] | INT | FAST | LOW)
    Res2 = 0x02,  // Reserved
    Mode2 = 0x03, // Mode 2 register (T | LP | PT | Reserved[4..0])
}

/// TLV493D Measurement values
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Values {
    pub x: f32,    // X axis magnetic flux (mT)
    pub y: f32,    // Y axis magnetic flux (mT)
    pub z: f32,    // Z axis magnetic flux (mT)
    pub temp: f32, // Device temperature (C)
}

/// Device operating mode
/// Note that in most cases the mode is a combination of mode and IRQ flags
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mode {
    Disabled,      // Reading disabled
    Master,        // Master initiated mode (reading occurs after readout)
    Fast,          // Fast mode (3.3kHz)
    LowPower,      // Low power mode (100Hz)
    UltraLowPower, // Ultra low power mode (10Hz)
}

bitflags! {
    /// Device Mode1 register
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Mode1: u8 {
        const PARITY     = 0b1000_0000;     // Parity of configuration map, must be calculated prior to write command
        const I2C_ADDR_1 = 0b0100_0000;     // Set I2C address top bit in bus configuration
        const I2C_ADDR_0 = 0b0010_0000;     // Set I2C address bottom bit in bus configuration
        const IRQ_EN     = 0b0000_0100;      // Enable read-complete interrupts
        const FAST       = 0b0000_0010;      // Enable fast mode (must be disabled for power-down)
        const LOW        = 0b0000_0001;      // Low power mode
    }
}

bitflags! {
    /// Device Mode2 register
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Mode2: u8 {
        const TEMP_DISABLE   = 0b1000_0000;     // DISABLE temperature measurement
        const LOW_POW_PERIOD = 0b0100_0000;     // Set low power period ("0": 100ms, "1": 12ms)
        const PARITY_TEST_EN = 0b0010_0000;     // Enable / Disable parity test
    }
}

/// Tlv493d related errors
#[derive(Debug)]
#[cfg_attr(feature = "std", derive(thiserror::Error))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<I2cErr: Debug, DelayErr: Debug> {
    // No device found with specified i2c bus and address
    #[cfg_attr(
        feature = "std",
        error("No device found with specified i2c bus and address")
    )]
    NoDevice,

    // Device ADC locked up and must be reset
    #[cfg_attr(feature = "std", error("Device ADC lockup, reset required"))]
    AdcLockup,

    // Underlying I2C device error
    #[cfg_attr(feature = "std", error("I2C device error: {0:?}"))]
    I2c(I2cErr),

    // Underlying delay driver error
    #[cfg_attr(feature = "std", error("Delay error: {0:?}"))]
    Delay(DelayErr),
}

impl<I2c, I2cErr, Delay, DelayErr> Tlv493d<I2c, I2cErr, Delay, DelayErr>
where
    I2c: i2c::Read<Error = I2cErr> + i2c::Write<Error = I2cErr> + i2c::WriteRead<Error = I2cErr>,
    I2cErr: Debug,
    Delay: delay::DelayMs<u32, Error = DelayErr>,
    DelayErr: Debug,
{
    /// Create a new TLV493D instance
    pub fn new(
        i2c: I2c,
        delay: Delay,
        addr: u8,
        mode: Mode,
    ) -> Result<Self, Error<I2cErr, DelayErr>> {
        debug!("New Tlv493d with address: 0x{:02x}", addr);

        // Construct object
        let mut s = Self {
            i2c,
            delay,
            addr,
            initial: [0u8; 10],
            last_frm: 0xff,
            _e_i2c: PhantomData,
            _e_delay: PhantomData,
        };

        // Reset and configure
        s.configure(mode, true)?;

        // Return object
        Ok(s)
    }

    /// Configure the device into the specified mode
    pub fn configure(&mut self, mode: Mode, reset: bool) -> Result<(), Error<I2cErr, DelayErr>> {
        // Startup per fig. 5.1 in TLV493D-A1B6 user manual

        // Reset if enabled
        if reset {
            debug!("Resetting device");

            // Recovery and reset is a bit fraught with disaster

            // First send recovery (S 0xFF P)
            // Note this will never be ACK'd so will report failure
            // Optional, could free sensor from aborted communication
            let _ = self.i2c.try_write(0xFF, &[]);

            // Then send reset (S 0x00 ADR P)
            // Note this is not ACK'd so will report failure
            // Setting address seems like it requires bit-bashing
            let _ = self.i2c.try_write(0x00, &[]);

            // Wait for startup delay
            self.delay.try_delay_ms(40).map_err(Error::Delay)?;

            debug!("Setting device address");

            // TODO: work out why things get upset if we set the address


            debug!("Read device initial state");

            // Read initial bitmap from device
            let _ = self
                .i2c
                .try_read(self.addr, &mut self.initial[..])
                .map_err(Error::I2c)?;

            debug!("Initial state: {:02x?}", self.initial);
        }

        // Parse out initial mode settings
        let mut m1 = unsafe { Mode1::from_bits_unchecked(self.initial[7]) };
        let m2 = unsafe { Mode2::from_bits_unchecked(self.initial[9]) };

        debug!("Current config: {:?} ({:02x?})", m1, self.initial);

        // Clear mode flags
        m1.remove(Mode1::PARITY);
        m1.remove(Mode1::FAST | Mode1::LOW);

        match mode {
            Mode::Disabled => (),
            Mode::Master => m1 |= Mode1::FAST | Mode1::LOW,
            Mode::Fast => m1 |= Mode1::FAST | Mode1::IRQ_EN,
            Mode::LowPower => m1 |= Mode1::LOW | Mode1::IRQ_EN,
            Mode::UltraLowPower => m1 |= Mode1::IRQ_EN,
        }

        let mut cfg = [0x00, m1.bits(), self.initial[8], m2.bits()];

        self.initial[7] = m1.bits();
        self.initial[9] = m2.bits();

        let mut parity = 0;
        for v in &cfg {
            for i in 0..8 {
                if v & (1 << i) != 0 {
                    parity += 1;
                }
            }
        }
        if parity % 2 == 0 {
            m1 |= Mode1::PARITY;
            cfg[1] = m1.bits();
        }

        debug!(
            "Writing config: Mode1: {:?} Mode2: {:?} (cfg: {:02x?}",
            m1, m2, cfg
        );

        self.i2c.try_write(self.addr, &cfg).map_err(Error::I2c)?;

        Ok(())
    }

    /// Read raw values from the sensor
    pub fn read_raw(&mut self) -> Result<[i16; 4], Error<I2cErr, DelayErr>> {
        let mut v = [0i16; 4];

        // Read data from device
        let mut b = [0u8; 7];
        self.i2c
            .try_read(self.addr, &mut b[..])
            .map_err(Error::I2c)?;

        // Detect ADC lockup (stalled FRM field)
        let frm = b[3] & 0b0000_1100;
        if self.last_frm == frm {
            return Err(Error::AdcLockup);
        } else {
            self.last_frm = frm;
        }

        // Convert to values
        // Double-cast here required for sign-extension
        v[0] = (b[0] as i8 as i16) << 4 | ((b[4] & 0xF0) >> 4) as i16;
        v[1] = (b[1] as i8 as i16) << 4 | (b[4] & 0x0F) as i16;
        v[2] = (b[2] as i8 as i16) << 4 | (b[5] & 0x0F) as i16;
        v[3] = (b[3] as i8 as i16 & 0xF0) << 4 | (b[6] as i16 & 0xFF);

        debug!("Read data {:02x?} values: {:04x?}", b, v);

        Ok(v)
    }

    /// Read and convert values from the sensor
    pub fn read(&mut self) -> Result<Values, Error<I2cErr, DelayErr>> {
        let raw = self.read_raw()?;

        Ok(Values {
            x: raw[0] as f32 * 0.098f32,
            y: raw[1] as f32 * 0.098f32,
            z: raw[2] as f32 * 0.098f32,
            temp: (raw[3] - 340) as f32 * 1.1f32 + 24.2f32,
        })
    }

    #[cfg(feature = "math")]
    pub fn read_angle_f32(&mut self) -> Result<f32, Error<I2cErr, DelayErr>> {
        // Read values
        let v = self.read()?;

        // https://en.wikipedia.org/wiki/Atan2
        Ok(v.x.atan2(v.y))
    }

    /// Fetch a reference to the inner delay object
    pub fn inner_delay(&mut self) -> &mut Delay {
        &mut self.delay
    }
}
