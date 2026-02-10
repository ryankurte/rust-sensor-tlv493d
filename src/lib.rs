#![no_std]

/// Rust tlv493d 3-DoF I2C Hall Effect Sensor Driver
///
/// Copyright 2020 Ryan Kurte
///
/// - https://www.infineon.com/dgdl/Infineon-TLV493D-A1B6-DataSheet-v01_10-EN.pdf?fileId=5546d462525dbac40152a6b85c760e80
/// - https://www.infineon.com/dgdl/Infineon-TLV493D-A1B6_3DMagnetic-UM-v01_03-EN.pdf?fileId=5546d46261d5e6820161e75721903ddd
use bitflags::bitflags;
use core::fmt::Debug;
use core::marker::PhantomData;
use maybe_async_cfg::maybe;

#[cfg(feature = "async")]
use embedded_hal_async::{delay::DelayNs, i2c, i2c::Error as I2cError};

#[cfg(feature = "blocking")]
use embedded_hal::{delay::DelayNs, i2c, i2c::Error as I2cError};

#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "defmt")]
#[allow(unused_imports)]
#[macro_use]
extern crate defmt;

pub struct Tlv493d<I2c, I2cErr, Delay> {
    i2c: I2c,
    delay: Delay,
    addr: u8,
    initial: [u8; 10],
    last_frm: Option<u8>,
    _e_i2c: PhantomData<I2cErr>,
}

/// Base address for Tlv493d, bottom bit set during power up
/// based on value of SDA.
pub const ADDRESS_BASE: u8 = 0b1011110;

#[derive(Debug, PartialEq, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

#[derive(Debug, PartialEq, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Write registers for the Tlv493d
pub enum WriteRegisters {
    Res = 0x00,   // Reserved
    Mode1 = 0x01, // Mode 1 register (P | IICAddr[1..0] | Reserved[1..0] | INT | FAST | LOW)
    Res2 = 0x02,  // Reserved
    Mode2 = 0x03, // Mode 2 register (T | LP | PT | Reserved[4..0])
}

/// TLV493D Measurement values
#[derive(Debug, PartialEq, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Values {
    pub x: f32,    // X axis magnetic flux (mT)
    pub y: f32,    // Y axis magnetic flux (mT)
    pub z: f32,    // Z axis magnetic flux (mT)
    pub temp: f32, // Device temperature (C)
}

/// Device operating mode
/// Note that in most cases the mode is a combination of mode and IRQ flags
#[derive(Debug, PartialEq, Clone)]
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
    pub struct Mode2: u8 {
        const TEMP_DISABLE   = 0b1000_0000;     // DISABLE temperature measurement
        const LOW_POW_PERIOD = 0b0100_0000;     // Set low power period ("0": 100ms, "1": 12ms)
        const PARITY_TEST_EN = 0b0010_0000;     // Enable / Disable parity test
    }
}

/// Tlv493d related errors
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "std", derive(thiserror::Error))]
pub enum Error<I2cErr: I2cError + Debug> {
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
}

maybe_async_cfg::content! {
    impl<I2c, I2cErr, Delay> Tlv493d<I2c, I2cErr, Delay>
    where
        I2c: i2c::I2c,
        I2cErr: I2cError + Debug,
        Error<I2cErr>: From<Error<<I2c as i2c::ErrorType>::Error>>,
        Delay: DelayNs,
    {
        /// Create a new TLV493D instance
        #[maybe(
            sync(feature = "blocking"),
            async(feature = "async")
        )]
        pub async fn new(
            i2c: I2c,
            delay: Delay,
            addr: u8,
            mode: Mode,
        ) -> Result<Self, (Error<I2cErr>, I2c)> {
            #[cfg(feature = "defmt")]
            debug!("New Tlv493d with address: 0x{:02x}", addr);

            // Construct object
            let mut s = Self {
                i2c,
                delay,
                addr,
                initial: [0u8; 10],
                last_frm: None,
                _e_i2c: PhantomData,
            };

            // Reset and configure
            #[cfg(feature = "async")]
            if let Err(err) = s.configure_async(mode, true).await {
                return Err((err, s.i2c));
            };
            #[cfg(feature = "blocking")]
            if let Err(err) = s.configure_sync(mode, true) {
                return Err((err, s.i2c));
            };

            // Return object
            Ok(s)
        }

        pub fn into_i2c(self) -> I2c {
            self.i2c
        }

        /// Configure the device into the specified mode
        #[maybe(
            sync(feature = "blocking"),
            async(feature = "async")
        )]
        pub async fn configure(&mut self, mode: Mode, reset: bool) -> Result<(), Error<I2cErr>> {
            // Startup per fig. 5.1 in TLV493D-A1B6 user manual

            // Reset if enabled
            if reset {
                #[cfg(feature = "defmt")]
                debug!("Resetting device");

                // Wait for startup delay
                #[cfg(feature = "async")]
                self.delay.delay_ms(1).await;
                #[cfg(feature = "blocking")]
                self.delay.delay_ms(1);

                // Write recovery value
                #[cfg(feature = "async")]
                self.i2c.write(0xff, &[]).await
                    .map_err(Error::I2c).ok();
                #[cfg(feature = "blocking")]
                self.i2c.write(0xff, &[])
                    .map_err(Error::I2c).ok();

                #[cfg(feature = "defmt")]
                debug!("Setting device address");

                // Write reset
                #[cfg(feature = "async")]
                self.i2c.write(0x00, &[0xff]).await
                    .map_err(Error::I2c).ok();
                #[cfg(feature = "blocking")]
                self.i2c.write(0x00, &[0xff])
                    .map_err(Error::I2c).ok();

                #[cfg(feature = "defmt")]
                debug!("Read device initial state");

                // Read initial bitmap from device
                #[cfg(feature = "async")]
                self.i2c.read(self.addr, &mut self.initial[..]).await
                    .map_err(Error::I2c)?;
                #[cfg(feature = "blocking")]
                self.i2c.read(self.addr, &mut self.initial[..])
                    .map_err(Error::I2c)?;

                #[cfg(feature = "defmt")]
                debug!("Initial state: {:02x}", self.initial);
            }

            // Parse out initial mode settings
            let Some(mut mod1) = Mode1::from_bits(self.initial[7]) else {
                panic!("implementation error");
            };
            let Some(mod2) = Mode2::from_bits(self.initial[9]) else {
                panic!("implementation error");
            };

            // Clear mode flags
            mod1.remove(Mode1::PARITY);
            mod1.remove(Mode1::FAST | Mode1::LOW);

            match mode {
                Mode::Disabled => (),
                Mode::Master => mod1 |= Mode1::FAST | Mode1::LOW,
                Mode::Fast => mod1 |= Mode1::FAST | Mode1::IRQ_EN,
                Mode::LowPower => mod1 |= Mode1::LOW | Mode1::IRQ_EN,
                Mode::UltraLowPower => mod1 |= Mode1::IRQ_EN,
            }

            let mut cfg = [0x00, mod1.bits(), self.initial[8], mod2.bits()];

            self.initial[7] = mod1.bits();
            self.initial[9] = mod2.bits();

            let mut parity = 0;
            for v in &cfg {
                for i in 0..8 {
                    if v & (1 << i) != 0 {
                        parity += 1;
                    }
                }
            }
            if parity % 2 == 0 {
                mod1 |= Mode1::PARITY;
                cfg[1] = mod1.bits();
            }

            #[cfg(feature = "async")]
            self.i2c.write(self.addr, &cfg).await
                .map_err(Error::I2c)?;
            #[cfg(feature = "blocking")]
            self.i2c.write(self.addr, &cfg)
                .map_err(Error::I2c)?;

            Ok(())
        }

        /// Read raw values from the sensor
        #[maybe(
            sync(feature = "blocking"),
            async(feature = "async")
        )]
        pub async fn read_raw(&mut self) -> Result<[i16; 4], Error<I2cErr>> {
            let mut v = [0i16; 4];

            // Read data from device
            let mut b = [0u8; 7];
            #[cfg(feature = "async")]
            self.i2c.read(self.addr, &mut b[..]).await
                .map_err(Error::I2c)?;
            #[cfg(feature = "blocking")]
            self.i2c.read(self.addr, &mut b[..])
                .map_err(Error::I2c)?;

            // Detect ADC lockup (stalled FRM field)
            let frm = b[3] & 0b0000_1100;
            if let Some(last_frm) = self.last_frm
            && last_frm == frm
            {
                return Err(Error::AdcLockup);
            }
            self.last_frm = Some(frm);

            // Convert to values
            // Double-cast here required for sign-extension
            v[0] = (b[0] as i8 as i16) << 4 | ((b[4] & 0xf0) >> 4) as i16;
            v[1] = (b[1] as i8 as i16) << 4 | (b[4] & 0x0f) as i16;
            v[2] = (b[2] as i8 as i16) << 4 | (b[5] & 0x0f) as i16;
            v[3] = (b[3] as i8 as i16 & 0xf0) << 4 | (b[6] as i16 & 0xff);

            #[cfg(feature = "defmt")]
            debug!("Read data {:02x} values: {:04x}", b, v);

            Ok(v)
        }

        /// Read and convert values from the sensor
        #[maybe(
            sync(feature = "blocking"),
            async(feature = "async")
        )]
        pub async fn read(&mut self) -> Result<Values, Error<I2cErr>> {
            #[cfg(feature = "async")]
            let raw = self.read_raw_async().await?;
            #[cfg(feature = "blocking")]
            let raw = self.read_raw_sync()?;

            Ok(Values {
                x: raw[0] as f32 * 0.098f32,
                y: raw[1] as f32 * 0.098f32,
                z: raw[2] as f32 * 0.098f32,
                temp: (raw[3] - 340) as f32 * 1.1f32 + 24.2f32,
            })
        }

        #[cfg(feature = "math")]
        #[maybe(
            sync(feature = "blocking"),
            async(feature = "async")
        )]
        pub async fn read_angle_f32(&mut self) -> Result<f32, Error<I2cErr>> {
            // Read values
            #[cfg(feature = "async")]
            let v = self.read_async().await?;
            #[cfg(feature = "blocking")]
            let v = self.read_sync()?;

            // https://en.wikipedia.org/wiki/Atan2
            Ok(libm::Libm::<f32>::atan2(v.x, v.y))
        }
    }
}
