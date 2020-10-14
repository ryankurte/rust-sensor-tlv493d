# sensor-tlv493d

A rust driver (and utility) for the Infineon tlv493d 3-DoF I2C hall-effect sensor

## Status

[![GitHub tag](https://img.shields.io/github/tag/ryankurte/rust-sensor-tlv493d.svg)](https://github.com/ryankurte/rust-sensor-tlv493d)
[![Crates.io](https://img.shields.io/crates/v/sensor-tlv493d.svg)](https://crates.io/crates/sensor-tlv493d)
[![Docs.rs](https://docs.rs/sensor-tlv493d/badge.svg)](https://docs.rs/sensor-tlv493d)

[Open Issues](https://github.com/ryankurte/rust-radio-sx127x/issues)

## Usage

- Add to your project with `cargo add sensor-tlv493d`
- Install the utility with `cargo install sensor-tlv498d`

Note that for `no_std` use you must `default_features = false` to disable dependencies on `std` for error handling etc.

## Resources

- [Datasheet](https://www.infineon.com/dgdl/Infineon-TLV493D-A1B6-DataSheet-v01_10-EN.pdf?fileId=5546d462525dbac40152a6b85c760e80)
- [User Manual](https://www.infineon.com/dgdl/Infineon-TLV493D-A1B6_3DMagnetic-UM-v01_03-EN.pdf?fileId=5546d46261d5e6820161e75721903ddd)
