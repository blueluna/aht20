//! A platform agnostic driver to interface with the AHT20 temperature and
//! humidity sensor.
//!
//! This driver was built using [`embedded-hal`] traits and is a fork of Anthony
//! Romano's [AHT10 crate].
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.2
//! [AHT10 crate]: https://github.com/heyitsanthony/aht10

#![deny(missing_docs)]
#![no_std]

use {
    bitflags::bitflags,
    crc_all::CrcAlgo,
    embedded_hal::{delay::DelayNs, i2c::I2c},
};

const I2C_ADDRESS: u8 = 0x38;

bitflags! {
    /// AHT20 status
    #[derive(Debug, Copy, Clone)]
    pub struct StatusFlags: u8 {
        /// Device is busy measuring
        const BUSY = (1 << 7);
        /// Device is calibrated
        const CALIBRATION_ENABLE = (1 << 3);
    }
}

/// AHT20 Error.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<I2CERR> {
    /// Device is not calibrated.
    Uncalibrated,
    /// Checksum mismatch.
    Checksum,
    /// Underlying I2C error.
    I2c(I2CERR),
}

impl<E> core::convert::From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::I2c(e)
    }
}

/// Humidity reading from AHT20.
pub struct Humidity {
    h: u32,
}

impl Humidity {
    /// Humidity converted to Relative Humidity %.
    pub fn rh(&self) -> f32 {
        100.0 * (self.h as f32) / ((1 << 20) as f32)
    }

    /// Raw humidity reading.
    pub fn raw(&self) -> u32 {
        self.h
    }
}

/// Temperature reading from AHT20.
pub struct Temperature {
    t: u32,
}

impl Temperature {
    /// Temperature converted to Celsius.
    pub fn celsius(&self) -> f32 {
        (200.0 * (self.t as f32) / ((1 << 20) as f32)) - 50.0
    }

    /// Raw temperature reading.
    pub fn raw(&self) -> u32 {
        self.t
    }
}

/// AHT20 driver.
pub struct Aht20<I2C, D> {
    i2c: I2C,
    delay: D,
}

impl<I2C, D> Aht20<I2C, D>
where
    I2C: I2c,
    D: DelayNs,
{
    /// Creates a new AHT20 device from an I2C peripheral and a Delay.
    pub fn new(i2c: I2C, delay: D) -> Self {
        Self { i2c, delay }
    }

    /// Gets the sensor status.
    pub fn status(&mut self) -> Result<StatusFlags, I2C::Error> {
        let mut buf = [0u8; 1];
        self.i2c.read(I2C_ADDRESS, &mut buf)?;

        Ok(StatusFlags::from_bits_retain(buf[0]))
    }

    /// Self-calibrate the sensor.
    pub fn calibrate(&mut self) -> Result<(), Error<I2C::Error>> {
        // Send calibrate command
        self.i2c.write(I2C_ADDRESS, &[0xE1, 0x08, 0x00])?;

        // Wait until not busy
        while self.status()?.contains(StatusFlags::BUSY) {
            self.delay.delay_ms(10);
        }

        // Confirm sensor is calibrated
        if !self.status()?.contains(StatusFlags::CALIBRATION_ENABLE) {
            return Err(Error::Uncalibrated);
        }

        Ok(())
    }

    /// Soft resets the sensor.
    pub fn reset(&mut self) -> Result<(), I2C::Error> {
        // Send soft reset command
        self.i2c.write(I2C_ADDRESS, &[0xBA])?;

        // Wait 20ms as stated in specification
        self.delay.delay_ms(100);
        // Wait until not busy
        while self.status()?.contains(StatusFlags::BUSY) {
            self.delay.delay_ms(10);
        }

        Ok(())
    }

    /// Reads humidity and temperature.
    pub fn read(&mut self) -> Result<(Humidity, Temperature), Error<I2C::Error>> {
        // Send trigger measurement command
        self.i2c.write(I2C_ADDRESS, &[0xac, 0x33, 0x00])?;

        self.delay.delay_ms(80);
        // Wait until not busy
        // Read in sensor data
        let mut buf = [0u8; 7];
        loop {
            self.i2c.read(I2C_ADDRESS, &mut buf)?;
            let status = StatusFlags::from_bits_retain(buf[0]);
            if !status.contains(StatusFlags::BUSY) {
                break;
            }
        }

        // Check for CRC mismatch
        let crc = &mut 0u8;
        let crc_hasher = CrcAlgo::<u8>::new(49, 8, 0xFF, 0x00, false);
        crc_hasher.init_crc(crc);
        if crc_hasher.update_crc(crc, &buf[..=5]) != buf[6] {
            return Err(Error::Checksum);
        };

        // Check calibration
        let status = StatusFlags::from_bits_retain(buf[0]);
        if !status.contains(StatusFlags::CALIBRATION_ENABLE) {
            return Err(Error::Uncalibrated);
        }

        // Extract humitidy and temperature values from data
        let hum = ((buf[1] as u32) << 12) | ((buf[2] as u32) << 4) | ((buf[3] as u32) >> 4);
        let temp = (((buf[3] as u32) & 0x0f) << 16) | ((buf[4] as u32) << 8) | (buf[5] as u32);

        Ok((Humidity { h: hum }, Temperature { t: temp }))
    }
}
