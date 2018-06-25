//! AT86RF212 Radio Driver
//! See also: 
//! - https://github.com/ryankurte/libat86rf212
//! Copyright 2018 Ryan Kurte

#![no_std]
#![feature(never_type)]
#![feature(unproven)]
extern crate embedded_hal as hal;
extern crate futures;

extern crate nb;

use hal::blocking::spi;
use hal::digital::{InputPin, OutputPin};
use hal::spi::{Mode, Phase, Polarity};

pub mod device;
pub mod regs;
use regs::{Register};

/// AT86RF212 SPI operating mode
pub const MODE: Mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
};

/// AT86RF212 device object
pub struct AT86RF212<SPI, OUTPUT, INPUT> {
    spi: SPI,
    reset: OUTPUT,
    cs: OUTPUT,
    gpio: [Option<INPUT>; 4],
}

/// AT86RF212 configuration
pub struct Config {
    /// Radio frequency for communication
    rf_freq_mhz: u16,
}

impl<E, SPI, OUTPUT, INPUT> AT86RF212<SPI, OUTPUT, INPUT>
where
    SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>,
    OUTPUT: OutputPin,
    INPUT: InputPin,
{
    pub fn new(spi: SPI, reset: OUTPUT, cs: OUTPUT, gpio: [Option<INPUT>; 4]) -> Result<Self, E> {
        let mut AT86RF212 = AT86RF212 { spi, reset, cs, gpio };

        AT86RF212.reset.set_low();
        // TODO: wait 1ms
        AT86RF212.reset.set_high();
        // TODO: wait 10ms

        // TODO: the rest of the init here


        Ok(AT86RF212)
    }

    /// Read data from a specified register address
    /// This consumes the provided input data array and returns a reference to this on success
    fn reg_read<'a>(&mut self, reg: Register) -> Result<u8, E> {
        // Setup read command
        let mut buf: [u8; 2] = [device::REG_READ_FLAG as u8 | reg as u8, 0];
        // Assert CS
        self.cs.set_low();
        // Transfer data
        let res = self.spi.transfer(&mut buf);
        // Clear CS
        self.cs.set_high();
        // Return result
        res.map(|v| v[1])
    }

    /// Write data to a specified register address
    fn reg_write(&mut self, reg: Register, value: u8) -> Result<(), E> {
        // Setup write command
        let buf: [u8; 2] = [device::REG_WRITE_FLAG as u8 | reg as u8, value];
        // Assert CS
        self.cs.set_low();
        // Write command
        let res = self.spi.write(&buf);
        // Clear CS
        self.cs.set_high();
        // Return result
        res.map(|_v| ())
    }

    /// Update a portion of a register with the provided mask and value
    /// Note that shifting is not automatically applied
    fn reg_update(&mut self, reg: Register, mask: u8, value: u8) -> Result<(), E> {
        let mut data = self.reg_read(reg.clone())?;
        data &= !mask;
        data |= mask & value;
        self.reg_write(reg, data)?;
        Ok(())
    }

    /// Read a frame from the device
    fn read_frame<'a>(&mut self, data: &'a mut [u8]) -> Result<&'a [u8], E> {
        // Setup read frame command
        let cmd: [u8; 1] = [device::FRAME_READ_FLAG as u8];
        // Assert CS
        self.cs.set_low();
        // Write command
        match self.spi.write(&cmd) {
            Ok(_r) => (),
            Err(e) => {
                self.cs.set_high();
                return Err(e);
            }
        };
        // Transfer data
        let res = match self.spi.transfer(data) {
            Ok(r) => r,
            Err(e) => {
                self.cs.set_high();
                return Err(e);
            }
        };
        // Clear CS
        self.cs.set_high();
        // Return result (contains returned data)
        Ok(res)
    }

    /// Write a frame to the device
    fn write_frame(&mut self, data: &[u8]) -> Result<(), E> {
        // Setup write frame command
        let cmd: [u8; 1] = [device::FRAME_WRITE_FLAG as u8];
        // Assert CS
        self.cs.set_low();
        // Write command
        match self.spi.write(&cmd) {
            Ok(_r) => (),
            Err(e) => {
                self.cs.set_high();
                return Err(e);
            }
        };
        // Transfer data
        match self.spi.write(&data) {
            Ok(_r) => (),
            Err(e) => {
                self.cs.set_high();
                return Err(e);
            }
        };
        // Clear CS
        self.cs.set_high();

        Ok(())
    }

    /// Set the radio state
    pub fn set_state(&mut self, state: device::TrxCmd) -> Result<(), E> {
        self.reg_update(Register::TRX_STATE, regs::TRX_STATE_TRX_CMD_MASK, state as u8)
    }

    /// Fetch the radio state
    pub fn get_state(&mut self) -> Result<u8, E> {
        self.reg_read(Register::TRX_STATE).map(|v| v & regs::TRX_STATUS_TRX_STATUS_MASK)
    }

    /// Set the radio channel
    pub fn set_channel(&mut self, channel: u8) -> Result<(), E> {
        self.reg_update(Register::PHY_CC_CCA, regs::PHY_CC_CCA_CHANNEL_MASK, channel << regs::PHY_CC_CCA_CHANNEL_SHIFT)
    }

    /// Fetch the radio channel
    pub fn get_channel(&mut self) -> Result<u8, E> {
        self.reg_read(Register::PHY_CC_CCA).map(|v| (v & regs::PHY_CC_CCA_CHANNEL_MASK) >> regs::PHY_CC_CCA_CHANNEL_SHIFT)
    }

    /// Set the IRQ mask
    pub fn set_irq_mask(&mut self, mask: u8) -> Result<(), E> {
        self.reg_write(Register::IRQ_MASK, mask)
    }

    /// Fetch the IRQ status (clears pending interrupts)
    pub fn get_irq_status(&mut self) -> Result<u8, E> {
        self.reg_read(Register::IRQ_STATUS)
    }

    /// Set the Clear Channel Assessment (CCA) mode
    pub fn set_cca_mode(&mut self, mode: u8) -> Result<(), E> {
        self.reg_update(Register::PHY_CC_CCA, regs::PHY_CC_CCA_CCA_MODE_MASK, mode << regs::PHY_CC_CCA_CCA_MODE_SHIFT)
    }

    /// Fetch the Clear Channel Assessment (CCA) mode
    pub fn get_cca_mode(&mut self) -> Result<u8, E> {
        self.reg_read(Register::PHY_CC_CCA).map(|v| (v & regs::PHY_CC_CCA_CCA_MODE_MASK) >> regs::PHY_CC_CCA_CCA_MODE_SHIFT)
    }

    /// Set the 802.15.4 short address
    pub fn set_short_address(&mut self, address: u16) -> Result<(), E> {
        self.reg_write(Register::SHORT_ADDR_0, (address & 0xFF) as u8)?;
        self.reg_write(Register::SHORT_ADDR_1, (address >> 8) as u8)?;
        Ok(())
    }

    /// Set the 802.15.4 PAN ID
    pub fn set_pan_id(&mut self, pan_id: u16) -> Result<(), E> {
        self.reg_write(Register::PAN_ID_0, (pan_id & 0xFF) as u8)?;
        self.reg_write(Register::PAN_ID_1, (pan_id >> 8) as u8)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
