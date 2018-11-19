//! AT86RF212 Radio Driver
//! See also: 
//! - https://github.com/ryankurte/libat86rf212
//! Copyright 2018 Ryan Kurte

#![no_std]
#![feature(never_type)]

extern crate embedded_hal as hal;

extern crate nb;

use hal::blocking::{spi, delay};
use hal::digital::{InputPin, OutputPin};
use hal::spi::{Mode, Phase, Polarity};

pub mod device;
pub use device::{TrxCmd, TrxStatus, CCAMode, defaults};
pub mod regs;
pub use regs::{Register};

// At86rf212 error types
#[derive(Copy, Clone, Debug)]
pub enum At86rf212Error<SPIError> {
    /// Communication error
    SPI(SPIError),
    /// Invalid part error 
    InvalidPart(u8),
    /// Length error (mismatch or length exceeds allowable)
    InvalidLength(usize),
    /// Command failed after MAX_RETRIES
    MaxRetries,
    /// PLL locking error
    PLLLock,
    /// Digital voltage error
    DigitalVoltage,
    /// Analogue voltage error
    AnalogueVoltage,
}

impl <SPIError>From<SPIError> for At86rf212Error<SPIError> {
	fn from(e: SPIError) -> At86rf212Error<SPIError> {
		At86rf212Error::SPI(e)
	}
}

/// AT86RF212 SPI operating mode
pub const MODE: Mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
};

/// AT86RF212 device object
pub struct AT86RF212<SPI, OUTPUT, DELAY> {
    spi: SPI,
    reset: OUTPUT,
    cs: OUTPUT,
    sleep: OUTPUT,
    delay: DELAY,
    auto_crc: bool,
}

impl<E, SPI, OUTPUT, DELAY> AT86RF212<SPI, OUTPUT, DELAY>
where
    SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>,
    OUTPUT: OutputPin,
    DELAY: delay::DelayMs<u32>,
{
    pub fn new(spi: SPI, reset: OUTPUT, cs: OUTPUT, sleep: OUTPUT, delay: DELAY) -> Result<Self, At86rf212Error<E>> {
        let mut at86rf212 = AT86RF212 { spi, reset, cs, sleep, delay, auto_crc: true };

        // Disable slee mode
        at86rf212.sleep.set_low();

        // Reset pulse
        at86rf212.reset.set_low();
        at86rf212.delay.delay_ms(1);
        at86rf212.reset.set_high();
        
        // Wait for init
        at86rf212.delay.delay_ms(10);

        // Check communication
        let who = at86rf212.reg_read(Register::PART_NUM)?;
        if who != 0x07 {
            return Err(At86rf212Error::InvalidPart(who));
        }

        // Disable TRX
        at86rf212.set_state(TrxCmd::TRX_OFF)?;

        // Check digital voltage is ok
        let v = at86rf212.reg_read(Register::VREG_CTRL)?;
        if v & regs::VREG_CTRL_DVDD_OK_MASK == 0 {
            return Err(At86rf212Error::DigitalVoltage);
        }

        // Set default channel
        at86rf212.set_channel(defaults::CHANNEL)?;

        // Set default CCA mode
        at86rf212.set_cca_mode(defaults::CCA_MODE)?;

        // Enable CSMA-CA
        // Set binary exponentials
        at86rf212.reg_write(Register::CSMA_BE, (defaults::MINBE << regs::CSMA_BE_MIN_SHIFT) | ((defaults::CCA_MODE as u8) << regs::CSMA_BE_MAX_SHIFT))?;

        // Set max CMSA retries
        at86rf212.reg_update(Register::XAH_CTRL_0, regs::XAH_CTRL_MAX_CSMA_RETRIES_MASK, defaults::MAX_CSMA_BACKOFFS << regs::XAH_CTRL_MAX_CSMA_RETRIES_SHIFT)?;

        // Enable promiscuous mode auto ack
        at86rf212.reg_update(Register::XAH_CTRL_1, regs::XAH_CTRL_1_AACK_PROM_MODE_MASK, 1 << regs::XAH_CTRL_1_AACK_FLTR_RES_FT_MASK)?;

        // Enable auto-crc for transmit mode
        at86rf212.reg_update(Register::TRX_CTRL_1, regs::TRX_CTRL1_TX_AUTO_CRC_ON_MASK, 1 << regs::TRX_CTRL1_TX_AUTO_CRC_ON_SHIFT)?;

        // Enable IRQ_MASK so enabled interrupt cause IRQ to be asserted
        at86rf212.reg_update(Register::TRX_CTRL_1, regs::TRX_CTRL1_IRQ_MASK_MODE_MASK, 1 << regs::TRX_CTRL1_IRQ_MASK_MODE_SHIFT)?;

        // Enable dynamic frame buffer protection
        at86rf212.reg_update(Register::TRX_CTRL_2, regs::TRX_CTRL2_RX_SAFE_MODE_MASK, 1 << regs::TRX_CTRL2_RX_SAFE_MODE_SHIFT)?;

        // TODO: Enable desired interrupts

        Ok(at86rf212)
    }

    /// Read data from a specified register address
    /// This consumes the provided input data array and returns a reference to this on success
    pub fn reg_read<'a>(&mut self, reg: Register) -> Result<u8, At86rf212Error<E>> {
        // Setup read command
        let mut buf: [u8; 2] = [device::REG_READ_FLAG as u8 | reg as u8, 0];
        // Assert CS
        self.cs.set_low();
        // Transfer data
        let res = self.spi.transfer(&mut buf);
        // Clear CS
        self.cs.set_high();
        // Return result
        match res {
            Ok(v) => Ok(v[1]),
            Err(e) => Err(At86rf212Error::SPI(e)),
        }
    }

    /// Write data to a specified register address
    pub fn reg_write(&mut self, reg: Register, value: u8) -> Result<(), At86rf212Error<E>> {
        // Setup write command
        let buf: [u8; 2] = [device::REG_WRITE_FLAG as u8 | reg as u8, value];
        // Assert CS
        self.cs.set_low();
        // Write command
        let res = self.spi.write(&buf);
        // Clear CS
        self.cs.set_high();
        // Return result
        match res {
            Ok(_) => Ok(()),
            Err(e) => Err(At86rf212Error::SPI(e)),
        }
    }

    /// Update a portion of a register with the provided mask and value
    /// Note that shifting is not automatically applied
    pub fn reg_update(&mut self, reg: Register, mask: u8, value: u8) -> Result<(), At86rf212Error<E>> {
        let mut data = self.reg_read(reg.clone())?;
        data &= !mask;
        data |= mask & value;
        self.reg_write(reg, data)?;
        Ok(())
    }

    /// Read a frame from the device
    pub fn read_frame<'a>(&mut self, data: &'a mut [u8]) -> Result<&'a [u8], At86rf212Error<E>> {
        // Setup read frame command
        let cmd: [u8; 1] = [device::FRAME_READ_FLAG as u8];
        // Assert CS
        self.cs.set_low();
        // Write command
        match self.spi.write(&cmd) {
            Ok(_r) => (),
            Err(e) => {
                self.cs.set_high();
                return Err(At86rf212Error::SPI(e));
            }
        };
        // Transfer data
        let res = match self.spi.transfer(data) {
            Ok(r) => r,
            Err(e) => {
                self.cs.set_high();
                return Err(At86rf212Error::SPI(e));
            }
        };
        // Clear CS
        self.cs.set_high();
        // Return result (contains returned data)
        Ok(res)
    }

    /// Write a frame to the device
    pub fn write_frame(&mut self, data: &[u8]) -> Result<(), At86rf212Error<E>> {
        // Setup write frame command
        let mut cmd: [u8; 2] = [device::FRAME_WRITE_FLAG as u8, 0];
        if self.auto_crc {
            cmd[1] = (data.len() + device::LEN_FIELD_LEN + device::CRC_LEN) as u8;
        } else {
            cmd[1] = (data.len() + device::LEN_FIELD_LEN) as u8;
        }
        // Assert CS
        self.cs.set_low();
        // Write command
        match self.spi.write(&cmd) {
            Ok(_r) => (),
            Err(e) => {
                self.cs.set_high();
                return Err(At86rf212Error::SPI(e));
            }
        };
        // Transfer data
        match self.spi.write(&data) {
            Ok(_r) => (),
            Err(e) => {
                self.cs.set_high();
                return Err(At86rf212Error::SPI(e));
            }
        };
        // Add CRC padding if required
        if self.auto_crc {
            let crc = [0u8; device::CRC_LEN];
            match self.spi.write(&crc) {
                Ok(_r) => (),
                Err(e) => {
                    self.cs.set_high();
                    return Err(At86rf212Error::SPI(e));
                }
            };
        }
        // Clear CS
        self.cs.set_high();

        Ok(())
    }

    /// Set the radio state
    pub fn set_state(&mut self, state: device::TrxCmd) -> Result<(), At86rf212Error<E>> {
        self.reg_update(Register::TRX_STATE, regs::TRX_STATE_TRX_CMD_MASK, state as u8)
    }

    pub fn set_state_blocking(&mut self, state: device::TrxCmd) -> Result<(), At86rf212Error<E>> {
        self.reg_update(Register::TRX_STATE, regs::TRX_STATE_TRX_CMD_MASK, state as u8)?;

        for _i in 0..defaults::MAX_SPI_RETRIES {
            let v = self.reg_read(Register::TRX_STATE)?;
            if (v & regs::TRX_STATUS_TRX_STATUS_MASK) != (TrxStatus::STATE_TRANSITION_IN_PROGRESS as u8) {
                return Ok(());
            }
            self.delay.delay_ms(1);
        }

        Err(At86rf212Error::MaxRetries)
    }

    /// Fetch the radio state
    pub fn get_state(&mut self) -> Result<u8, At86rf212Error<E>> {
        self.reg_read(Register::TRX_STATE).map(|v| v & regs::TRX_STATUS_TRX_STATUS_MASK)
    }

    /// Set the radio channel
    pub fn set_channel(&mut self, channel: u8) -> Result<(), At86rf212Error<E>> {
        self.reg_update(Register::PHY_CC_CCA, regs::PHY_CC_CCA_CHANNEL_MASK, channel << regs::PHY_CC_CCA_CHANNEL_SHIFT)
    }

    /// Fetch the radio channel
    pub fn get_channel(&mut self) -> Result<u8, At86rf212Error<E>> {
        self.reg_read(Register::PHY_CC_CCA).map(|v| (v & regs::PHY_CC_CCA_CHANNEL_MASK) >> regs::PHY_CC_CCA_CHANNEL_SHIFT)
    }

    /// Set the IRQ mask
    pub fn set_irq_mask(&mut self, mask: u8) -> Result<(), At86rf212Error<E>> {
        self.reg_write(Register::IRQ_MASK, mask)
    }

    /// Fetch the IRQ status (clears pending interrupts)
    pub fn get_irq_status(&mut self) -> Result<u8, At86rf212Error<E>> {
        self.reg_read(Register::IRQ_STATUS)
    }

    /// Set the Clear Channel Assessment (CCA) mode
    pub fn set_cca_mode(&mut self, mode: CCAMode) -> Result<(), At86rf212Error<E>> {
        self.reg_update(Register::PHY_CC_CCA, regs::PHY_CC_CCA_CCA_MODE_MASK, (mode as u8) << regs::PHY_CC_CCA_CCA_MODE_SHIFT)
    }

    /// Fetch the Clear Channel Assessment (CCA) mode
    pub fn get_cca_mode(&mut self) -> Result<u8, At86rf212Error<E>> {
        self.reg_read(Register::PHY_CC_CCA).map(|v| (v & regs::PHY_CC_CCA_CCA_MODE_MASK) >> regs::PHY_CC_CCA_CCA_MODE_SHIFT)
    }

    /// Set the 802.15.4 short address
    pub fn set_short_address(&mut self, address: u16) -> Result<(), At86rf212Error<E>> {
        self.reg_write(Register::SHORT_ADDR_0, (address & 0xFF) as u8)?;
        self.reg_write(Register::SHORT_ADDR_1, (address >> 8) as u8)?;
        Ok(())
    }

    /// Set the 802.15.4 PAN ID
    pub fn set_pan_id(&mut self, pan_id: u16) -> Result<(), At86rf212Error<E>> {
        self.reg_write(Register::PAN_ID_0, (pan_id & 0xFF) as u8)?;
        self.reg_write(Register::PAN_ID_1, (pan_id >> 8) as u8)?;
        Ok(())
    }

    // Set transmit power (raw, see datasheet for calculations)
    pub fn set_power_raw(&mut self, power: u8) -> Result<(), At86rf212Error<E>> {
        self.reg_update(Register::PHY_TX_PWR, regs::PHY_TX_PWR_TX_PWR_MASK, power << regs::PHY_TX_PWR_TX_PWR_SHIFT)?;
        Ok(())
    }

    pub fn start_rx(&mut self, channel: u8) -> Result<(), At86rf212Error<E>> {
        // Set to IDLE
        self.set_state_blocking(TrxCmd::TRX_OFF)?;

        // Set channel
        self.set_channel(channel)?;

        // Clear interrupts
        self.get_irq_status()?;

        // Enable PLL
        self.enable_pll()?;

        // Enable RX
        self.set_state_blocking(TrxCmd::RX_ON)?;

        Ok(())
    }

    fn enable_pll(&mut self) -> Result<(), At86rf212Error<E>> {
        // Send PLL on cmd
        self.set_state_blocking(TrxCmd::PLL_ON)?;

        // Await PLL lock (IRQ)
        for _i in 0 .. defaults::MAX_SPI_RETRIES {
            let v = self.get_irq_status()?;
            if (v & regs::IRQ_STATUS_IRQ_0_PLL_LOCK_MASK) != 0 {
                return Ok(())
            }
        }

        Err(At86rf212Error::PLLLock)
    }

    pub fn check_tx_rx(&mut self) -> Result<bool, At86rf212Error<E>> {
        let v = self.get_irq_status()?;
        if (v & regs::IRQ_STATUS_IRQ_3_TRX_END_MASK) != 0 {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    pub fn get_rx<'a>(&mut self, data: &'a mut [u8]) -> Result<&'a [u8], At86rf212Error<E>>{
        // Fetch frame length
        let mut buf = [0u8; 1];
        let len = self.read_frame(&mut buf)?[0] as usize;
        // Check length is valid
        if len > device::MAX_LENGTH {
            return Err(At86rf212Error::InvalidLength(len));
        }
        // Check length fits in provided data array
        if (len + device::LEN_FIELD_LEN + device::FRAME_RX_OVERHEAD) > data.len() {
            return Err(At86rf212Error::InvalidLength(len));
        }
        // Read the frame
        self.read_frame(data)
    }

    pub fn start_tx(&mut self, data: &[u8]) -> Result<(), At86rf212Error<E>>{
        // Disable TRX
        self.set_state_blocking(TrxCmd::TRX_OFF)?;
        // Clear IRQs
        self.get_irq_status()?;
        // Enable the PLL
        self.enable_pll()?;
        // Write data to buffer
        self.write_frame(data)?;
        // Set TX mode
        self.set_state_blocking(TrxCmd::TX_START)
    }

}
