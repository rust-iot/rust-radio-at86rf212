//! AT86RF212 Radio Driver
//! Copyright 2018 Ryan Kurte

#![allow(non_camel_case_types)]

// SPI control flags
pub const REG_WRITE_FLAG: usize = (0xC0);
pub const REG_READ_FLAG: usize = (0x80);
pub const FRAME_WRITE_FLAG: usize = (0x60);
pub const FRAME_READ_FLAG: usize = (0x20);
pub const SRAM_WRITE_FLAG: usize = (0x40);
pub const SRAM_READ_FLAG: usize = (0x00);

// Device pub constants

/// Maximum packet length
pub const MAX_LENGTH: usize = (127);
/// Length of the PDSU length field
pub const LEN_FIELD_LEN: usize = 1;
/// Length of the CRC field
pub const CRC_LEN: usize = 2;
/// Number of additional bytes read from frame buffer on RX
pub const FRAME_RX_OVERHEAD: usize = 3;

// TRX commands
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum TrxCmd {
    NOP = 0x00,           // No-op for reading SPI status (configurable)
    TX_START = 0x02,      // Start TX
    FORCE_TRX_OFF = 0x03, // Force disable TRX module
    FORCE_PLL_ON = 0x04,  // Force enable PLL
    RX_ON = 0x06,         // Enable RX
    TRX_OFF = 0x08,       // Disable TRX module
    PLL_ON = 0x09,        // Enable PLL
    RX_AACK_ON = 0x16,    // Enable RX with Auto-Ack
    TX_ARET_ON = 0x19,    // Enable TX with Auto-Retry
    SLEEP = 0x0F,         // Switch to sleep mode
}

// TRX status
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum TrxStatus {
    P_ON = 0x00,               // pub constant P_ON for sub-register @ref SR_TRX_STATUS
    BUSY_RX = 0x01,            // Busy receiving
    BUSY_TX = 0x02,            // Busy transmitting
    RX_ON = 0x06,              // Receive mode
    TRX_OFF = 0x08,            // TRX module disabled
    PLL_ON = 0x09,             // PLL enabled
    TRX_SLEEP = 0x0F,          // TRX module sleeping
    BUSY_RX_AACK = 0x11,       // RX busy (auto-ack)
    BUSY_TX_ARET = 0x12,       // TX busy (auto-retry)
    RX_AACK_ON = 0x16,         // RX on (auto-ack)
    TX_ARET_ON = 0x19,         // TX on (auto-retry)
    RX_ON_NOCLK = 0x1C,        // RX on but no clock enabled
    RX_AACK_ON_NOCLK = 0x1D,   // RX on (auto-ack) but no clock enabled
    BUSY_RX_AACK_NOCLK = 0x1E, // RX busy (auto-ack) but no clock enabled
    STATE_TRANSITION_IN_PROGRESS = 0x1F, // Currently changing states
}

// TRAC Status (Advanced mode)
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum TrxTracStatus {
    TRAC_SUCCESS = 0,                // AACK or ARET successful
    TRAC_SUCCESS_DATA_PENDING = 1,   // ARET ACK frame received with pending flag set
    TRAC_SUCCESS_WAIT_FOR_ACK = 2,   // AACK ACK frame will be sent in nexta vailable slot
    TRAC_CHANNEL_ACCESS_FAILURE = 3, // Error accessing channel
    TRAC_NO_ACK = 5,                 // NO ack received
    TRAC_INVALID = 7,                // Invalid state
}

// TAL states
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum TALStates {
    TAL_IDLE = 0,
    TAL_TX_AUTO = 1,
    TAL_TX_END = 2,
}

// CCA Mode
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum CCAMode {
    CCA_MODE_CS_OR_ENERGY = 0,  // Carrier sense OR energy above threshold
    CCA_MODE_ENERGY = 1,        // Energy above threshold
    CCA_MODE_CS = 2,            // Carrier sense
    CCA_MODE_CS_AND_ENERGY = 3, // Carrier sense AND energy above threshold
}

// Modulation mode
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Modulation {
    MODULATION_BPSK = 0,  // BPSK modulation
    MODULATION_OQPSK = 2, // OQPSK modulation
}

// OQPSK Data Rate
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum OqpskDataRate {
    OQPSK_DATA_RATE_0_100K_1_250K = 0, // Data rate where SUB_MODE 0 100K, 1 250K
    OQPSK_DATA_RATE_0_200K_1_500K = 1, // Data rate where SUB_MODE 0 200K, 1 500K
    OQPSK_DATA_RATE_0_400K_1_1000K = 2, // Data rate where SUB_MODE 0 400K, 1 1000K
    OQPSK_DATA_RATE_0_NA_1_500K = 3,   // Data rate where SUB_MODE 0 NA, 1 500K
}

// IRQ flags
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum IRQ {
    IRQ_NONE = 0x00,
    IRQ_0_PLL_LOCK = 0x01,
    IRQ_1_PLL_UNLOCK = 0x02,
    IRQ_2_RX_START = 0x04,
    IRQ_3_TRX_END = 0x08,
    IRQ_4_CCA_ED_DONE = 0x10,
    IRQ_5_AMI = 0x20,
    IRQ_6_TRX_UR = 0x40,
    IRQ_7_BAT_LOW = 0x80,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum ClkmRate {
    CLKM_RATE_NONE = 0x00,
    CLKM_RATE_1MHZ = 0x01,
    CLKM_RATE_2MHZ = 0x02,
    CLKM_RATE_4MHZ = 0x03,
    CLKM_RATE_8MHZ = 0x04,
    CLKM_RATE_16MHZ = 0x05,
    CLKM_RATE_250KHZ = 0x06,
    CLKM_RATE_802_15_4_SYMBOL_RATE = 0x07,
}

pub mod defaults {
    use super::CCAMode;

    pub const CHANNEL: u8 = 1;
    pub const CCA_MODE: CCAMode = CCAMode::CCA_MODE_ENERGY;
    pub const MINBE: u8 = 3;
    pub const MAXBE: u8 = 5;
    pub const MAX_CSMA_BACKOFFS: u8 = 4;
    pub const PLL_LOCK_RETRIES: u8 = 10;
    pub const STATE_CHANGE_RETRIES: u8 = 10;
    pub const MAX_SPI_RETRIES: usize = 10;
}
