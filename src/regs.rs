//! AT86RF212 Radio Driver
//! Register Definitions
//! 
// Copyright 2018 Ryan Kurte

#![allow(non_camel_case_types)]

/// Register base addresses
pub const REG_BASE_ADDR: u8 = (0x00);
/// AES base address
pub const AES_BASE_ADDR: u8 = (0x80);

/// AT86RF212 Register Enumeration

#[derive(Copy, Clone, Debug)]
pub enum Register {
    TRX_STATUS = 0x01,   // TRX_STATUS register address
    TRX_STATE = 0x02,    // TRX_STATE register address
    TRX_CTRL_0 = 0x03,   // TRX_CTRL_0 register address
    TRX_CTRL_1 = 0x04,   // TRX_CTRL_1 register address
    PHY_TX_PWR = 0x05,   // PHY_TX_PWR register address
    PHY_RSSI = 0x06,     // PHY_RSSI register address
    PHY_ED_LEVEL = 0x07, // PHY_ED_LEVEL register address
    PHY_CC_CCA = 0x08,   // PHY_CC_CCA register address
    CCA_THRES = 0x09,    // CCA_THRES register address
    RX_CTRL = 0x0A,      // RX_CTRL register address
    SFD_VALUE = 0x0B,    // SFD_VALUE register address
    TRX_CTRL_2 = 0x0C,   // TRX_CTRL_2 register address
    ANT_DIV = 0x0D,      // ANT_DIV register address
    IRQ_MASK = 0x0E,     // IRQ_MASK register address
    IRQ_STATUS = 0x0F,   // IRQ_STATUS register address
    VREG_CTRL = 0x10,    // VREG_CTRL register address
    BATMON = 0x11,       // BATMON register address
    XOSC_CTRL = 0x12,    // XOSC_CTRL register address
    CC_CTRL_0 = 0x13,    // CC_CTRL_0 register address
    CC_CTRL_1 = 0x14,    // CC_CTRL_1 register address
    RX_SYN = 0x15,       // RX_SYN register address
    RF_CTRL_0 = 0x16,    // RF_CTRL_0 register address
    XAH_CTRL_1 = 0x17,   // XAH_CTRL_1 register address
    FTN_CTRL = 0x18,     // FTN_CTRL register address
    RF_CTRL_1 = 0x19,    // RF_CTRL_1 register address
    PLL_CF = 0x1A,       // PLL_CF register address
    PLL_DCU = 0x1B,      // PLL_DCU register address
    PART_NUM = 0x1C,     // PART_NUM register address
    VERSION_NUM = 0x1D,  // VERSION_NUM register address
    MAN_ID_0 = 0x1E,     // MAN_ID_0 register address
    MAN_ID_1 = 0x1F,     // MAN_ID_1 register address
    SHORT_ADDR_0 = 0x20, // SHORT_ADDR_0 register address
    SHORT_ADDR_1 = 0x21, // SHORT_ADDR_1 register address
    PAN_ID_0 = 0x22,     // PAN_ID_0 register address
    PAN_ID_1 = 0x23,     // PAN_ID_1 register address
    IEEE_ADDR_0 = 0x24,  // IEEE_ADDR_0 register address
    IEEE_ADDR_1 = 0x25,  // IEEE_ADDR_1 register address
    IEEE_ADDR_2 = 0x26,  // IEEE_ADDR_2 register address
    IEEE_ADDR_3 = 0x27,  // IEEE_ADDR_3 register address
    IEEE_ADDR_4 = 0x28,  // IEEE_ADDR_4 register address
    IEEE_ADDR_5 = 0x29,  // IEEE_ADDR_5 register address
    IEEE_ADDR_6 = 0x2A,  // IEEE_ADDR_6 register address
    IEEE_ADDR_7 = 0x2B,  // IEEE_ADDR_7 register address
    XAH_CTRL_0 = 0x2C,   // XAH_CTRL_0 register address
    CSMA_SEED_0 = 0x2D,  // CSMA_SEED_0 register address
    CSMA_SEED_1 = 0x2E,  // CSMA_SEED_1 register address
    CSMA_BE = 0x2F,      // CSMA_BE register address
    AES_STATUS = 0x82,   // AES_STATUS register address
}

// TRX_STATUS register
pub const TRX_STATUS_TRX_STATUS_MASK: u8 = 0x1F;
pub const TRX_STATUS_CCA_STATUS_MASK: u8 = 0x40;
pub const TRX_STATUS_CCA_DONE_MASK: u8 = 0x80;

// TRX_STATE register
pub const TRX_STATE_TRX_CMD_MASK: u8 = 0x1F;
pub const TRX_STATE_TRX_CMD_SHIFT: u8 = 0;
pub const TRX_STATE_TRAC_STATUS_MASK: u8 = 0xA0;
pub const TRX_STATE_TRAC_STATUS_SHIFT: u8 = 5;

// TRX_CTRL0
pub const TRX_CTRL0_CLKM_CTRL_MASK: u8 = 0x07;
pub const TRX_CTRL0_CLKM_CTRL_SHIFT: u8 = 0;
pub const TRX_CTRL0_CLKM_SHA_SEL_MASK: u8 = 0x08;
pub const TRX_CTRL0_CLKM_SHA_SEL_SHIFT: u8 = 3;
pub const TRX_CTRL0_PAD_IO_CLKM_MASK: u8 = 0x30;
pub const TRX_CTRL0_PAD_IO_CLKM_SHIFT: u8 = 4;
pub const TRX_CTRL0_PAD_IO_MASK: u8 = 0xC0;
pub const TRX_CTRL0_PAD_IO_SHIFT: u8 = 6;

// TRX_CTRL1
pub const TRX_CTRL1_IRQ_POLARITY_MASK: u8 = 0x01;
pub const TRX_CTRL1_IRQ_POLARITY_SHIFT: u8 = 0;
pub const TRX_CTRL1_IRQ_MASK_MODE_MASK: u8 = 0x02;
pub const TRX_CTRL1_IRQ_MASK_MODE_SHIFT: u8 = 1;
pub const TRX_CTRL1_SPI_CMD_MODE_MASK: u8 = 0x0C;
pub const TRX_CTRL1_SPI_CMD_MODE_SHIFT: u8 = 2;
pub const TRX_CTRL1_RX_BL_CTRL_MASK: u8 = 0x10;
pub const TRX_CTRL1_RX_BL_CTRL_SHIFT: u8 = 4;
pub const TRX_CTRL1_TX_AUTO_CRC_ON_MASK: u8 = 0x20;
pub const TRX_CTRL1_TX_AUTO_CRC_ON_SHIFT: u8 = 5;
pub const TRX_CTRL1_IRQ_2_EXT_EN_MASK: u8 = 0x40;
pub const TRX_CTRL1_IRQ_2_EXT_EN_SHIFT: u8 = 6;
pub const TRX_CTRL1_PA_EXT_EN_MASK: u8 = 0x80;
pub const TRX_CTRL1_PA_EXT_EN_SHIFT: u8 = 7;

// TRX_CTRL2
pub const TRX_CTRL2_OQPSK_DATA_RATE_MASK: u8 = 0x02;
pub const TRX_CTRL2_OQPSK_DATA_RATE_SHIFT: u8 = 0;
pub const TRX_CTRL2_SUB_MODE_MASK: u8 = 0x04;
pub const TRX_CTRL2_SUB_MODE_SHIFT: u8 = 2;
pub const TRX_CTRL2_BPSK_OQPSK_MASK: u8 = 0x08;
pub const TRX_CTRL2_BPSK_OQPSK_SHIFT: u8 = 3;
pub const TRX_CTRL2_ALT_SPECTRUM_MASK: u8 = 0x10;
pub const TRX_CTRL2_ALT_SPECTRUM_SHIFT: u8 = 4;
pub const TRX_CTRL2_OQPSK_SCRAM_EN_MASK: u8 = 0x20;
pub const TRX_CTRL2_OQPSK_SCRAM_EN_SHIFT: u8 = 5;
pub const TRX_CTRL2_TRX_OFF_AVDD_EN_MASK: u8 = 0x40;
pub const TRX_CTRL2_TRX_OFF_AVDD_EN_SHIFT: u8 = 6;
pub const TRX_CTRL2_RX_SAFE_MODE_MASK: u8 = 0x80;
pub const TRX_CTRL2_RX_SAFE_MODE_SHIFT: u8 = 7;

// PHY_CC_CCA
pub const PHY_CC_CCA_CHANNEL_MASK: u8 = 0x1F;
pub const PHY_CC_CCA_CHANNEL_SHIFT: u8 = 0;
pub const PHY_CC_CCA_CCA_MODE_MASK: u8 = 0x40;
pub const PHY_CC_CCA_CCA_MODE_SHIFT: u8 = 5;
pub const PHY_CC_CCA_CCA_REQ_MASK: u8 = 0x80;
pub const PHY_CC_CCA_CCA_REQ_SHIFT: u8 = 7;

// IRQ_STATUS
pub const IRQ_STATUS_IRQ_0_PLL_LOCK_MASK: u8 = 0x01;
pub const IRQ_STATUS_IRQ_0_PLL_LOCK_SHIFT: u8 = 0;
pub const IRQ_STATUS_IRQ_1_PLL_UNLOCK_MASK: u8 = 0x02;
pub const IRQ_STATUS_IRQ_1_PLL_UNLOCK_SHIFT: u8 = 1;
pub const IRQ_STATUS_IRQ_2_RX_START_MASK: u8 = 0x04;
pub const IRQ_STATUS_IRQ_2_RX_START_SHIFT: u8 = 2;
pub const IRQ_STATUS_IRQ_3_TRX_END_MASK: u8 = 0x08;
pub const IRQ_STATUS_IRQ_3_TRX_END_SHIFT: u8 = 3;
pub const IRQ_STATUS_IRQ_4_CCA_ED_DONE_MASK: u8 = 0x10;
pub const IRQ_STATUS_IRQ_4_CCA_ED_DONE_SHIFT: u8 = 4;
pub const IRQ_STATUS_IRQ_5_AMI_MASK: u8 = 0x20;
pub const IRQ_STATUS_IRQ_5_AMI_SHIFT: u8 = 5;
pub const IRQ_STATUS_IRQ_6_TRX_UR_MASK: u8 = 0x40;
pub const IRQ_STATUS_IRQ_6_TRX_UR_SHIFT: u8 = 6;
pub const IRQ_STATUS_IRQ_7_BAT_LOW_MASK: u8 = 0x80;
pub const IRQ_STATUS_IRQ_7_BAT_LOW_SHIFT: u8 = 7;

// CSMA_BE
pub const CSMA_BE_MIN_MASK: u8 = 0x0F;
pub const CSMA_BE_MIN_SHIFT: u8 = 0;
pub const CSMA_BE_MAX_MASK: u8 = 0xF0;
pub const CSMA_BE_MAX_SHIFT: u8 = 4;

// XAH_CTRL_0
pub const XAH_CTRL_SLOTTED_OPERATION_MASK: u8 = 0x01;
pub const XAH_CTRL_SLOTTED_OPERATION_SHIFT: u8 = 0;
pub const XAH_CTRL_MAX_CSMA_RETRIES_MASK: u8 = 0x0E;
pub const XAH_CTRL_MAX_CSMA_RETRIES_SHIFT: u8 = 1;
pub const XAH_CTRL_MAX_FRAME_RETRIES_MASK: u8 = 0xF0;
pub const XAH_CTRL_MAX_FRAME_RETRIES_SHIFT: u8 = 4;

// XAH_CTRL_1
pub const XAH_CTRL_1_AACK_PROM_MODE_MASK: u8 = 0x02;
pub const XAH_CTRL_1_AACK_PROM_MODE_SHIFT: u8 = 1;
pub const XAH_CTRL_1_AACK_ACK_TIME_MASK: u8 = 0x04;
pub const XAH_CTRL_1_AACK_ACK_TIME_SHIFT: u8 = 2;
pub const XAH_CTRL_1_AACK_UPLD_RES_FT_MASK: u8 = 0x10;
pub const XAH_CTRL_1_AACK_UPLD_RES_FT_SHIFT: u8 = 4;
pub const XAH_CTRL_1_AACK_FLTR_RES_FT_MASK: u8 = 0x20;
pub const XAH_CTRL_1_AACK_FLTR_RES_FT_SHIFT: u8 = 5;
pub const XAH_CTRL_1_CSMA_LBT_MODE_MASK: u8 = 0x40;
pub const XAH_CTRL_1_CSMA_LBT_MODE_SHIFT: u8 = 6;

// VREG_CTRL
pub const VREG_CTRL_DVDD_OK_MASK: u8 = 0x04;
pub const VREG_CTRL_DVDD_OK_SHIFT: u8 = 2;
pub const VREG_CTRL_DVREG_EXT_MASK: u8 = 0x08;
pub const VREG_CTRL_DVREG_EXT_SHIFT: u8 = 3;
pub const VREG_CTRL_AVDD_OK_MASK: u8 = 0x40;
pub const VREG_CTRL_AVDD_OK_SHIFT: u8 = 6;
pub const VREG_CTRL_AVREG_EXT_MASK: u8 = 0x80;
pub const VREG_CTRL_AVREG_EXT_SHIFT: u8 = 7;

// BATMON
pub const BATMON_BATMON_VTH_MASK: u8 = 0x0F;
pub const BATMON_BATMON_VTH_SHIFT: u8 = 0;
pub const BATMON_BATMON_HR_MASK: u8 = 0x10;
pub const BATMON_BATMON_HR_SHIFT: u8 = 4;
pub const BATMON_BATMON_OK_MASK: u8 = 0x20;
pub const BATMON_BATMON_OK_SHIFT: u8 = 5;
pub const BATMON_PLL_LOCK_MASK: u8 = 0x80;
pub const BATMON_PLL_LOCK_SHIFT: u8 = 7;

// PHY_TX_PWR
pub const PHY_TX_PWR_TX_PWR_MASK: u8 = 0x1F;
pub const PHY_TX_PWR_TX_PWR_SHIFT: u8 = 0;
pub const PHY_TX_PWR_GC_PA_MASK: u8 = 0x60;
pub const PHY_TX_PWR_GC_PA_SHIFT: u8 = 5;
pub const PHY_TX_PWR_PA_BOOST_MASK: u8 = 0x80;
pub const PHY_TX_PWR_PA_BOOST_SHIFT: u8 = 7;
