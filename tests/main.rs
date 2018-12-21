//! AT86RF212 Radio Driver 
//! HITL testing
//! 
//! Copyright 2018 Ryan Kurte

use std::env;

extern crate embedded_hal;

extern crate linux_embedded_hal;
use linux_embedded_hal::{Spidev, Pin, Delay};
use linux_embedded_hal::spidev::{SpidevOptions, SPI_MODE_0};

extern crate radio_at86rf212;
use radio_at86rf212::{AT86RF212, Register, TrxCmd, TrxStatus};
use radio_at86rf212::device::defaults;

#[test]
fn test_devices() {
    let spi0_name = env::var("RADIO0_SPI")
        .expect("RADIO0_SPI environmental variable undefined");

    let spi1_name = env::var("RADIO1_SPI")
	.expect("RADIO1_SPI environmental variable undefined");

    let cs0_name = env::var("RADIO0_CS")
        .expect("RADIO0_CS environmental variable undefined");
    let cs1_name = env::var("RADIO1_CS")
        .expect("RADIO1_CS environmental variable undefined");

    let reset0_name = env::var("RADIO0_RESET")
        .expect("RADIO0_RESET environmental variable undefined");
    let reset1_name = env::var("RADIO1_RESET")
        .expect("RADIO1_RESET environmental variable undefined");

    let sleep0_name = env::var("RADIO0_SLEEP")
        .expect("RADIO0_SLEEP environmental variable undefined");
    let sleep1_name = env::var("RADIO1_SLEEP")
        .expect("RADIO1_SLEEP environmental variable undefined");

    println!("Connecting to peripherals");

    let mut spi0 = Spidev::open(spi0_name)
        .expect("Failed to open SPI0");

    let mut spi1 = Spidev::open(spi1_name)
	    .expect("Failed to open SPI1");

    let cs0 = Pin::from_path(cs0_name)
        .expect("Failed to open CS0");
    let cs1 = Pin::from_path(cs1_name)
        .expect("Failed to open CS1");

    let reset0 = Pin::from_path(reset0_name)
        .expect("Failed to open RESET0");
    let reset1 = Pin::from_path(reset1_name)
        .expect("Failed to open RESET1");

    let sleep0 = Pin::from_path(sleep0_name)
        .expect("Failed to open SLEEP0");
    let sleep1 = Pin::from_path(sleep1_name)
        .expect("Failed to open SLEEP1");

    println!("Configuring peripherals");
 
    let options = SpidevOptions::new()
         .bits_per_word(8)
         .max_speed_hz(20_000)
         .mode(SPI_MODE_0)
         .build();

    spi0.configure(&options).unwrap();
    spi1.configure(&options).unwrap();

    println!("Connecting to devices");
   
    let mut radio0 = AT86RF212::new(spi0, reset0, cs0, sleep0, Delay{})
        .expect("Failed to initialise radio0");

    let mut radio1 = AT86RF212::new(spi1, reset1, cs1, sleep1, Delay{})
	.expect("Failed to initialise radio1");

    println!("Test initial configuration");

    let val = radio0.reg_read(Register::PHY_CC_CCA)
        .expect("Failed reading PHY_CC_CCA register");
    assert_eq!((val >> 5 ) & 0x03, 1, "CCA mode");

    let val = radio0.reg_read(Register::CSMA_BE)
        .expect("Failed reading CSMA_BE register");
    assert_eq!(defaults::MINBE, (val >> 0) & 0x0f, "MINBE");
    assert_eq!(defaults::MAXBE, (val >> 4) & 0x0f, "MAXBE");

    let val = radio0.reg_read(Register::XAH_CTRL_0)
        .expect("Failed reading XAH_CTRL_0 register");
    assert_eq!(defaults::MAX_CSMA_BACKOFFS, (val >> 1) & 0x07, "CSMA backoffs");

    let val = radio0.reg_read(Register::XAH_CTRL_1)
        .expect("Failed reading XAH_CTRL_1 register");
    assert_eq!(1, (val >> 1) & 0x01, "Promiscuous mode auto ack enabled");

    let val = radio0.reg_read(Register::TRX_CTRL_1)
        .expect("Failed reading XAH_CTRL_1 register");
    assert_eq!(1, (val >> 5) & 0x01, "Auto CRC");
    assert_eq!(1, (val >> 1) & 0x01, "IRQ mask mode");


    println!("Testing set state");

    radio0.set_state_blocking(TrxCmd::RX_ON)
        .expect("Failed setting state to RX");

    let val = radio0.get_state()
        .expect("Failed fetching radio state");
    assert_eq!(TrxStatus::RX_ON as u8, val, "Radio state (RX)");

    radio0.set_state_blocking(TrxCmd::TRX_OFF)
        .expect("Failed setting state to TRX_OFF");

    let val = radio0.get_state()
        .expect("Failed fetching radio state");
    assert_eq!(TrxStatus::TRX_OFF as u8, val, "Radio state (TRX_OFF)");


    println!("Testing send/receive");
    radio0.start_rx(0).expect("Error starting receive");
    let val = radio0.get_state().expect("Failed fetching radio state");
    assert_eq!(TrxStatus::RX_ON as u8, val, "Radio state (RX_ON)");

    // TODO


}
