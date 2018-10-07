use std::env;

extern crate embedded_hal;

extern crate linux_embedded_hal;
use linux_embedded_hal::{Spidev, Pin, Delay};

extern crate shared_bus;
use shared_bus::BusManager;

extern crate radio_at86rf212;
use radio_at86rf212::{AT86RF212, Register, TrxCmd, TrxStatus};
use radio_at86rf212::device::defaults;

#[test]
fn test_devices() {
    let spi_name = env::var("RADIO_SPI")
        .expect("RADIO_SPI environmental variable undefined");

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

    let mut spi = Spidev::open(spi_name)
        .expect("Failed to open SPI");

    let mut cs0 = Pin::from_path(cs0_name)
        .expect("Failed to open CS0");
    let mut cs1 = Pin::from_path(cs1_name)
        .expect("Failed to open CS1");

    let mut reset0 = Pin::from_path(reset0_name)
        .expect("Failed to open RESET0");
    let mut reset1 = Pin::from_path(reset1_name)
        .expect("Failed to open RESET1");

    let mut sleep0 = Pin::from_path(sleep0_name)
        .expect("Failed to open SLEEP0");
    let mut sleep1 = Pin::from_path(sleep1_name)
        .expect("Failed to open SLEEP1");


    println!("Connecting to devices");
    
    // Create shared bus manager
    let manager = BusManager::<std::sync::Mutex<_>, _>::new(spi);

    let mut spi0 = manager.acquire();
    let mut radio0 = AT86RF212::new(spi0, reset0, cs0, sleep0, [None; 4], Delay{})
        .expect("Failed to initialise radio0");

    let mut spi1 = manager.acquire();
    let mut radio0 = AT86RF212::new(spi1, reset1, cs1, sleep1, [None; 4], Delay{})
        .expect("Failed to initialise radio0");


    println!("Test initial configuration");

    let val = radio0.reg_read(Register::PHY_CC_CCA)
        .expect("Failed reading PHY_CC_CCA register");
    assert_eq!((val >> 5 ) & 0x03, 1, "CCA mode");

    let val = radio.read_reg(Register::CSMA_BE)
        .expect("Failed reading CSMA_BE register");
    assert_eq!(defaults::MINBE, (val >> 0) & 0x0f, "MINBE");
    assert_eq!(defaults::MAXBE, (val >> 4) & 0x0f, "MAXBE");

    let val = radio.read_reg(Register::XAH_CTRL_0)
        .expect("Failed reading XAH_CTRL_0 register");
    assert_eq!(defaults::MAX_CSMA_BACKOFFS, (val >> 1) & 0x07, "CSMA backoffs");

    let val = radio0.reg_read(Register::XAH_CTRL_1)
        .expect("Failed reading XAH_CTRL_1 register");
    assert_eq!(1, (val >> 1) & 0x0, "Promiscuous mode");

    let val = radio0.reg_read(Register::TRX_CTRL_1)
        .expect("Failed reading XAH_CTRL_1 register");
    assert_eq!(1, (val >> 5) & 0x01, "Auto CRC");
    assert_eq!(1, (val >> 1) & 0x01, "IRQ mask mode");


    println!("Testing set state");

    radio0.set_state_blocking(TrxCmd::RX_ON)
        .expect("Failed setting state to RX");

    let val = radio0.get_state()
        .expect("Failed fetching radio state");
    assert_eq!(TrxStatus::RX_ON, val, "Radio state (RX)");

    radio0.set_state_blocking(TrxCmd::TRX_OFF)
        .expect("Failed setting state to TRX_OFF");

    let val = radio0.get_state()
        .expect("Failed fetching radio state");
    assert_eq!(TrxStatus::TRX_OFF, val, "Radio state (TRX_OFF)");


    

}
