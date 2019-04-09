//! At86Rf212 Radio Driver 
//! HITL testing
//! 
//! Copyright 2018 Ryan Kurte

//#![feature(await_macro, futures_api)]

use std::{env, fmt, thread};
use std::time::Duration;

extern crate tokio;
use tokio::runtime::Builder;

extern crate futures;
use futures::prelude::*;

extern crate simplelog;
use simplelog::{TermLogger, LevelFilter, Config as LogConfig};

extern crate embedded_hal;
use embedded_hal::{digital, blocking::spi};

extern crate linux_embedded_hal;
use linux_embedded_hal::{Delay};

extern crate remote_hal;
use remote_hal::common::{PinMode, SpiMode};
use remote_hal::{manager::Manager, remote::Client, remote_addr};
use remote_hal::remote::{InitRequest};

extern crate radio;
use radio::{Transmit, Receive, Registers};

extern crate radio_at86rf212;
use radio_at86rf212::{At86Rf212, Register, TrxCmd, TrxStatus};
use radio_at86rf212::device::defaults;


#[test]
fn test_devices() {

    let _ = TermLogger::init(LevelFilter::Info, LogConfig::default()).unwrap();

    let spi0_name = env::var("RADIO0_SPI").expect("RADIO0_SPI environmental variable undefined");

    let spi1_name = env::var("RADIO1_SPI").expect("RADIO1_SPI environmental variable undefined");

    let cs0_name = env::var("RADIO0_CS").expect("RADIO0_CS environmental variable undefined");
    let cs1_name = env::var("RADIO1_CS").expect("RADIO1_CS environmental variable undefined");

    let reset0_name = env::var("RADIO0_RESET").expect("RADIO0_RESET environmental variable undefined");
    let reset1_name = env::var("RADIO1_RESET").expect("RADIO1_RESET environmental variable undefined");

    let sleep0_name = env::var("RADIO0_SLEEP").expect("RADIO0_SLEEP environmental variable undefined");
    let sleep1_name = env::var("RADIO1_SLEEP").expect("RADIO1_SLEEP environmental variable undefined");

    let mut rt = Builder::new()
    .blocking_threads(2)
    .core_threads(2)
    .build()
    .unwrap();

    let worker = futures::lazy(move || {

        Client::new(remote_addr()).map_err(|e| panic!(e) )
        .and_then(move |mut c|  {

            let reqs = vec![
                InitRequest::Spi{path: spi0_name, baud: 20_000, mode: SpiMode::Mode0},
                InitRequest::Spi{path: spi1_name, baud: 20_000, mode: SpiMode::Mode0},
                InitRequest::Pin{path: reset0_name, mode: PinMode::Output},
                InitRequest::Pin{path: reset1_name, mode: PinMode::Output},
                InitRequest::Pin{path: cs0_name, mode: PinMode::Output},
                InitRequest::Pin{path: cs1_name, mode: PinMode::Output},
                InitRequest::Pin{path: sleep0_name, mode: PinMode::Output},
                InitRequest::Pin{path: sleep1_name, mode: PinMode::Output},
            ];

            c.init_all(&reqs).map(move |devs| (c, devs) )
        })
        .and_then(move |(c, mut devs)| {

            println!("Connecting to peripherals");

            devs.reverse();
            let spi0 = devs.pop().unwrap().spi().unwrap();
            let spi1 = devs.pop().unwrap().spi().unwrap();

            let reset0 = devs.pop().unwrap().pin().unwrap();
            let reset1 = devs.pop().unwrap().pin().unwrap();

            let cs0 = devs.pop().unwrap().pin().unwrap();
            let cs1 = devs.pop().unwrap().pin().unwrap();

            let sleep0 = devs.pop().unwrap().pin().unwrap();
            let sleep1 = devs.pop().unwrap().pin().unwrap();


            println!("Running tests");

            run_tests(spi0, spi1, reset0, reset1, cs0, cs1, sleep0, sleep1);

            Ok(())

        })

    });
    
    rt.block_on(worker.map(|_| () ).map_err(|e| panic!(e) )).unwrap();
}

fn run_tests<S, O, E>(spi0: S, spi1: S, reset0: O, reset1: O, cs0: O, cs1: O, sleep0: O, sleep1: O) 
where
    S: spi::Transfer<u8, Error=E> + spi::Write<u8, Error=E>,
    O: digital::OutputPin,
    E: fmt::Debug,
{

    println!("Connecting to devices");
   
    let mut radio0 = At86Rf212::new(spi0, reset0, cs0, sleep0, Delay{})
        .expect("Failed to initialise radio0");

    let mut radio1 = At86Rf212::new(spi1, reset1, cs1, sleep1, Delay{})
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
    
    // Start RX
    let ch = 1;
    radio0.start_receive(ch).expect("Error starting receive");
    let val = radio0.get_state().expect("Failed fetching radio state");
    assert_eq!(TrxStatus::RX_ON as u8, val, "Radio state error (RX_ON)");

    // Start send
    let send = [0x11, 0x22, 0x33];
    radio1.start_transmit(ch, &send).expect("Error starting TX");

    // Poll on tx and rx complete
    let mut sent = false;
    let mut received = false;
    let mut buff = [0u8; 1024];

    for _i in 0..10 {
        if radio1.check_transmit().expect("Failed checking radio tx complete") {
            println!("Send complete ({:?})", send);
            sent = true;
        }
        if let Some((recv, _)) = radio0.get_received(&mut buff).expect("Failed checking radio rx complete") {
            println!("Receive complete ({:?})", recv);
            received = true;
        }
        thread::sleep(Duration::from_millis(100));
    }

    assert!(sent, "Send not completed");
    assert!(received, "Receive not completed")
}

