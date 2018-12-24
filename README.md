# rust-radio-at86rf212

A rust driver for the Atmel AT86RF212 Sub 1GHz ISM band radio IC, based on [ryankurte/libat86rf212](https://github.com/ryankurte/libat86rf212).

## Status

[![GitHub tag](https://img.shields.io/github/tag/ryankurte/rust-radio-at86rf212.svg)](https://github.com/ryankurte/rust-radio-at86rf212)
[![Build status](https://badge.buildkite.com/ce94e220efbf89b7d77f787703dceff6147dbcbaeb1ead0272.svg)](https://buildkite.com/ryankurte/rust-radio-at86rf212)
[![Crates.io](https://img.shields.io/crates/v/radio-at86rf212.svg)](https://crates.io/crates/radio-at86rf212)
[![Docs.rs](https://docs.rs/radio-at86rf212/badge.svg)](https://docs.rs/radio-at86rf212)

[Open Issues](https://github.com/ryankurte/rust-radio-at86rf212/issues)

**Work in Progress**

- [X] Register operations
- [X] Initialisation
- [X] Polling
- [X] Simple Send
- [X] Simple Receive
- [ ] Packet building & parsing
- [ ] Auto ACK
- [ ] Auto Retransmit
- [ ] Interrupt Mode
- [ ] DMA support
- [ ] Unit testing
- [x] Integration Testing

## Testing

Unit testing should be implemented using [dbrgn/embedded-hal-mock](https://github.com/dbrgn/embedded-hal-mock).

Integration testing is run using a Raspberry Pi model 3 with an IO shield and a pair of [XPlained PRO Zigbit + ATRF212B-0-U](http://ww1.microchip.com/downloads/en/devicedoc/atmel-42270-wireless-zigbit-atzb-rf-212b-0-u_datasheet.pdf) modules.

The RPi pins are configured at startup as in [rpi_setup.sh](rpi_setup.sh) and the environment is configured in [rpi_env.sh](rpi_env.sh) (though these are separately injected into the test system). Note that CS0 and CS1 functions are not currently used.

[buildkite](https://buildkite.com/ryankurte/rust-radio-at86rf212) with a custom worker is used to run integration tests against a physical hardware.

![at86rf212-test-setup](https://user-images.githubusercontent.com/860620/50320284-7cb02b00-0530-11e9-811f-2d256d614a57.jpg)

## Licensing

This project is licensed as GPLv3 for all purposes. For alternative licensing options / proprietary use, please contact the author (and we'll be happy to help ^_^).