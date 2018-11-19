# rust-radio-at86rf212

A rust driver for the Atmel AT86RF212 Sub 1GHz ISM band radio IC.

## Status

[![Build status](https://badge.buildkite.com/ce94e220efbf89b7d77f787703dceff6147dbcbaeb1ead0272.svg)](https://buildkite.com/ryankurte/rust-radio-at86rf212)


## Testing

Testing is run via buildkite using a Raspberry Pi model 3 with an IO shield and a pair of XPlained PRO Zigbit + ATRF212B-0-U modules.

Pinout:
```
1.  ID
2.  GND
...
5.  nRST
...
9.  IRQ
10. SLP/TR
...
15. CS
16. MOSI
17. MISO
18. CLK
19. GND
20. VCC
```

