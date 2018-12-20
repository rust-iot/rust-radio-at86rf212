# RPI AT86RF212 test peripheral configuration

export RADIO0_SPI=/dev/spidev0.0
export RADIO1_SPI=/dev/spidev0.1

export RADIO0_CS=/sys/class/gpio/gpio4
export RADIO1_CS=/sys/class/gpio/gpio5

export RADIO0_RESET=/sys/class/gpio/gpio18
export RADIO1_RESET=/sys/class/gpio/gpio24

export RADIO0_SLEEP=/sys/class/gpio/gpio17
export RADIO1_SLEEP=/sys/class/gpio/gpio22

