# RPI AT86RF212 test peripheral configuration

export RADIO0_SPI=/dev/spidev0.0
export RADIO1_SPI=/dev/spidev0.1

export RADIO0_CS=/sys/class/gpio/gpio4
export RADIO1_CS=/sys/class/gpio/gpio7

export RADIO0_RESET=/sys/class/gpio/gpio18
export RADIO1_RESET=/sys/class/gpio/gpio5

export RADIO0_SLEEP=/sys/class/gpio/gpio17
export RADIO1_SLEEP=/sys/class/gpio/gpio24

echo "4" >> /sys/class/gpio/export
echo "7" >> /sys/class/gpio/export
echo "18" >> /sys/class/gpio/export
echo "5" >> /sys/class/gpio/export
echo "17" >> /sys/class/gpio/export
echo "24" >> /sys/class/gpio/export

echo "out" >> /sys/class/gpio/gpio4/direction
echo "out" >> /sys/class/gpio/gpio7/direction

echo "out" >> /sys/class/gpio/gpio18/direction
echo "out" >> /sys/class/gpio/gpio5/direction

echo "out" >> /sys/class/gpio/gpio17/direction
echo "out" >> /sys/class/gpio/gpio24/direction

