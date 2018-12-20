# RPI AT86RF212 test peripheral configuration

echo "4" >> /sys/class/gpio/export
echo "5" >> /sys/class/gpio/export
echo "18" >> /sys/class/gpio/export
echo "24" >> /sys/class/gpio/export
echo "17" >> /sys/class/gpio/export
echo "22" >> /sys/class/gpio/export
echo "27" >> /sys/class/gpio/export
echo "23" >> /sys/class/gpio/export


echo "out" >> /sys/class/gpio/gpio4/direction
echo "out" >> /sys/class/gpio/gpio5/direction

echo "out" >> /sys/class/gpio/gpio18/direction
echo "out" >> /sys/class/gpio/gpio24/direction

echo "out" >> /sys/class/gpio/gpio17/direction
echo "out" >> /sys/class/gpio/gpio22/direction

echo "in" >> /sys/class/gpio/gpio27/direction
echo "in" >> /sys/class/gpio/gpio23/direction

