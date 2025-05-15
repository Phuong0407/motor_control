#!/bin/bash
##servo
echo 5 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio5/direction

## moteur 1 : 
echo 13 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio13/direction
echo 12 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio12/direction
echo 26 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio26/direction
echo 27 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio27/direction

## moteur 2 : 
echo 23 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio23/direction
echo 22 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio22/direction
echo 18 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio18/direction
echo 19 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio19/direction


## moteur 3 : 
echo 6 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio6/direction
echo 5 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio5/direction
echo 16 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio16/direction
echo 17 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio17/direction



