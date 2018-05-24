#!/bin/sh
sudo mkdir /usr/src/mk_arcade_joystick_rpi-0.1.5.6
sudo cp -a * /usr/src/mk_arcade_joystick_rpi-0.1.5.6/
sudo apt-get install -y --force-yes dkms cpp-4.7 gcc-4.7 joystick raspberrypi-kernel-headers wiringpi
sudo dkms build -m mk_arcade_joystick_rpi -v 0.1.5.6
sudo dkms install -m mk_arcade_joystick_rpi -v 0.1.5.6
sudo sh -c 'echo "mk_arcade_joystick_rpi" >> /etc/modules'
#sudo sh -c 'echo "options mk_arcade_joystick_rpi map=4 gpio=4,17,6,5,19,26,16,24,23,18,15,14,-20,42,43,41,40 hkmode=2" > /etc/modprobe.d/mk_arcade_joystick.conf'
sudo sh -c 'echo "options mk_arcade_joystick_rpi map=4 gpio=4,17,6,5,19,26,16,24,23,18,15,14,-20,-1,-1,-1,-1 hkmode=2" > /etc/modprobe.d/mk_arcade_joystick.conf'

