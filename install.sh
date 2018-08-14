#!/bin/sh
sudo mkdir /usr/src/mk_arcade_joystick_rpi-0.1.5.7
sudo cp -a * /usr/src/mk_arcade_joystick_rpi-0.1.5.7/
sudo apt-get install -y --force-yes dkms cpp-4.7 gcc-4.7 joystick raspberrypi-kernel-headers wiringpi
sudo dkms build -m mk_arcade_joystick_rpi -v 0.1.5.7
sudo dkms install -m mk_arcade_joystick_rpi -v 0.1.5.7
sudo sh -c 'echo "mk_arcade_joystick_rpi" >> /etc/modules'
#U,D,L,R BTN_START, BTN_SELECT, BTN_A, BTN_B, BTN_TR, BTN_Y, BTN_X, BTN_TL, BTN_MODE, BTN_TL2, BTN_TR2, BTN_C, BTN_Z
sudo sh -c 'echo "options mk_arcade_joystick_rpi map=4 gpio=4,17,6,5,19,26,16,24,23,18,15,14,-20,-1,-1,-1,-1 hkmode=2" >> /etc/modprobe.d/mk_arcade_joystick.conf'
sudo sh -c 'echo "#this next line is for use with 4 extra buttons (maybe L2,R2,C,Z)" >> /etc/modprobe.d/mk_arcade_joystick.conf'
sudo sh -c 'echo "#options mk_arcade_joystick_rpi map=4 gpio=4,17,6,5,19,26,16,24,23,18,15,14,-20,42,43,41,40 hkmode=2" >> /etc/modprobe.d/mk_arcade_joystick.conf'
sudo sh -c 'echo "#this next line is for use with a single PSP1000 analog stick" >> /etc/modprobe.d/mk_arcade_joystick.conf'
sudo sh -c 'echo "#options mk_arcade_joystick_rpi map=4 hkmode=2 i2cbus=1 x1addr=72 y1addr=77 gpio=4,17,6,5,19,26,16,24,23,18,15,14,-20,42,43,-1,41" >> /etc/modprobe.d/mk_arcade_joystick.conf'
