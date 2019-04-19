#!/bin/sh
sudo modprobe -r mk_arcade_joystick_rpi
sudo dkms remove -m mk_arcade_joystick_rpi -v 0.1.5.12 --all
sudo mkdir /usr/src/mk_arcade_joystick_rpi-0.1.5.12
sudo cp -a * /usr/src/mk_arcade_joystick_rpi-0.1.5.12/
sudo dkms build -m mk_arcade_joystick_rpi -v 0.1.5.12
sudo dkms install -m mk_arcade_joystick_rpi -v 0.1.5.12 --force
sudo modprobe mk_arcade_joystick_rpi

