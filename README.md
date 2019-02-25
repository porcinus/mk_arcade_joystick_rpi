This version of mk_arcade_joystick_rpi is intended for use with the Freeplay Zero and Freeplay CM3 DIY kits that allow you to build a Raspberry Pi Zero or Raspberry Pi Compute Module 3 into a handheld portable unit often used for retro gaming.

### Installation ###

### Installation Script ###

The install.sh included with this repository should build and install the driver.  Prior to running this script, you will likely want to install/update your kernel and kernel headers, just to be sure that they are current.

sudo apt-get install -y --force-yes raspberrypi-kernel raspberrypi-kernel-headers
sudo shutdown -h now

(REBOOT, and after reboot, change to this repository's directory)

./install.sh


### Driver Options ###

Please see install.sh (or /etc/modprobe.d/mk_arcade_joystick.conf) for examples.

This command will list the available parameters.
modinfo mk_arcade_joystick_rpi


### Testing/Calibrating ###

Use the following commands to test (or dump) joysticks inputs :
```shell
jstest /dev/input/js0
jscal -c /dev/input/js0
evtest
evdev-joystick --showcal /dev/input/event0
```

Joystick min, max, fuzz, flat can be set in /etc/modprobe.d/mk_arcade_joystick.conf or you can use the following commands to configure joysticks inputs on the fly :
```shell
evdev-joystick --evdev /dev/input/event0 --axis 0 --minimum 374 --maximum 3418 --deadzone 384 --fuzz 16
evdev-joystick --evdev /dev/input/event0 --axis 1 --minimum 517 --maximum 3378 --deadzone 384 --fuzz 16
```

# mk_arcade_joystick_rpi #

*** The information here is mainly historic from the previous version.  The version here likely works differently. ***

The Raspberry Pi GPIO Joystick Driver

Other versions of mk_arcade_joystick_rpi are fully integrated in the **recalbox** distribution : see http://www.recalbox.com

** Please see the [Recalbox mk_arcade_joystick_rpi](https://github.com/recalbox/mk_arcade_joystick_rpi/) repository for more info**

## Revisions ##

UPDATE 0.1.5.10 : Added support for analog parameters (min, max, fuzz, flat)

UPDATE 0.1.5.9 : Added support for 4 extra GPIO inputs

UPDATE 0.1.5.8 : Added some new parameters to allow inverting of analog inputs

UPDATE 0.1.5.7 : Added MCP3021 i2c support for up to 4 analog inputs

UPDATE 0.1.5.6 : Added 4 buttons, the ability to use GPIO32 and higher, and hkmode parameter

UPDATE 0.1.5.5 : Modified for use with Freeplay Zero/CM3

UPDATE 0.1.5 : Added GPIO customization

UPDATE 0.1.4 : Compatibily with rpi2 

UPDATE 0.1.3 : Compatibily with 3.18.3 :

The driver installation now works with 3.18.3 kernel, distributed with the last firmware.

UPDATE 0.1.2 : Downgrade to 3.12.28+ :

As the module will not load with recent kernel and headers, we add the possibility of downgrading your firmware to a compatible version, until we find a fix.

UPDATE 0.1.1 : RPi B+ VERSION :

The new Raspberry Pi B+ Revision brought us 9 more GPIOs, so we are now able to connect 2 joysticks and 12 buttons directly to GPIOs. I updated the driver in order to support the 2 joysticks on GPIO configuration.

## The Software ##
The joystick driver is based on the gamecon_gpio_rpi driver by [marqs](https://github.com/marqs85)


Credits
-------------
-  [gamecon_gpio_rpi](https://github.com/petrockblog/RetroPie-Setup/wiki/gamecon_gpio_rpi) by [marqs](https://github.com/marqs85)
-  [recalbox mk_arcade_joystick_rpi](https://github.com/recalbox/mk_arcade_joystick_rpi)
-  [RetroPie-Setup](https://github.com/petrockblog/RetroPie-Setup) by [petRockBlog](http://blog.petrockblock.com/)
-  [Low Level Programming of the Raspberry Pi in C](http://www.pieter-jan.com/node/15) by [Pieter-Jan](http://www.pieter-jan.com/)
