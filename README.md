This version of mk_arcade_joystick_rpi is intended for use with the Freeplay Zero and Freeplay CM3 DIY kits that allow you to build a Raspberry Pi Zero or Raspberry Pi Compute Module 3 into a handheld portable unit often used for retro gaming.

# mk_arcade_joystick_rpi #

*** The information here is mainly historic from the previous version.  The version here likely works differently. ***

The Raspberry Pi GPIO Joystick Driver

Other versions of mk_arcade_joystick_rpi are fully integrated in the **recalbox** distribution : see http://www.recalbox.com

** Please see the [Recalbox mk_arcade_joystick_rpi](https://github.com/recalbox/mk_arcade_joystick_rpi/) repository for more info**

## Revisions ##

UPDATE 0.1.5.7 : Added MCP3021A i2c support for up to 4 analog inputs

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

## Common Case : Joysticks connected to GPIOs ##


### Pinout ###
Let's consider a 6 buttons cab panel with this button order : 

     ↑   Ⓨ Ⓧ Ⓛ  
    ← →	 Ⓑ Ⓐ Ⓡ  
     ↓  

With R = TR and L = TL


Here is the rev B GPIO pinout summary :

![GPIO Interface](https://github.com/recalbox/mk_arcade_joystick_rpi/raw/master/wiki/images/mk_joystick_arcade_GPIOs.png)

If you have a Rev B+ RPi or RPi2:


![GPIO Interface](https://github.com/recalbox/mk_arcade_joystick_rpi/raw/master/wiki/images/mk_joystick_arcade_GPIOsb+.png)

Of course the ground can be common for all switches.

### Installation ###

### Installation Script ###

The install.sh included with this repository should build and install the driver.


### Driver Options ###

Please see install.sh (or /etc/modprobe.d/mk_arcade_joystick.conf) for examples.

This command will list the available parameters.
modinfo mk_arcade_joystick_rpi


### Testing ###

Use the following command to test joysticks inputs :
```shell
jstest /dev/input/js0
```



Credits
-------------
-  [gamecon_gpio_rpi](https://github.com/petrockblog/RetroPie-Setup/wiki/gamecon_gpio_rpi) by [marqs](https://github.com/marqs85)
-  [RetroPie-Setup](https://github.com/petrockblog/RetroPie-Setup) by [petRockBlog](http://blog.petrockblock.com/)
-  [Low Level Programming of the Raspberry Pi in C](http://www.pieter-jan.com/node/15) by [Pieter-Jan](http://www.pieter-jan.com/)
