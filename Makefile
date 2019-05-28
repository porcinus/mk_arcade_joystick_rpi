obj-m := mk_arcade_joystick_rpi.o
KVER ?= $(shell uname -r)

ifneq (,$(findstring -v7, $(KVER)))
CFLAGS_mk_arcade_joystick_rpi.o := -DRPI2
endif

all:
	$(MAKE) -C /lib/modules/$(KVER)/build M=$(PWD) modules

clean:
	$(MAKE) -C /lib/modules/$(KVER)/build M=$(PWD) clean

config:
	$(MAKE) -o mk_joystick_config mk_joystick_config.cpp -lwiringPi -lpthread
	sudo mv /etc/modprobe.d/mk_arcade_joystick.conf /etc/modprobe.d/mk_arcade_joystick.conf.bak
	mk_joystick_config -maxnoise 60 -adcselect
	echo SYSTEM SHUTTING DOWN NOW
	sudo reboot
