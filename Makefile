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
	mk_joystick_config -maxnoise 60 -adcselect
	sudo mv /opt/retropie/configs/all/emulationstation/es_input.cfg /opt/retropie/configs/all/emulationstation/es_input.cfg.bak
	echo SYSTEM SHUTTING DOWN NOW
	sudo reboot
