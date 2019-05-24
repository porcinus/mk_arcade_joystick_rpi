obj-m := mk_arcade_joystick_rpi.o
KVER ?= $(shell uname -r)

ifneq (,$(findstring -v7, $(KVER)))
CFLAGS_mk_arcade_joystick_rpi.o := -DRPI2
endif

all:
	$(MAKE) -C /lib/modules/$(KVER)/build M=$(PWD) modules

clean:
	$(MAKE) -C /lib/modules/$(KVER)/build M=$(PWD) clean
