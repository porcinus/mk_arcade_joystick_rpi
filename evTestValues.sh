#!/bin/bash

if [ -f ./output.txt ]; then
	export LINE_NUM=$(expr $(grep -n "Testing ... (interrupt to exit)" output.txt | cut -d ':' -f 1) + 1)
	tail -n +$LINE_NUM output.txt > cutOutput.txt
	mv cutOutput.txt output.txt

	grep ABS_X output.txt > LX.txt
	grep ABS_Y output.txt > LY.txt
	grep ABS_RX output.txt > RX.txt
	grep ABS_RY output.txt > RY.txt

	export EXT=.txt
	export NEW_PREFIX=new

	for FILE in LX LY RX RY
	do
		cut $FILE$EXT -d ' ' -f 11 > $NEW_PREFIX$FILE$EXT
		mv $NEW_PREFIX$FILE$EXT $FILE$EXT
		sort -n $FILE$EXT > $NEW_PREFIX$FILE$EXT
		declare -i MIN_$FILE=$(head -n 1 $NEW_PREFIX$FILE$EXT)
		declare -i MAX_$FILE=$(tail -n 1 $NEW_PREFIX$FILE$EXT)
	done

	MIN_LX=$(awk "BEGIN{print int(${MIN_LX}*1.1)}")
	MAX_LX=$(awk "BEGIN{print int(${MAX_LX}*0.9)}")

	MIN_LY=$(awk "BEGIN{print int(${MIN_LY}*1.1)}")
	MAX_LY=$(awk "BEGIN{print int(${MAX_LY}*0.9)}")

	MIN_RX=$(awk "BEGIN{print int(${MIN_RX}*1.1)}")
	MAX_RX=$(awk "BEGIN{print int(${MAX_RX}*0.9)}")

	MIN_RY=$(awk "BEGIN{print int(${MIN_RY}*1.1)}")
	MAX_RY=$(awk "BEGIN{print int(${MAX_RY}*0.9)}")

	printf "Consider using these values in /etc/modprobe.d/mk_arcade_joystick.conf"
	printf "\n\tx1params=$MIN_LX,$MAX_LX,16,300"
	printf "\n\tx2params=$MIN_RX,$MAX_RX,16,300"
	printf "\n\ty1params=$MIN_LY,$MAX_LY,16,300"
	printf "\n\ty2params=$MIN_RY,$MAX_RY,16,300\n"
else
	printf "File \"output.txt\" not found.\nMake sure to do the follwing before running this script:"
	printf "\n\tEnable I2C by running \"sudo raspi-config\", then going to Interfacing -> I2C, selecting Yes to turn it on"
	printf "\n\tRun Porcinus' I2C mapping script to setup your /etc/modprobe.d/mk_arcade_joystick.conf, rebooting to enable the settings"
	printf "\n\tRun \"evtest /dev/input/event0 > output.txt\", slowly rotating each analog stick along its outermost edge before terminating with Control-C\n"
fi

exit 0
