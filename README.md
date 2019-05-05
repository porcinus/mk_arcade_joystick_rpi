### DO NOT MERGE THIS BRANCH ###

Very experimental version:
- Implement of ADS1015 support.
- Implement of dead zone for Evdev.
- Creation of 'mk_joystick_config' program to allow end user to generation its own settings

### mk_joystick_config
This software can be use by End User to create a new configuration file for the driver itself, it take in account Analog and GPIO input.
Analog input part is able to detect the right ADC chip adress, minimum and maximum values of the axis, determinate if axis is reversed.
After running this program End User may need to remake Input configuration from EmulationStation but also restart its device in order to get everything working fine.

Important note about Hotkey button: some user doesn't set this button in order to user Select button when configuring input from EmulationStation settings menu, in this case, Power Slider need to be used in mk_joystick_config to allow user to bind Select to the right input.



