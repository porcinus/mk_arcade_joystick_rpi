## mk_joystick_config
Require: libpthread and libwiringpi in order to be compile.

This software can be use by End User to create a new configuration file for the driver itself, it take in account Analog and GPIO input. Analog input part is able to detect the right ADC chip adress, minimum and maximum values of the axis, determinate if axis is reversed. After running this program End User may need to remake Input configuration from EmulationStation but also restart its device in order to get everything working fine.

Important note about Hotkey button: some user doesn't set this button in order to user Select button when configuring input from EmulationStation settings menu, in this case, Power Slider need to be used in mk_joystick_config to allow user to bind Select to the right input.

### Usage
./mk_joystick_config -debug -maxnoise 60 -adcselect

Options:
-debug, enable some debug stuff [Optional]
-adcselect, force user to select ADC chip type [Optional]
-maxnoise, maximum noise allowed for ADC chip, relative from raw analog center, lower value than 60 could create false positive [Optional]

### History
- 0.1a : Initial testing release.
- 0.1b : Autodetection of ADC type fully implemented.

