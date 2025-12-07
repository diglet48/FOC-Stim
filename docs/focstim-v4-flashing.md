![](images/focstim-v4-chips.jpg)

The FOC-Stim V4 contains 2 chips that need software to work, an ESP32 and STM32.

Only the switch is required to be soldered before flashing.

# The easy way

The first time flashing the board, you will need to hold the boot
button next to the respective chip while flipping the power switch.
This puts the chip in firmware update mode.

ESP32: Use the webflasher at https://github.com/diglet48/FOC-Stim-esp32-Webflasher

STM32: Start Restim, use tools -> firmware updater. Download firmware binary here: https://github.com/diglet48/FOC-Stim/releases

# Development

Development for both chips is done with Visual Studio Code.
Install the plugins PlatformIO and optionally Teleplot.

## ESP32

Get the code here: https://github.com/diglet48/FOC-Stim-esp32  
Make sure the correct environment is selected. (`focstim_v4_1` or `focstim_v4_0`).
Click the `upload` button in the sidebar.

V4.0 and V4.1 are not compatible.

## STM32

Get the code here: https://github.com/diglet48/FOC-Stim  
Make sure the correct environment is selected. (`focstim_v4`).

Use Restim or attach a debugger to upload the code to the board.

V4.0 and V4.1 use the same STM32 firmware.