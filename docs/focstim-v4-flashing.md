![](images/focstim-v4-chips.jpg)

The FOC-Stim V4 contains 2 chips that need software to work, an ESP32 and STM32.

Each chip has a boot button, hold the button and flip the power switch
to put the chip in firmware update mode. This should only be required when
programming for the boards for the first time.

# The easy way

ESP32: Use the webflasher  at https://github.com/diglet48/FOC-Stim-esp32-Webflasher

STM32: Start Restim, use tools -> firmware updater. Download firmware binary here: https://github.com/diglet48/FOC-Stim/releases

# Development

Development for both chips is done with Visual Studio Code.
Install the plugins PlatformIO and optionally Teleplot.

## ESP32

Get the code here: https://github.com/diglet48/FOC-Stim-esp32  
Make sure the correct environment is selected. (`focstim_v4_1`).
Click the `upload` button in the sidebar.

# STM32

Get the code here: https://github.com/diglet48/FOC-Stim  
Make sure the correct environment is selected. (`focstim_v4`).

Use Restim or attach a debugger to upload the code to the board.