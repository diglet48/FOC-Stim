The FOC-Stim contains 2 chips that need software to work, an ESP32 and STM32.

# ESP32

The ESP32 handles usb and wifi communication.

## build/upload from source

Clone this repository: https://github.com/diglet48/FOC-Stim-esp32

Open the project in visual studio code. 
Make sure the plugin platformio is installed.
Build and upload the software using the controls on the left-hand pane of the screen.

## disaster recovery

If you brick your ESP32 firmware, you can force
the chip into bootloader mode where you can upload the firmware as normal.

To do this, open your box and hold the `ESP32_PRG` button located next
to the USB port. Then turn on the power to the box.

# STM32

The STM32 handles all realtime control and user interface tasks.

## pre-built binaries

TODO

## build from source

Clone this repository. https://github.com/diglet48/FOC-Stim

Open the project in visual studio code.
Make sure the plugin platformio is installed.
Select the environment "focstim_v4" and build the software.

## upload with Restim

This requires your ESP32 to have valid firmware.

Open Restim. Go to tools -> firmware updater and follow the instructions.

## upload with STM32CubeProgrammer

This requires your ESP32 to have valid firmware.

Follow the disaster recovery instructions to put the chip in bootloader mode.
Then use STM32CubeProgrammer (UART mode) to upload the binary to the chip.

## upload the code with jtag probe

Connect your probe to the header marked "TJAG" with the cable sticking
towards the frontpanel connectors.

I had good luck with Segger J-link probes and Ozone.

## disaster recovery

If you do not have a jtag probe and brick your STM32, you can force the chip
into bootloader mode where you can upload the firmware as normal.

To do this, open your box and hold the `STM_BOOT` button located near the
frontpanel connector. Then turn on the power to the box.
