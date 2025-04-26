# Hardware

For component selection, refer to the [Bill of materials](/docs/focstim-v1-BOM.md).

How to wire:

![](/docs/images/schematic.png)

`U`, `V`, `W` are the 3 outputs on the board, with `U` being the one closest to the potentiometer.

The side of the transformer marked with black sharpie should be connected to the board-side.
The primary, indicated with a "P", is the output side.

![](/docs/images/breadboard.jpg)

# Software

From the factory, the board has a very old ST-link firmware that prevents the usb-serial from working
correctly on some computers. It is highly recommended to update the ST-link firmware using
[STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) before proceeding.

Install Visual Studio Code with plugins `platformio` and `teleplot`,
then build and upload the firmware, instructions below.

For this hardware, you must select project `env:disco_b_g431b_esc1`.

If you find that the box is not powerful enough, the current limit can be increased by modifying `TCODE_MAX_CURRENT` in `FOC-Stim/src/bsp/config_g431b_esc1.h`.


![](/docs/images/pio.png)

# Control

Control over USB with Restim.

View live stats with teleplot.

The onboard pot can be used to control the signal intensity.
If you only want to use software control, turn it all the way clockwise.
You can desolder the pot and use the pins to wire an external pot for your box.

All restim features are supported except: vibration.
