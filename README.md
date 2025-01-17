FOC-Stim is an implementation of the [Restim](https://github.com/diglet48/restim) threephase signal generation algorithm
for the [B-G431B-ESC1](https://www.st.com/en/evaluation-tools/b-g431b-esc1.html) electronic speed controller.

FOC stands for Field Oriented Control, an algorithm for controlling motors, referencing the intended purpose of the hardware.

It utilizes MRAC (model reference adaptive control) to generate consistent current-controlled waveforms
as environment conditions change.

Powered by SimpleFOC.

# Hardware setup

BOM:

* B-G431B-ESC1
* USB cable
* 12v power supply (0.5A minimum)
* 3x transformer (XICON 42TU200-RC suggested)
* 3x inductor (220µH, 1A)
* 3x ceramic caps (10µF 25v)

When selecting an inductor, look for saturation current about 1A and flat inductance-current curve in the datasheet. Some options:
* KEMET SBCP-11HY221H. 80HY or 14HY series also a possibility.
* Fastron 09HVP or 09HCP series.
* Coilcraft RFB0810 or RFB1010 or RFS1317 series.
* Coilcraft DR0608-224L, usable if cramped for space but the saturation current isn't quite as high as I'd like.


A panel-mount USB micro-B is useful if you want to make this into a nice box.
[USB Micro-B](https://www.sparkfun.com/products/15464)
or [USB B](https://www.sparkfun.com/products/15463).

How to wire:

![](docs/images/schematic.png)

`U`, `V`, `W` are the 3 outputs on the board, with `U` being the one closest to the potentiometer.

The side of the transformer marked with black sharpie should be connected to the board-side.
The primary, indicated with a "P", is the output side.

![](docs/images/breadboard.jpg)

# Software setup

From the factory, the board has a very old ST-link firmware that prevents the usb-serial from working
correctly on some computers. It is highly recommended to update the ST-link firmware using
[STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) before proceeding.

Install Visual Studio Code with plugins `platformio` and `teleplot`,
then build and upload the firmware, instructions below.

Inspect `FOC-Stim/src/config.h` to configure the current limits.

![](docs/images/pio.png)

# Control

Control over serial with Restim.

View live stats with teleplot.

The onboard pot can be used to control the signal intensity.
If you only want to use software control, turn it all the way clockwise.
You can desolder the pot and use the pins to wire an external pot for your box.

All restim features are supported except: vibration.