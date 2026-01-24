The FOC-Stim V4 has support for an erection sensor based on the AS5311.

Get the 3D files here: https://github.com/diglet48/toy-designs/tree/main/omega%20clamp

To construct this sensor and attach it to your FOC-Stim, you need these parts:

* modified rear panel [found in extras](/3d%20files/V4/readme.md).
* ams OSRAM AS5311-TS_EK_AB sensor board. 
* 3.3cm of magnet strip https://aliexpress.com/item/1005002805940724.html
* jtag breakout board https://aliexpress.com/item/1005004180810695.html
* 10cm 1.27mm 2x5 flatcable, for inside the box. https://aliexpress.com/item/1005008046702049.html
* 50cm 1.27mm 2x5 flatcable, male to female. https://aliexpress.com/item/1005006350473754.html
* 1.27mm 2x5 IDC header socket https://aliexpress.com/item/1005010396002233.html

# Build instructions

Embed your 10mm cable inside the rear panel like this. 
Inside the box, it will connect to the header marked "EXPD".

![](/docs/images/as5311-rear-panel.jpg)

Cut off part of the 50cm flatcable and swap wires 1/2, 3/4, 5/6, 7/8, 9/10.
Then crimp on a new header.

This needs to be done because the male-to-female cable mirrors the pins. It
is not needed if you plug the sensor with a simple female-to-female cable
directly into the mainboard.

![](/docs/images/as5311-wire-transposition.jpg)

Solder the breakout board with 4 2.54mm pins to the sensor.

Connect the line of (four) pins "vref, gnd, gnd, key" on the breakout board to
"gnd, DO, CLK, CSn" on the AS5311. Then use a small piece of wire to bridge
SWO to both 3v3 and 5v.

![](/docs/images/as5311-soldering-1.jpg)

![](/docs/images/as5311-soldering-2.jpg)

![](/docs/images/as5311-soldering-3.jpg)
