![](images/focstim-v4-complete.jpg)

Schematics, PCB and case design by Sekizen.  
Software by Diglet.

# Useful pages

* [Schematics / Gerbers](/schematics/V4/README.md)
* [Bill of materials](focstim-v4-BOM.md)
* [3D files](../3d%20files/V4)
* [Assembly guide](focstim-v4-assembly.md)
* [Flashing your board](focstim-v4-flashing.md)

Schematics/gerbers coming at a later date.

# Nerd zone

* [Output specifications](focstim-v4-output-specifications.md)

# Changelog

## V4.2
Added LSM6DSOX 6-DOF IMU.  
Added cutout for ESP32 antenna.  
Change the part number of the boot buttons to save space.  
Change the part number of the boost inductor due to parts shortage.  
Add ground spokes to some solder pads.  
Modified board outline to reduce abrasion on frontpanel.

## V4.1
Initial public version.  
Changed most pin assignments on the ESP32.  
Adjust switch circuit to avoid inrush current killing the TSP631000.  
Modify frontpanel dimensions for extra creepage/clearance.  

## V4.0
Not publicly released, switched to DRV8231A drivers.

## V3.0
Not publicly released, based on MAX22213 drivers.