Not finalized.

# Signal generation parameters

FOC-Stim uses the [T-Code](https://github.com/multiaxis/TCode-Specification) protocol for receiving commands. See source code for available axis and ranges.

# Boot process

The device will periodically print a status message `status: 4\r\n`. The status message contains information about the boot status, vbus and play status. See the source code for details.

Once the device has booted and vbus is up (indicated by status message), send `DSTART` to start pulse generation, `DSTOP` to stop.

Version string can be received with `D0`. It is highly recommended to check for an exact version string.

# Logs

The device will regularly send metrics, for example electrode resistance or device temperature information. Logs start with `$` and end with a newline. Example logline `$R_a:1.5 R_b:2.0 V_BUS:42\r\n`.

If an error occurs, unformatted debug information is printed. Errors can only be recovered by restarting the device.

These metrics are defined:

* `R_neutral`, `R_left`, `R_right`: estimated circuit resistance in ohm. Subtract the inductor and transformer resistance and multipy with the square of the winding ratio to get the skin resistance.
* `L`: estimated inductance in µH.
* `rms_neutral`, `rms_left`, `rms_right`: RMS current. Divide by the winding ratio to get the RMS current delivered to the skin.
* `V_bus`: power supply input voltage.
* `V_drive`: actual voltage used to drive the circuit. Limited to about 80% of `V_bus`.
* `F_pulse`: actual pulse frequency.
* `temp`: temperature of the onboard NTC in degrees celsius.
* `pot`: value of the potentiometer, in scale from 0 to 1.


# Keepalive

The device will stop generating pulses if no valid commands have been received for some time. Spam `DPING` or any other valid command to subvert.
