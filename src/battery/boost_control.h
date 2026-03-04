#ifndef FOCSTIM_BOOST_CONTROL_H
#define FOCSTIM_BOOST_CONTROL_H

#ifdef BOARD_FOCSTIM_V4


/**
 * Control the boost circuit and limit pulse voltage.
 *
 * On the V4, there are output capacitors fed by a boost circuit,
 * controllable in a range approximately 5v-30v. The voltage must be
 * set high enough for the desired output power, but also as low as
 * possible For efficiency reasons.
 *
 * We utilize:
 * (V_cap - V_drop) / max_duty_cycle = V_drive + margin
 * V_cap is the boost setpoint.
 * V_drop is the max observed drop in capacitor voltage during a pulse.
 * max_duty_cycle is a constant that depends on hardware.
 * V_drive is the maximum allowed peak-to-peak voltage of a pulse.
 * margin is about 1V.
 *
 * The amount V_drive is allowed to increase per-pulse is limited
 * to avoid crashing when the device is instructed to send heavy
 * pulses right after start.
 */
class BoostControl {
public:
    BoostControl();

    // call when start/stop to enable/disable the boost circuit
    // and reset slow start
    void play_started();
    void play_stopped();

    // call after every pulse
    void update(float pulse_vdrive, float boost_min);

    // check if boost caps are sufficiently charged
    bool boost_is_ready() const;

    // maximum v_drive with current settings
    float max_allowed_vdrive() const;

    // give an estimate how close the given v_drive is to the maximum
    // the hardware allows
    float utilizaton_percent(float voltage) const;

private:
    float boost_setpoint;
    float vdrive;   // the maximum requested drive voltage.
    float vdrop;    // maximum observed voltage drop during pulse playback.
    float vdrive_slow_start;
};

#endif
#endif