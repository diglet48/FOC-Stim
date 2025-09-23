#if defined(BOARD_FOCSTIM_V3)
#ifndef FOCSTIM_CONFIG_G474RE_FOCSTIM_V3_H
#define FOCSTIM_CONFIG_G474RE_FOCSTIM_V3_H

// transformer winding ratio
#define STIM_WINDING_RATIO 6.66f    // xicon 42TL004
#define STIM_WINDING_RATIO_SQ (STIM_WINDING_RATIO * STIM_WINDING_RATIO)
// board resistance: 0.4ohm (inductor) + 1.06 ohm (transformer input) + 0.22ohm (max22213 Rdson, HFS on)
// high side 11.5 (transformer output)
// total 2ohm driving / 88.7ohm output
// measured value: 105 ohm ?


// current limits
#define BODY_CURRENT_MAX  0.15f  // in amps
#define ESTOP_CURRENT_LIMIT_MARGIN 0.15f                // accounts for measurement noise, driving current

// board temperature limits
#define MAXIMUM_TEMPERATURE 50.f    // degrees celsius.

// boost voltage
#define STIM_DYNAMIC_VOLTAGE
#define STIM_BOOST_VOLTAGE 22.f                     // the configured boost voltage
#define STIM_BOOST_VOLTAGE_OK_THRESHOLD 21.5f       // the minimum boost voltage required before starting a pulse
#define STIM_BOOST_VOLTAGE_LOW_THRESHOLD 15.f       // the lowest the boost voltage is allowed to drop during a pulse
#define STIM_BOOST_OVERVOLTAGE_THRESHOLD 23.f       // overvoltage threshold, estop if exceeded
#define STIM_BOOST_UNDERVOLTAGE_THRESHOLD 12.f      // the lowest the boost voltage is allowed to drop during a pulse, before triggering estop.

// pwm
#define STIM_PWM_FREQ 50000 // switching frequency is twice this frequency
#define STIM_PWM_MINIMUM_OFF_TIME 3e-6f // limited by pwm-rejection of current sense. Experimentally determined to be 2.6us.
#define STIM_PWM_MAX_DUTY_CYCLE float(1 - STIM_PWM_MINIMUM_OFF_TIME * STIM_PWM_FREQ)
#define STIM_PWM_MAX_VDRIVE (STIM_PWM_MAX_DUTY_CYCLE * STIM_BOOST_VOLTAGE_LOW_THRESHOLD)

// potentiometer volume control
#define POTMETER_ZERO_PERCENT_VALUE      0.f
#define POTMETER_HUNDRED_PERCENT_VALUE   float(4096 * .99f)

// initial conditions and limits for the model
#define MODEL_RESISTANCE_INIT (200.f / STIM_WINDING_RATIO_SQ)
#define MODEL_RESISTANCE_MIN (50.f / STIM_WINDING_RATIO_SQ)
#define MODEL_RESISTANCE_MAX (1500.f / STIM_WINDING_RATIO_SQ)
#define MODEL_PHASE_ANGLE_MIN -1.5f
#define MODEL_PHASE_ANGLE_MAX 1.5f

// enable for nicer looking waveforms on the scope. Not safe for humans!
// #define THREEPHASE_PULSE_DEFEAT_RANDOMIZATION

#define CURRENT_SENSE_SCALE_HALF
#define BSP_ENABLE_FOURPHASE
#define BATTERY_ENABLE
#define MAX22213_HFS 1

#endif
#endif