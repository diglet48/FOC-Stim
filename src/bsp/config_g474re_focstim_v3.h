#if defined(ARDUINO_FOCSTIM_V3)
#ifndef FOCSTIM_CONFIG_G474RE_FOCSTIM_V3_H
#define FOCSTIM_CONFIG_G474RE_FOCSTIM_V3_H

// transformer winding ratio
#define STIM_WINDING_RATIO 6.66f    // xicon 42TL004
#define STIM_WINDING_RATIO_SQ (STIM_WINDING_RATIO * STIM_WINDING_RATIO)

// current limits
#define BODY_CURRENT_MAX  0.12f  // in amps
#define ESTOP_CURRENT_LIMIT_MARGIN 0.15f                // accounts for measurement noise, driving current

// board temperature limits
#define MAXIMUM_TEMPERATURE 50.f    // degrees celsius.

// boost voltage
#define STIM_DYNAMIC_VOLTAGE
#define STIM_BOOST_VOLTAGE 18.f                     // the configured boost voltage
#define STIM_BOOST_VOLTAGE_OK_THRESHOLD 17.5f       // the minimum boost voltage required before starting a pulse
#define STIM_BOOST_VOLTAGE_LOW_THRESHOLD 12.f       // the lowest the boost voltage is allowed to drop during a pulse
#define STIM_BOOST_OVERVOLTAGE_THRESHOLD 19.f       // overvoltage threshold, estop if exceeded
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
#define MODEL_RESISTANCE_INIT 8.0f
#define MODEL_RESISTANCE_MIN 0.7f
#define MODEL_RESISTANCE_MAX 15.0f
#define MODEL_INDUCTANCE_INIT 450e-6f
#define MODEL_INDUCTANCE_MIN 20e-6f
#define MODEL_INDUCTANCE_MAX 1500e-6f
#define MODEL_PHASE_ANGLE_MIN -1.f
#define MODEL_PHASE_ANGLE_MAX 1.f


// enable for nicer looking waveforms on the scope. Not safe for humans!
// TODO: implement
// #define THREEPHASE_PULSE_DEFEAT_RANDOMIZATION

#define CURRENT_SENSE_SCALE_HALF
// #define BSP_ENABLE_THREEPHASE
#define BSP_ENABLE_FOURPHASE
#define MAX22213_HFS 1

#endif
#endif