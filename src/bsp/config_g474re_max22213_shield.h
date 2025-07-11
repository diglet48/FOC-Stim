#if defined(ARDUINO_NUCLEO_G474RE)
#ifndef FOCSTIM_CONFIG_G474RE_MAX22213_SHIELD_H
#define FOCSTIM_CONFIG_G474RE_MAX22213_SHIELD_H

// transformer winding ratio
#define STIM_WINDING_RATIO 10.0f    // xicon 42TU200, middle pin on driving side
#define STIM_WINDING_RATIO_SQ (STIM_WINDING_RATIO * STIM_WINDING_RATIO)

// current limits
#define BODY_CURRENT_MAX  0.12f              // in amps
#define ESTOP_CURRENT_LIMIT_MARGIN 0.15f     // accounts for measurement noise, driving current

// board temperature limits
#define MAXIMUM_TEMPERATURE 100.f    // degrees celsius.

// supply voltage
#define STIM_PSU_VOLTAGE 12.0f
#define STIM_PSU_VOLTAGE_MIN 11.5f // e-stop if exceeded
#define STIM_PSU_VOLTAGE_MAX 13.0f // e-stop if exceeded

// pwm
#define STIM_PWM_FREQ 50000 // switching frequency is twice this frequency
#define STIM_PWM_MINIMUM_OFF_TIME 3e-6f // limited by pwm-rejection of current sense. Experimentally determined to be 2.6us.
#define STIM_PWM_MAX_DUTY_CYCLE float(1 - STIM_PWM_MINIMUM_OFF_TIME * STIM_PWM_FREQ)

// potentiometer volume control
#define POTMETER_ZERO_PERCENT_VOLTAGE      3.3f
#define POTMETER_HUNDRED_PERCENT_VOLTAGE   0.0f

// initial conditions and limits for the model
#define MODEL_RESISTANCE_INIT 8.0f
#define MODEL_RESISTANCE_MIN 0.7f
#define MODEL_RESISTANCE_MAX 15.0f
#define MODEL_PHASE_ANGLE_MIN -1.f
#define MODEL_PHASE_ANGLE_MAX 1.f


// enable for nicer looking waveforms on the scope. Not safe for humans!
// #define THREEPHASE_PULSE_DEFEAT_RANDOMIZATION

#define CURRENT_SENSE_SCALE_HALF
// #define BSP_ENABLE_THREEPHASE
#define BSP_ENABLE_FOURPHASE
#define MAX22213_HFS 1

#endif
#endif