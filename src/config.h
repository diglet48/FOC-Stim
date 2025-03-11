#ifndef FOCSTIM_CONFIG_H
#define FOCSTIM_CONFIG_H

// current limits
#define TCODE_MAX_CURRENT 1.2f              // in amps
#define ESTOP_CURRENT_LIMIT_MARGIN 0.3f     // accounts for measurement noise

// board temperature limits
#define MAXIMUM_TEMPERATURE 100.f    // degrees celsius.

// supply voltage and pwm
#define STIM_PWM_FREQ 50000 // switching frequency is twice this frequency
#define STIM_PSU_VOLTAGE 12.0f
#define STIM_PSU_VOLTAGE_MIN 11.5f // e-stop if exceeded
#define STIM_PSU_VOLTAGE_MAX 15.0f // e-stop if exceeded
#define STIM_PWM_MINIMUM_OFF_TIME 3e-6f // limited by pwm-rejection of current sense. Experimentally determined to be 2.6us.
#define STIM_PWM_MAX_DUTY_CYCLE float(1 - STIM_PWM_MINIMUM_OFF_TIME * STIM_PWM_FREQ)

// potentiometer volume control
#define POTMETER_ZERO_PERCENT_VOLTAGE      3.3f
#define POTMETER_HUNDRED_PERCENT_VOLTAGE   0.0f

// initial conditions and limits for the model
#define MODEL_RESISTANCE_INIT 2.0f
#define MODEL_RESISTANCE_MIN 0.7f
#define MODEL_RESISTANCE_MAX 15.0f
#define MODEL_INDUCTANCE_INIT 450e-6f
#define MODEL_INDUCTANCE_MIN 80e-6f
#define MODEL_INDUCTANCE_MAX 1500e-6f

// size of precomputed pulse buffer
#define THREEPHASE_PULSE_BUFFER_SIZE 256
#define ONEPHASE_PULSE_BUFFER_SIZE 256

// enable for nicer looking waveforms on the scope. Not safe for humans!
// #define THREEPHASE_PULSE_DEFEAT_RANDOMIZATION

#endif // FOCSTIM_CONFIG_H