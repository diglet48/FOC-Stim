#if defined(BOARD_FOCSTIM_V4)
#ifndef FOCSTIM_CONFIG_G473RE_FOCSTIM_V4_H
#define FOCSTIM_CONFIG_G473RE_FOCSTIM_V4_H

// transformer winding ratio
#define STIM_WINDING_RATIO 6.66f    // xicon 42TL004
#define STIM_WINDING_RATIO_SQ (STIM_WINDING_RATIO * STIM_WINDING_RATIO)
// The true winding ratio actually is 5.5, but due to core losses not all
// current makes it to the output. Over-estimating the winding ratio
// accounts for this in the general case (1000hz / 200ohm)

// current limits
#define BODY_CURRENT_MAX  0.15f  // in amps
#define ESTOP_CURRENT_LIMIT_MARGIN 0.12f                // accounts for measurement noise, driving current

// board temperature limits
#define MAXIMUM_TEMPERATURE 60.f    // degrees celsius.

// boost voltage control
#define STIM_DYNAMIC_VOLTAGE
#define BOOST_MINIMUM_VOLTAGE       10.f            // boost voltage setpoint (min)
#define BOOST_MAXIMUM_VOLTAGE       29.5f           // boost voltage setpoint (max)
#define BOOST_OVERVOLTAGE_THRESHOLD 31.f            // e-stop when exceeded

// pwm
#define STIM_PWM_FREQ 50000 // switching frequency is twice this frequency
#define STIM_PWM_MINIMUM_OFF_TIME 3e-6f // limited by pwm-rejection of current sense. Experimentally determined to be 2.6us.
#define STIM_PWM_MAX_DUTY_CYCLE float(1 - STIM_PWM_MINIMUM_OFF_TIME * STIM_PWM_FREQ)
#define STIM_PWM_MAX_VDRIVE (STIM_PWM_MAX_DUTY_CYCLE * STIM_BOOST_VOLTAGE_LOW_THRESHOLD)

// deadtime compensation
#define DEADTIME_COMPENSATION_ENABLE
#define DEADTIME_COMPENSATION_MAGIC_VALUE  160e-9f   // experimentally determined. Close to driver deadtime (200ns).
#define DEADTIME_COMPENSATION_PERCENTAGE (DEADTIME_COMPENSATION_MAGIC_VALUE * STIM_PWM_FREQ)
#define DEADTIME_COMPENSATION_CURRENT_THRESHOLD .025f // experimentally determined (at 22v)

// initial conditions and limits for the model
#define MODEL_RESISTANCE_INIT (200.f / STIM_WINDING_RATIO_SQ)
#define MODEL_RESISTANCE_MIN (50.f / STIM_WINDING_RATIO_SQ)
#define MODEL_RESISTANCE_MAX (1500.f / STIM_WINDING_RATIO_SQ)
#define MODEL_PHASE_ANGLE_MIN -1.5f
#define MODEL_PHASE_ANGLE_MAX 1.5f

// transformer saturation
#define MODEL_FIXED_RESISTANCE      1.6f        // driver Rdson (0.3) + low pass filter (0.3) + transformer low-side resistance (1.0)
#define MODEL_MAXIMUM_VOLT_SECONDS  1100e-6f    // Xicon 42TL004 and 001

// I2C clocks
#define I2C_CLOCK_NORMAL        400'000ULL
#define I2C_CLOCK_DISPLAY       1'000'000ULL
#define I2C_CLOCK_LSM6DSOX      1'000'000ULL
#define I2C_CLOCK_MICROPRESSURE 400'000ULL
#define I2C_CLOCK_BQ27411       400'000ULL
#define I2C_CLOCK_ESP32         400'000ULL

// enable for nicer looking waveforms on the scope. Not safe for humans!
// #define THREEPHASE_PULSE_DEFEAT_RANDOMIZATION

#define CURRENT_SENSE_SCALE_HALF
#define BSP_ENABLE_FOURPHASE
#define BATTERY_ENABLE

#endif
#endif