#if defined(ARDUINO_B_G431B_ESC1)
#ifndef FOCSTIM_BSP_G431B_ESC1_H
#define FOCSTIM_BSP_G431B_ESC1_H

#include "bsp_options.h"
#include "config.h"


#define STIM_PWM_FREQ 50000 // switching frequency is twice this frequency
#define STIM_PWM_MINIMUM_OFF_TIME 3e-6f // limited by pwm-rejection of current sense. Experimentally determined to be 2.6us.
#define STIM_PWM_MAX_DUTY_CYCLE float(1 - STIM_PWM_MINIMUM_OFF_TIME * STIM_PWM_FREQ)

#define CURRENT_SENSE_SCALE CURRENT_SENSE_SCALE_FULL
#define BSP_ENABLE_THREEPHASE



#endif
#endif
