#if defined(ARDUINO_NUCLEO_G474RE)
#ifndef FOCSTIM_G474_MAX22213_SHIELD_H
#define FOCSTIM_G474_MAX22213_SHIELD_H

#include "bsp_options.h"
#include "config.h"
#include "vec.h"

#include <cstdint>


#define STIM_PWM_FREQ 90000 // switching frequency is twice this frequency
#define STIM_PWM_MINIMUM_OFF_TIME 3e-6f // TODO: test
#define STIM_PWM_MAX_DUTY_CYCLE float(1 - STIM_PWM_MINIMUM_OFF_TIME * STIM_PWM_FREQ)

#define CURRENT_SENSE_SCALE CURRENT_SENSE_SCALE_HALF
#define BSP_ENABLE_THREEPHASE
#define BSP_ENABLE_FOURPHASE

#define MAX22213_HFS 1
#define MAX22213_RISEN 2000
#if MAX22213_HFS == 0
#define MAX22213_KISEN 7500     // HFS 0, max current 3A
#else
#define MAX22213_KISEN 3840     // HFS 0, max current 1.5A, higher resoution
#endif






// board config
void BSP_SetSleep(bool sleep);  // sleep pin to MAX22213. low = device sleeps.
bool BSP_ReadFault();           // Fault pin to MAX22213. Active low.

// // various sensors
// float BSP_ReadTemperatureInternal();
// float BSP_ReadVMSenseVoltage();
// float BSP_ReadChipAnalogVoltage();




#endif
#endif
