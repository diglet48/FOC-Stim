#ifndef FOCSTIM_BSP_H
#define FOCSTIM_BSP_H

#include "bsp/config_g431b_esc1.h"
#include "bsp/config_g474re_max22213_shield.h"
#include "vec.h"
#include <functional>


void BSP_Init();

/*
PWM notes on g431b-ESC1 and MAX22213 shield:
pwm center aligned.
when CNT >=  CCR, fet connected to grond.
When CNT <   CCR, fet connected to +v
Therefore, writing CCR = 0 forces all outputs to ground.

At pwm peak the following occurs:
 - The current sense DMA is triggered
 - The output compare registers are refreshed with the new values written in the TIM1->OCRx registers (preload)
 - The timer interrupt is called
*/
void BSP_AttachPWMInterrupt(std::function<void()>);

void BSP_DisableOutputs();

// three outputs interface
void BSP_OutputEnable(bool a, bool b, bool c);      // enable or high-z
void BSP_SetPWM3(float a, float b, float c);        // duty cycle 0-1, internally clamped.
void BSP_SetPWM3Atomic(float a, float b, float c);
Vec3f BSP_ReadPhaseCurrents3();


// four outputs interface
#ifdef BSP_ENABLE_FOURPHASE
void BSP_OutputEnable(bool a, bool b, bool c, bool d);
void BSP_SetPWM4(float a, float b, float c, float d);
void BSP_SetPWM4Atomic(float a, float b, float c, float d);
Vec4f BSP_ReadPhaseCurrents4();
#endif



void BSP_AdjustCurrentSenseOffsets();

// various sensors
float BSP_ReadTemperatureInternal();    // stm32 internal temperature sensor.
float BSP_ReadTemperatureOnboardNTC();  // onboard temperature sensor (ESC1)
float BSP_ReadChipAnalogVoltage();      // stm32 vdda
float BSP_ReadPotentiometer();
float BSP_ReadVBus();

void BSP_WriteStatusLED(bool on);



#if defined(ARDUINO_NUCLEO_G474RE)
void BSP_SetSleep(bool sleep);  // sleep pin to MAX22213. low = device sleeps.
bool BSP_ReadFault();           // Fault pin to MAX22213. Active low.
void BSP_SetBoostEnable(bool enable);
void BSP_SetBoostVoltage(float boost_voltage);
void BSP_SetBoostMinimumInputVoltage(float voltage);
float BSP_ReadVSYS();
#endif



#endif // FOCSTIM_BSP_H