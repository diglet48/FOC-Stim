#ifndef FOCSTIM_BSP_H
#define FOCSTIM_BSP_H

#include "vec.h"
#include <functional>

void BSP_Init();

void BSP_EnableOutputs();
void BSP_DisableOutputs();
// void BSP_SetOutputState(bool a, bool b, bool c);     
// void BSP_SetOutputState(bool a, bool b, bool c, bool d);

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

// 3-pwm control functions
void BSP_SetPWM3(float a, float b, float c);    // duty cycle
void BSP_SetPWM3Atomic(float a, float b, float c);
Vec3f BSP_ReadPhaseCurrents3();

// 4-pwm control functions
// void BSP_OutputEnable4(bool a, bool b, bool c, bool d);
// void BSP_SetPWM4(float a, float b, float c, float d);
// Vec4f BSP_ReadPhaseCurrents4();

// various sensors
// float BSP_ReadTemperatureInternal();
// float BSP_ReadVMSenseVoltage();
// float BSP_ReadChipAnalogVoltage();
float BSP_ReadPotentiometer();
float BSP_ReadTemperatureOnboardNTC();
float BSP_ReadVBus();

void BSP_AdjustCurrentSenseOffsets();


void BSP_WriteStatusLED(bool on);







#endif // FOCSTIM_BSP_H