#ifndef FOCSTIM_BSP_H
#define FOCSTIM_BSP_H

#include "bsp/config_g431b_esc1.h"
#include "bsp/config_g474re_max22213_shield.h"
#include "bsp/config_g474re_focstim_v3.h"
#include "bsp/config_g473re_focstim_v4.h"
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
void BSP_AdjustCurrentSenseOffsets(); // ESC1: to be called regularly. V4: to be called regularly when the drivers are enabled but not driving current.

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

#ifdef ARDUINO_B_G431B_ESC1
float BSP_ReadTemperatureOnboardNTC();  // onboard temperature sensor (ESC1)
#endif

// various sensors
float BSP_ReadTemperatureSTM();         // stm32 internal temperature sensor.
float BSP_ReadChipAnalogVoltage();      // stm32 vdda
float BSP_ReadPotentiometerPercentage();
float BSP_ReadVBus();

// LED
#if defined(ARDUINO_B_G431B_ESC1)
void BSP_WriteStatusLED(bool on);
#elif defined(BOARD_FOCSTIM_V3) || defined(BOARD_FOCSTIM_V4)

enum LedPattern {
    Idle,
    Error,
    PlayingVeryLow,
    PlayingLow,
    PlayingMedium,
    PlayingHigh,
};
void BSP_WriteLedPattern(LedPattern pattern);

void BSP_WriteGreenLedBrightness(float a);
void BSP_WriteRedLedBrightness(float a);
#endif


#if defined(ARDUINO_NUCLEO_G474RE) || defined(BOARD_FOCSTIM_V3) || defined(BOARD_FOCSTIM_V4)
bool BSP_ReadFault();           // Fault pin to MAX22213. Active low.
void BSP_SetBoostEnable(bool enable);
void BSP_SetBoostVoltage(float boost_voltage);
void BSP_SetBoostMinimumInputVoltage(float voltage);
float BSP_ReadVSYS();
Vec2f BSP_ReadVSYSRange();
float BSP_BoostDutyCycle(); // debug, remove
bool BSP_ReadPGood();       // low = usb5v present. high = not present.
#endif


void BSP_PrintDebugMsg(const char* fmt, ...);


#endif // FOCSTIM_BSP_H