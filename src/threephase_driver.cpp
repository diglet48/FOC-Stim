#include "threephase_driver.h"

#include <SimpleFOC.h>
#include <config.h>

#include "drivers/hardware_specific/stm32/stm32_mcu.h"

void ThreePhaseDriver::init(BLDCDriver *driver, CurrentSense *current_sense)
{
    this->driver = driver;
    this->current_sense = current_sense;
}

void ThreePhaseDriver::pwm_to_ground()
{
    TIM_TypeDef *tim = TIM1;
    // Temporarily disable the update event to ensure atomic update
    // of all 3 pwm timer outputs, preventing transients.
    SET_BIT(tim->CR1, TIM_CR1_UDIS_Msk);
    set_pwm_voltage(0, 0, 0);   // connect all to ground.
    CLEAR_BIT(tim->CR1, TIM_CR1_UDIS_Msk);
}

void ThreePhaseDriver::pwm_to_center()
{
    TIM_TypeDef *tim = TIM1;
    // Temporarily disable the update event to ensure atomic update
    // of all 3 pwm timer outputs, preventing transients.
    SET_BIT(tim->CR1, TIM_CR1_UDIS_Msk);
    float center = driver->voltage_power_supply / 2;
    set_pwm_voltage(center, center, center);
    CLEAR_BIT(tim->CR1, TIM_CR1_UDIS_Msk);
}

void ThreePhaseDriver::set_pwm_voltage(float voltage_neutral, float voltage_left, float voltage_right)
{
    // Ugly but much faster than simplefoc setpwm
    STM32DriverParams* params = (STM32DriverParams*)driver->params;
    float pwm_a = (voltage_right) / (STIM_PSU_VOLTAGE / _PWM_RANGE);
    float pwm_b = (voltage_neutral) / (STIM_PSU_VOLTAGE / _PWM_RANGE);
    float pwm_c = (voltage_left) / (STIM_PSU_VOLTAGE / _PWM_RANGE);
    params->timers[0]->setCaptureCompare(params->channels[0], pwm_a, (TimerCompareFormat_t)_PWM_RESOLUTION);
    params->timers[2]->setCaptureCompare(params->channels[2], pwm_b, (TimerCompareFormat_t)_PWM_RESOLUTION);
    params->timers[4]->setCaptureCompare(params->channels[4], pwm_c, (TimerCompareFormat_t)_PWM_RESOLUTION);
}

void ThreePhaseDriver::attach_interrupt(std::function<void()> fn)
{
    STM32DriverParams* params = (STM32DriverParams*)driver->params;
    HardwareTimer *timer = params->timers[0];
    timer->attachInterrupt(fn);
}

void ThreePhaseDriver::detach_interrupt()
{
    STM32DriverParams* params = (STM32DriverParams*)driver->params;
    HardwareTimer *timer = params->timers[0];
    timer->detachInterrupt();
}

void ThreePhaseDriver::adjust_offsets()
{
    float alpha = (1.f/1000) / current_sense->gain_a;
    PhaseCurrent_s currents = current_sense->getPhaseCurrents();
    current_sense->offset_ia += currents.a * alpha;
    current_sense->offset_ib += currents.b * alpha;
    current_sense->offset_ic += currents.c * alpha;
}

ThreePhaseCurrents ThreePhaseDriver::get_currents()
{
    PhaseCurrent_s currents = current_sense->getPhaseCurrents();
    return ThreePhaseCurrents{
        .neutral = currents.b,
        .left = currents.c,
        .right = currents.a
    };
}


