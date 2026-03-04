#include "boost_control.h"

#ifdef BOARD_FOCSTIM_V4

#include <algorithm>
#include "bsp/bsp.h"

#include "protobuf_api.h"

#define VOLTAGE_SET_MARGIN 1.0f
#define VOLTAGE_ALLOW_MARGIN 0.5f
#define SLOW_START_MARGIN 1.0f
#define SLOW_START_STEP_SIZE 0.5f
#define DECAY 0.001f

BoostControl::BoostControl()
{
    vdrive_slow_start = 5;
    vdrop = 0;
    vdrive = 0;
    boost_setpoint = BOOST_MINIMUM_VOLTAGE;
}

void BoostControl::play_started()
{
    vdrive_slow_start = 5;
    vdrop = 0;
    vdrive = 0;
    boost_setpoint = BOOST_MINIMUM_VOLTAGE;
    BSP_SetBoostVoltage(boost_setpoint);
    BSP_SetBoostEnable(true);
}

void BoostControl::play_stopped()
{
    BSP_SetBoostEnable(false);
    BSP_SetBoostVoltage(0);
}

void BoostControl::update(float requested_vdrive, float boost_min)
{
    // update V_drop
    // V_drop is the maximum observed voltage drop during pulse playback.
    float drop_during_pulse = std::max<float>(0, boost_setpoint - boost_min);
    vdrop = std::max(vdrop, drop_during_pulse);

    // update V_drive
    // V_drive is the maximum requested drive voltage.
    vdrive = std::max(vdrive, requested_vdrive);

    // update V_drive slow start
    vdrive_slow_start = std::clamp(requested_vdrive + SLOW_START_MARGIN, vdrive_slow_start, vdrive_slow_start + SLOW_START_STEP_SIZE);

    // slowly forget old values
    vdrop = std::max<float>(0, vdrop - DECAY);
    vdrive = std::max<float>(0, vdrive - DECAY);
    vdrive_slow_start = std::max<float>(5, vdrive_slow_start - DECAY);


    boost_setpoint = (vdrive + VOLTAGE_SET_MARGIN) / STIM_PWM_MAX_DUTY_CYCLE + vdrop;
    if (BSP_ReadEncoderButton()) {
        boost_setpoint = std::max(boost_setpoint, 20.f);
    }
    boost_setpoint = std::clamp<float>(boost_setpoint, BOOST_MINIMUM_VOLTAGE, BOOST_MAXIMUM_VOLTAGE);
    BSP_SetBoostVoltage(boost_setpoint);
}

bool BoostControl::boost_is_ready() const
{
    return BSP_ReadVBus() >= (boost_setpoint - 0.5f);
}

float BoostControl::max_allowed_vdrive() const
{
    float a = (boost_setpoint - vdrop) * STIM_PWM_MAX_DUTY_CYCLE - VOLTAGE_ALLOW_MARGIN;
    float b = vdrive_slow_start;
    return std::min(a, b);
}

float BoostControl::utilizaton_percent(float voltage) const
{
    float max_allow = (BOOST_MAXIMUM_VOLTAGE - vdrop) * STIM_PWM_MAX_DUTY_CYCLE - VOLTAGE_ALLOW_MARGIN;
    return voltage / max_allow;
}

#endif