#include "encoder.h"
#include "bsp/bsp.h"

#include <Arduino.h>

#define ENCODER_TOTAL_TICKS         200 // 1 rotation = 96 ticks
#define QUICK_TURN_TIME_CONSTANT_MS 500
#define QUICK_TURN_THRESHOLD        14  // about 1/8 turn

Encoder::Encoder()
{

}

void Encoder::update()
{
    uint16_t new_position = BSP_ReadEncoderPosition();
    int delta = static_cast<int16_t>(new_position - last_encoder_position);
    last_encoder_position = new_position;

    if (!locked) {
        position = std::clamp<int>((int)position + delta, 0, ENCODER_TOTAL_TICKS);
    }

    uint32_t time_ms = millis();
    uint32_t dt = time_ms - last_update_time;
    last_update_time = time_ms;
    negative_velocity *= std::max(1 - float(dt) / QUICK_TURN_TIME_CONSTANT_MS, 0.f); // ~ exp(1 - dt/time_constant)
    if (delta < 0) {
        negative_velocity -= delta;
    }
}

float Encoder::percentage() const
{
    return float(position) / ENCODER_TOTAL_TICKS;
}

float Encoder::volume() const
{
    float x = float(position) / ENCODER_TOTAL_TICKS;
    // this function has a derivative of 2 at x=0 and 0.5 at x=1.
    // With 200 ticks, increments are 1% and 0.25% per tick respectively.
    return 0.5f * x * x * x - 1.5f * x * x + 2 * x;
}

void Encoder::resetVolume()
{
    position = 0;
}

float Encoder::unlockProgress() const
{
    return std::clamp<float>(negative_velocity / QUICK_TURN_THRESHOLD, 0, 1);
}

void Encoder::setLocked(bool b)
{
    locked = b;
}

bool Encoder::isLocked()
{
    return locked;
}

bool Encoder::isQuickTurned() const
{
    return negative_velocity >= QUICK_TURN_THRESHOLD;
}
