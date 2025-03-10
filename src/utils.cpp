#include <cmath>

#include <Arduino.h>
#include "utils.h"

float float_rand(float min, float max)
{
    float scale = rand() / (float)RAND_MAX; /* [0, 1.0] */
    return min + scale * (max - min);       /* [min, max] */
};

float norm(float x, float y)
{
    return sqrtf(x * x + y * y);
}

float lerp(float p, float a, float b)
{
    return a + min(1.0f, max(0.0f, p)) * (b - a);
}

float inverse_lerp(float v, float a, float b)
{
    float p = (a - v) / (a - b);
    return constrain(p, 0.f, 1.f);
}

float ntc_voltage_to_temp(float ADCVoltage)
{
    // Formula: https://www.giangrandi.org/electronics/ntc/ntc.shtml
    const float ResistorBalance = 4700.0;
    const float Beta = 3425.0f;
    const float RoomTempI = 1.0F / 298.15f; //[K]
    const float Rt = ResistorBalance * ((3.3F / ADCVoltage) - 1);
    const float R25 = 10000.0F;

    float T = 1.0f / ((logf(Rt / R25) / Beta) + RoomTempI);
    T = T - 273.15f;

    return T;
}
