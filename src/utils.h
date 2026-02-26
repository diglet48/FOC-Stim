#ifndef FOCSTIM_UTILS_H
#define FOCSTIM_UTILS_H

#include <cmath>

float float_rand(float min, float max);

// 2-norm
float norm(float x, float y);

// linear interpolate
float lerp(float p, float a, float b);
float inverse_lerp(float v, float a, float b);
float interpolate(float x, float x0, float x1, float y0, float y1);

float ntc_voltage_to_temp(float ADCVoltage);


#endif // FOCSTIM_UTILS_H