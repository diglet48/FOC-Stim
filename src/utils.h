#ifndef FOCSTIM_UTILS_H
#define FOCSTIM_UTILS_H

#include <cmath>

#include <SimpleFOC.h>

float float_rand(float min, float max);

// 2-norm
float norm(float x, float y);

// linear interpolate
float lerp(float p, float a, float b);
float inverse_lerp(float v, float a, float b);

void estimate_resistance_and_inductance(float voltage_peak, BLDCDriver6PWM *driver, CurrentSense *currentSense);

float read_vbus(CurrentSense *currentSense);

float read_temperature(CurrentSense *currentSense);

float read_potentiometer(CurrentSense *currentSense);

void init_led_internal();

void write_led_internal(bool state);

#endif // FOCSTIM_UTILS_H