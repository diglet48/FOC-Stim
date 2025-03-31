#ifndef FOCSTIM_THREEPHASE_MATH
#define FOCSTIM_THREEPHASE_MATH

#include "complex.h"

struct ComplexThreephasePoints {
    Complex p1;
    Complex p2;
    Complex p3;
};

ComplexThreephasePoints project_threephase(
    float pulse_amplitude,
    float alpha, 
    float beta,
    float center_calibration,
    float up_down_calibration,
    float left_right_calibration,
    bool flip_polarity,
    float start_angle
);

#endif