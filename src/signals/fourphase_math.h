#ifndef FOCSTIM_FOURPHASE_MATH
#define FOCSTIM_FOURPHASE_MATH

#include "complex.h"

struct ComplexFourphasePoints {
    Complex p1;
    Complex p2;
    Complex p3;
    Complex p4;
};

ComplexFourphasePoints project_fourphase(
    float pulse_amplitude,
    float alpha, float beta, float gamma,
    float center_calibration,
    float a_calibration, float b_calibration, float c_calibration, float d_calibration,
    bool flip_polarity,
    float start_angle
);

#endif