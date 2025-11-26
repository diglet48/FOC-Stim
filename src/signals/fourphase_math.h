#ifndef FOCSTIM_FOURPHASE_MATH
#define FOCSTIM_FOURPHASE_MATH

#include "complex.h"
#include "vec.h"

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

ComplexFourphasePoints fourphase_electrode_amplitude_to_complex_points(Vec4f amplitude);

ComplexFourphasePoints fourphase_permute_complex_points(ComplexFourphasePoints points, bool flip_polarity, float random_start_angle, float amplitude_multiplicator);

#endif