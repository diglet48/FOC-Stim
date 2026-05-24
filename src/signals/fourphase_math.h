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

ComplexFourphasePoints fourphase_electrode_amplitude_to_complex_points(Vec4f amplitude);

ComplexFourphasePoints fourphase_permute_complex_points(ComplexFourphasePoints points, bool flip_polarity, float random_start_angle, float amplitude_multiplicator);

#endif