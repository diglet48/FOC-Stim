#ifndef FOCSTIM_FOURPHASE_MATH_H
#define FOCSTIM_FOURPHASE_MATH_H

#include "complex.h"
#include "vec.h"

struct ComplexFourphasePoints {
    Complex p1;
    Complex p2;
    Complex p3;
    Complex p4;
};

ComplexFourphasePoints project_fourphase_2(
    float pulse_amplitude,
    Vec4f position_vector,
    Vec4f calibration_vector,
    float reduction_in_center,
    bool flip_polarity,
    float start_angle
);


/**
 * Unit testing interface
 * clip fourphase coordinates to the nearest valid point
 */
Vec4f fourphase_constrain_coordinates(Vec4f in);

/**
 * Unit testing interface
 * convert calibration values (db) to maximum electrode amplitude (0-1)
 */
Vec4f fourphase_calibration_to_amplitude(Vec4f calibration_vector_in_db);

/**
 * Unit testing interface
 * Transform valid input point to electrode currents,
 * taking into account the calibration values.
 */
Vec4f fourphase_interpolate(Vec4f p, Vec4f max_amplitude);

/**
 * Unit testing interface
 * Calculate the intensity of the pulse coordinates
 */
float fourphase_intensity(Vec4f electrode_power_in_percent, float reduction_in_center);

/**
 * Unit testing interface
 * Convert the electrode amplitudes to points on the complex plane,
 * where each point is at the designated distance from the center.
 */
ComplexFourphasePoints fourphase_electrode_amplitude_to_complex_points(Vec4f amplitude);

/**
 * Unit testing interface
 * Randomly shuffly the projected points.
 */
ComplexFourphasePoints fourphase_permute_complex_points(ComplexFourphasePoints points, bool flip_polarity, float random_start_angle, float amplitude_multiplicator);


#endif