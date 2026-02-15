#ifndef FOCSTIM_FOURPHASE_MATH_2_H
#define FOCSTIM_FOURPHASE_MATH_2_H

#include "fourphase_math.h"
#include "vec.h"

ComplexFourphasePoints project_fourphase_2(
    float pulse_amplitude,
    Vec4f position_vector,
    Vec4f calibration_vector,
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
float fourphase_intensity(Vec4f electrode_power_in_percent);

#endif