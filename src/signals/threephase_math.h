#ifndef FOCSTIM_THREEPHASE_MATH_H
#define FOCSTIM_THREEPHASE_MATH_H

#include "complex.h"
#include "vec.h"

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


ComplexThreephasePoints project_threephase_2(
    float pulse_amplitude,
    float alpha,
    float beta,
    float center_calibration,
    float up_down_calibration,
    float left_right_calibration,
    bool flip_polarity,
    float start_angle
);

// Vec3f threephase_ab_to_electrode_intensity(float alpha, float beta);

/**
 * Unit testing interface
 * Convert the calibration parameters to maximum amplitude of the waveform
 * at the electrodes.
 */
Vec3f threephase_calibration_to_max_amplitude(float up_down_calibration, float left_right_calibration);

/**
 * Unit testing interface
 * Transform valid input point to electrode currents,
 * taking into account the calibration values.
 */
Vec3f threephase_interpolate(float alpha, float beta, Vec3f max_amplitude);

/**
 * Unit testing interface
 * Calculate the intensity of the given pulse
 */
float threephase_intensity(Vec3f electrode_power_in_percent, float reduction_in_center);

/**
 * Unit testing interface
 * Convert the electrode amplitudes to points on the complex plane,
 * where each point is at the designated distance from the center.
 */
ComplexThreephasePoints threephase_electrode_amplitude_to_complex_points(Vec3f amplitudes);

/**
 * Unit testing interface
 * Randomly shuffly the projected points.
 */
ComplexThreephasePoints threephase_permute_complex_points(ComplexThreephasePoints points, bool flip_polarity, float start_angle, float amplitude_multiplicator);

#endif