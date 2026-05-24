#ifndef FOCSTIM_COMPLEX_H
#define FOCSTIM_COMPLEX_H

#include <math.h>
#include <complex>

typedef std::complex<float> Complex;

static float dot(Complex l, Complex r) {
    return l.real() * r.real() + l.imag() * r.imag();
}

Complex constrain_in_bound(Complex c, float min_magnitude, float max_magnitude, float min_angle, float max_angle);

/**
 * Replace the point m in the complex plane with two complex points p, q. Such that:
 * p+q = m
 * norm(p) = a
 * norm(q) = b
 * Requires abs(a-b) <= abs(m) <= a+b
 * :param m: complex point
 * :param a: float, desired distance of point from origin
 * :param b: float, desired distance of point from origin
 */
void split_point(Complex m, float a, float b, Complex *out_1, Complex *out_2);



#endif