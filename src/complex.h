#ifndef FOCSTIM_COMPLEX_H
#define FOCSTIM_COMPLEX_H

#include <math.h>
#include <complex>

typedef std::complex<float> Complex;

static float dot(Complex l, Complex r) {
    return l.real() * r.real() + l.imag() * r.imag();
}

Complex constrain_in_bound(Complex c, float min_magnitude, float max_magnitude, float min_angle, float max_angle);



#endif