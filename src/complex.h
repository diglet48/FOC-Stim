#ifndef FOCSTIM_COMPLEX_H
#define FOCSTIM_COMPLEX_H

#include <math.h>

struct Complex {
    Complex() 
        : a(0), b(0) {}

    Complex(float a, float b)
        : a(a), b(b) {}

    
    float norm() {
        return sqrtf(a*a + b*b);
    }

    void constrain_in_bound(float min_magnitude, float max_magnitude, float min_angle, float max_angle);
        
    float a, b;
};
    
    
static Complex operator+(Complex l, Complex r) {
    return Complex(l.a + r.a, l.b + r.b);
}

static Complex operator-(Complex l, Complex r) {
    return Complex(l.a - r.a, l.b - r.b);
}

static Complex operator*(Complex l, Complex r) {
    return Complex(l.a * r.a - l.b * r.b, l.a * r.b + l.b * r.a);
}

static Complex operator*(Complex l, float f) {
    return Complex(l.a * f, l.b * f);
}

static float dot(Complex l, Complex r) {
    return l.a * r.a + l.b * r.b;
}

#endif