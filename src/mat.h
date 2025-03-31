#ifndef FOCSTIM_MAT_H
#define FOCSTIM_MAT_H

#include <cmath>
#include <vec.h>

struct Mat3f {
    Mat3f(
        float a11, float a12, float a13,
        float a21, float a22, float a23,
        float a31, float a32, float a33
    )
    :   a11(a11),   a12(a12),   a13(a13)
    ,   a21(a21),   a22(a22),   a23(a23)
    ,   a31(a31),   a32(a32),   a33(a33)
    {}

    // requires symmetric matrix
    float largest_eigenvalue(float ftol);

    friend Mat3f operator*(const Mat3f &l, const Mat3f &r) {
        return Mat3f(
            l.a11 * r.a11 + l.a12 * r.a21 + l.a13 * r.a31,
            l.a11 * r.a12 + l.a12 * r.a22 + l.a13 * r.a32,
            l.a11 * r.a13 + l.a12 * r.a23 + l.a13 * r.a33,
    
            l.a21 * r.a11 + l.a22 * r.a21 + l.a23 * r.a31,
            l.a21 * r.a12 + l.a22 * r.a22 + l.a23 * r.a32,
            l.a21 * r.a13 + l.a22 * r.a23 + l.a23 * r.a33,
    
            l.a31 * r.a11 + l.a32 * r.a21 + l.a33 * r.a31,
            l.a31 * r.a12 + l.a32 * r.a22 + l.a33 * r.a32,
            l.a31 * r.a13 + l.a32 * r.a23 + l.a33 * r.a33
        );
    }

    friend Vec3f operator*(const Mat3f &l, const Vec3f &r) {
        return Vec3f(
            l.a11 * r.a + l.a12 * r.b + l.a13 * r.c, 
            l.a21 * r.a + l.a22 * r.b + l.a23 * r.c, 
            l.a31 * r.a + l.a32 * r.b + l.a33 * r.c
        );
    }

    Mat3f operator*(const float f) {
        return Mat3f(
            a11 * f, a21 * f, a31 * f,
            a12 * f, a22 * f, a32 * f,
            a13 * f, a23 * f, a33 * f
        );
    }

    Mat3f transpose() {
        return Mat3f(
            a11, a21, a31,
            a12, a22, a32,
            a13, a23, a33
        );
    }

    float a11, a12, a13;
    float a21, a22, a23;
    float a31, a32, a33;
};



#endif // FOCSTIM_VEC_H
