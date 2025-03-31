#ifndef FOCSTIM_VEC_H
#define FOCSTIM_VEC_H

#include <cmath>

struct Vec2f {
    Vec2f() = default;
    Vec2f(float a, float b)
    : a(a), b(b) {}

    float a;
    float b;
};

struct Vec3f {
    Vec3f() = default;
    Vec3f(float a, float b, float c)
    : a(a), b(b), c(c) {}

    float norm() {
        return sqrtf(a*a + b*b + c*c);
    }

    Vec3f const operator*(float f) {
        return Vec3f(a * f, b * f, c * f);
    }

    Vec3f const operator-(const Vec3f &other) {
        return Vec3f(a - other.a, b - other.b, c - other.c);
    }

    Vec3f const operator+(const Vec3f &other) {
        return Vec3f(a + other.a, b + other.b, c + other.c);
    }    

    friend float dot(Vec3f l, Vec3f r) {
        return l.a * r.a + l.b * r.b + l.c * r.c;
    }    

    float a;
    float b;
    float c;
};

struct Vec4f {
    Vec4f() = default;
    Vec4f(float a, float b, float c, float d)
        : a(a), b(b), c(c), d(d) {}

    float a;
    float b;
    float c;
    float d;
};

#endif // FOCSTIM_VEC_H
