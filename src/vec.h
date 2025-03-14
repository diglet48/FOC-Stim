#ifndef FOCSTIM_VEC_H
#define FOCSTIM_VEC_H

struct Vec2f {
    Vec2f(float a, float b)
        : a(a), b(b) {}

    float a;
    float b;
};

struct Vec3f {
    Vec3f(float a, float b, float c)
        : a(a), b(b), c(c) {}

    void normalize() {
        float midpoint = (a + b + c) / 3;
        a -= midpoint;
        b -= midpoint;
        c -= midpoint;
    }

    float a;
    float b;
    float c;
};

struct Vec4f {
    Vec4f(float a, float b, float c, float d)
        : a(a), b(b), c(c), d(d) {}

    float a;
    float b;
    float c;
    float d;
};

#endif // FOCSTIM_VEC_H
