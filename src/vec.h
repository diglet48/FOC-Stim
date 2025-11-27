#ifndef FOCSTIM_VEC_H
#define FOCSTIM_VEC_H

#include <cmath>

struct Vec2f {
    Vec2f() = default;
    Vec2f(float a, float b)
    : a(a), b(b) {}

    float norm() {
        return sqrtf(a*a + b*b);
    }

    float a;
    float b;
};

struct Vec3f {
    Vec3f() = default;
    Vec3f(float a, float b, float c)
    : a(a), b(b), c(c) {}

    float const min() const {
        return std::min(std::min(a, b), c);
    }

    float const max() const {
        return std::max(std::max(a, b), c);
    }

    float norm() {
        return sqrtf(a*a + b*b + c*c);
    }

    Vec3f const operator*(float f) {
        return Vec3f(a * f, b * f, c * f);
    }

    Vec3f const operator/(float f) {
        return Vec3f(a / f, b / f, c / f);
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

    friend Vec3f cross(Vec3f l, Vec3f r) {
        return {
            l.b * r.c - l.c * r.b,
            l.c * r.a - l.a * r.c,
            l.a * r.b - l.b * r.a
        };
    }

    float a;
    float b;
    float c;
};

struct Vec4f {
    Vec4f() = default;
    constexpr Vec4f(float a, float b, float c, float d)
        : a(a), b(b), c(c), d(d) {}

    float const sum() const {
        return a + b + c + d;
    }

    float const min() const {
        return std::min(std::min(a, b), std::min(c, d));
    }

    float const max() const {
        return std::max(std::max(a, b), std::max(c, d));
    }

    float const norm() const {
        return sqrtf(a*a + b*b + c*c + d*d);
    }

    float const norm_sq() const {
        return a*a + b*b + c*c + d*d;
    }

    Vec4f const operator+(float f) const {
        return Vec4f(a + f, b + f, c + f, d + f);
    }

    Vec4f const operator-(float f) const {
        return Vec4f(a - f, b - f, c - f, d - f);
    }

    Vec4f const operator*(float f) const {
        return Vec4f(a * f, b * f, c * f, d * f);
    }

    Vec4f const operator/(float f) const {
        return Vec4f(a / f, b / f, c / f, d / f);
    }

    Vec4f const operator-(const Vec4f &other) const {
        return Vec4f(a - other.a, b - other.b, c - other.c, d - other.d);
    }

    Vec4f const operator+(const Vec4f &other) const {
        return Vec4f(a + other.a, b + other.b, c + other.c, d + other.d);
    }

    Vec4f const operator*(const Vec4f &other) const {
        return Vec4f(a * other.a, b * other.b, c * other.c, d * other.d);
    }

    friend float dot(Vec4f l, Vec4f r) {
        return l.a * r.a + l.b * r.b + l.c * r.c + l.d * r.d;
    }

    Vec3f const abc() const {
        return {a, b, c};
    }

    Vec3f const abd() const {
        return {a, b, d};
    }

    Vec3f const acd() const {
        return {a, c, d};
    }

    Vec3f const bcd() const {
        return {b, c, d};
    }

    friend bool operator==(const Vec4f &l, const Vec4f &r) {
        return (l.a == r.a) && (l.b == r.b) && (l.c == r.c) && (l.d == r.d);
    }


    float a;
    float b;
    float c;
    float d;
};

#endif // FOCSTIM_VEC_H
