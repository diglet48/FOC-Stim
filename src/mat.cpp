#include "mat.h"

#include <algorithm>

static float LOBPCG(const Mat3f &mat, Vec3f guess, float ftol) {
    Vec3f x = guess;
    float previous_quotient = 0;
    float quotient = 0;

    for (int i = 0; i < 100; i++) {
        quotient = dot(x, mat * x) / dot(x, x);
        Vec3f residual = mat * x - x * quotient;
        x = x + residual;
        x = x * (1.f / x.norm());
        float diff = std::abs(previous_quotient - quotient);
        if (diff < ftol) {
            break;
        }
        previous_quotient = quotient;

    }
    return std::abs(quotient);
}


float Mat3f::largest_eigenvalue(float ftol)
{
    float a = LOBPCG(*this, Vec3f(1, 0, 0), ftol);
    float b = LOBPCG(*this, Vec3f(0, 1, 0), ftol);
    float c = LOBPCG(*this, Vec3f(0, 0, 1), ftol);
    return std::max({a, b, c});
}