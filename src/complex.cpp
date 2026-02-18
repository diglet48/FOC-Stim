#include "complex.h"

#include "foc_utils.h"
#include <algorithm>


Complex constrain_in_bound(Complex c, float min_magnitude, float max_magnitude, float min_angle, float max_angle)
{
    float magnitude = std::abs(c);
    float angle = atan2f(c.imag(), c.real());

    float desired_magnitude = std::clamp<float>(magnitude, min_magnitude, max_magnitude);
    float desired_angle = std::clamp<float>(angle, min_angle, max_angle);

    if (magnitude == desired_magnitude && angle == desired_angle) {
        return c;
    }

    float a = cosf(desired_angle) * desired_magnitude;
    float b = sinf(desired_angle) * desired_magnitude;
    return Complex(a, b);
}
