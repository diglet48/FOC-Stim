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

void split_point(Complex m, float a, float b, Complex *out_1, Complex *out_2) {
    Complex m_normalized;
    if (std::abs(m) < .001f) {
        m_normalized = Complex(1, 0);
    } else {
        m_normalized = m * (1 / std::abs(m));
    }

    float c = std::abs(m);
    // handle special case c == 0. Note that if c==0 then a==b, therefore solution is trivial.
    c = std::max(0.0001f, c);
    float rational = (a*a - b*b + c*c) / (2 * c);
    float imaginary = sqrtf(std::max(a*a - rational*rational, 0.f));

    Complex p = Complex(rational, imaginary) * m_normalized;
    Complex q = m - p;

    *out_1 = p;
    *out_2 = q;
}
