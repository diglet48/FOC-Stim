#include "fourphase_math.h"

#include "vec.h"
#include "mat.h"
#include "utils.h"
#include <cmath>
#include <complex>


void split_point(Complex m, float a, float b, Complex *out_1, Complex *out_2) {
    // Replace the point m in the complex plane with two complex points p, q. Such that:
    // p+q = m
    // norm(p) = a
    // norm(q) = b
    // Requires abs(a-b) <= abs(m) <= a+b
    // :param m: complex point
    // :param a: float, desired distance of point from origin
    // :param b: float, desired distance of point from origin

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


ComplexFourphasePoints fourphase_electrode_amplitude_to_complex_points(Vec4f amplitudes)
{
    // construct point p, such that
    // abs(a1-a2) <= abs(p) <= a1+a2
    // abs(a3-a4) <= abs(p) <= a3+a4
    float p_min = abs(amplitudes.c - amplitudes.d);
    float p_max = amplitudes.c + amplitudes.d;
    p_max = std::min(p_max, amplitudes.a + amplitudes.b);
    p_min = std::max(p_min, abs(amplitudes.a - amplitudes.b));
    Complex p((p_min + p_max) * 0.5f, 0);

    // use p to project points on complex plane
    Complex p1, p2, p3, p4;
    split_point(p, amplitudes.a, amplitudes.b, &p1, &p2);
    p = Complex(-p.real(), 0);
    split_point(p, amplitudes.c, amplitudes.d, &p3, &p4);

    return ComplexFourphasePoints{
        p1, p2, p3, p4
    };
}

ComplexFourphasePoints fourphase_permute_complex_points(ComplexFourphasePoints points, bool flip_polarity, float random_start_angle, float amplitude_multiplicator)
{
    // flip polarity
    if (flip_polarity) {
        points.p1 = Complex(-points.p1.real(), points.p1.imag());
        points.p2 = Complex(-points.p2.real(), points.p2.imag());
        points.p3 = Complex(-points.p3.real(), points.p3.imag());
        points.p4 = Complex(-points.p4.real(), points.p4.imag());
    }

    // random start angle
    Complex rotation(cosf(random_start_angle), sinf(random_start_angle));

    points.p1 = points.p1 * rotation * amplitude_multiplicator;
    points.p2 = points.p2 * rotation * amplitude_multiplicator;
    points.p3 = points.p3 * rotation * amplitude_multiplicator;
    points.p4 = points.p4 * rotation * amplitude_multiplicator;
    return points;
}
