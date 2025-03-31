#include "fourphase_math.h"

#include "vec.h"
#include "mat.h"
#include "utils.h"
#include <cmath>

constexpr float COEF_1 = 1;
constexpr float COEF_2 = sqrtf(8) / 3;         // sqrt(1 - coef_1**2/3)
constexpr float COEF_3 = sqrtf(2) / sqrtf(3);  // sqrt(1 - coef_1**2/3 - coef_2**2/2)

static const Vec3f basis1(COEF_1, 0, 0);
static const Vec3f basis2(-COEF_1 / 3, COEF_2, 0);
static const Vec3f basis3(-COEF_1 / 3, -COEF_2 / 2, COEF_3);
static const Vec3f basis4(-COEF_1 / 3, -COEF_2 / 2, -COEF_3);


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
    if (m.norm() < .001f) {
        m_normalized = Complex(1, 0);
    } else {
        m_normalized = m * (1 / m.norm());
    }
   
    float c = m.norm();
    // handle special case c == 0. Note that if c==0 then a==b, therefore solution is trivial.
    c = std::max(0.0001f, c);
    float rational = (a*a - b*b + c*c) / (2 * c);
    float imaginary = sqrtf(std::max(a*a - rational*rational, 0.f));
    
    Complex p = Complex(rational, imaginary) * m_normalized;
    Complex q = m - p;

    *out_1 = p;
    *out_2 = q;
}

Mat3f scale_in_arb_dir(Vec3f vec, float s) {
    // vec * vec.T * (s - 1) + eye(3)
    // https://math.stackexchange.com/questions/3945174/how-do-you-scale-along-a-non-canonical-direction
    float q = s - 1;
    return Mat3f(
        vec.a * vec.a * q + 1,
        vec.a * vec.b * q,
        vec.a * vec.c * q,

        vec.b * vec.a * q,
        vec.b * vec.b * q + 1,
        vec.b * vec.c * q,
        
        vec.c * vec.a * q,
        vec.c * vec.b * q,
        vec.c * vec.c * q + 1
    );
}

Mat3f calibration_matrix(float calib_a, float calib_b, float calib_c, float calib_d) {
    float pa = powf(10, calib_a / 10);
    float pb = powf(10, calib_b / 10);
    float pc = powf(10, calib_c / 10);
    float pd = powf(10, calib_d / 10);

    // magic multiplication sequence to get a symmetric matrix.
    Mat3f ma = scale_in_arb_dir(basis1, pa);
    Mat3f mb = scale_in_arb_dir(basis2, sqrtf(pb));
    Mat3f mc = scale_in_arb_dir(basis3, sqrtf(pc));
    Mat3f md = scale_in_arb_dir(basis4, sqrtf(pd));
    Mat3f calibration_matrix = md * mc * mb * ma * mb * mc * md;

    // scale the calibration matrix by the largest eigenvalue
    // to ensure current amplitude never increases by calibration
    float eigenvalue = calibration_matrix.largest_eigenvalue(.0001f);
    return calibration_matrix * (1 / eigenvalue);
}



ComplexFourphasePoints project_fourphase(
    float pulse_amplitude,
    float alpha, float beta, float gamma,
    float center_calibration,
    float a_calibration, float b_calibration, float c_calibration, float d_calibration,
    bool flip_polarity,
    float start_angle)
{
    
    // clamp input position norm to <= 1
    Vec3f position(alpha, beta, gamma);
    float r = position.norm();  
    if (r > 1) {   
        position = position * (1 / r);
        r = 1;
    }
    
    // compute calibration matrix
    Mat3f calib = calibration_matrix(a_calibration, b_calibration, c_calibration, d_calibration);
    Vec3f vec1 = calib * basis1;
    Vec3f vec2 = calib * basis2;
    Vec3f vec3 = calib * basis3;
    Vec3f vec4 = calib * basis4;
    
    // compute electrode amplitude
    float a1 = (1 - r) * vec1.norm() + abs(dot(vec1, position));
    float a2 = (1 - r) * vec2.norm() + abs(dot(vec2, position));
    float a3 = (1 - r) * vec3.norm() + abs(dot(vec3, position));
    float a4 = (1 - r) * vec4.norm() + abs(dot(vec4, position));

    // construct point p, such that 
    // abs(a1-a2) <= abs(p) <= a1+a2
    // abs(a3-a4) <= abs(p) <= a3+a4
    float p_min = abs(a3 - a4);
    float p_max = a3 + a4;
    p_max = std::min(p_max, a1 + a2);
    p_min = std::max(p_min, abs(a1 - a2));
    Complex p((p_min + p_max) * 0.5f, 0);

    // use p to project points on complex plane
    Complex p1, p2, p3, p4;
    split_point(p, a1, a2, &p1, &p2);
    p = Complex(-p.a, 0);
    split_point(p, a3, a4, &p3, &p4);

    // center calibration
    float ratio = powf(10, (center_calibration / 10));
    if (ratio <= 1)
    {
        pulse_amplitude *= lerp(r, ratio, 1);
    }
    else
    {
        pulse_amplitude *= lerp(r, 1, 1 / ratio);
    }

    // flip polarity
    if (flip_polarity) {
        p1 = Complex(-p1.a, p1.b);
        p2 = Complex(-p2.a, p2.b);
        p3 = Complex(-p3.a, p3.b);
        p4 = Complex(-p4.a, p4.b);
    }

    // random start angle
    Complex rotation(cosf(start_angle), sinf(start_angle));

    return ComplexFourphasePoints{
        .p1 = p1 * rotation * pulse_amplitude,
        .p2 = p2 * rotation * pulse_amplitude,
        .p3 = p3 * rotation * pulse_amplitude,
        .p4 = p4 * rotation * pulse_amplitude
    };
}