#include "threephase_math.h"

#include "utils.h"
#include "foc_utils.h"

// See ThreePhaseHardwareCalibration in restim
// calibration coefs are in alpha-beta space
static void get_calibration_coefs(
    float left_right, float up_down,
    float *s11, float *s12,
    float *s21, float *s22)
{
    // generate_transform_in_ab()
    float theta = atan2f(left_right, up_down) / 2;
    float r = norm(up_down, left_right);
    float a = sinf(theta);
    float b = cosf(theta);
    float scale = 1.f / powf(10, (r / 10));

    if (r == 0)
    {
        // identity matrix
        *s11 = 1;
        *s12 = 0;
        *s21 = 0;
        *s22 = 1;
    }
    else
    {
        float s = (scale - 1);

        // scaling_contant()
        float norm1 = norm(1 + s * a * a, s * a * b);
        float norm2 = norm(1 + s * b * b, s * a * b);
        float scaling_constant = 1.f / std::max(norm1, norm2);

        // scale_in_arbitrary_direction()
        *s11 = (1 + s * a * a) * scaling_constant;
        *s12 = (s * a * b) * scaling_constant;
        *s21 = (s * a * b) * scaling_constant;
        *s22 = (1 + s * b * b) * scaling_constant;
    }
}

ComplexThreephasePoints project_threephase(
    float pulse_amplitude, 
    float alpha, 
    float beta, 
    float center_calibration, 
    float up_down_calibration, 
    float left_right_calibration,
    bool flip_polarity,
    float start_angle)
{
    // constrain (alpha, beta) to unit circle
    float r = sqrtf(alpha * alpha + beta * beta);
    if (r > 1)
    {
        alpha /= r;
        beta /= r;
        r = 1;
    }

    // https://github.com/diglet48/restim/wiki/software-basics
    // base projection matrix in ab space
    float a11 = 0.5f * (2 - r + alpha);
    float a12 = 0.5f * beta;
    float a21 = 0.5f * beta;
    float a22 = 0.5f * (2 - r - alpha);

    // calibration matrix
    float t11, t12, t21, t22;
    get_calibration_coefs(left_right_calibration, up_down_calibration, &t11, &t12, &t21, &t22);

    // calibration * projection
    float b11 = t11 * a11 + t12 * a21;
    float b12 = t11 * a12 + t12 * a22;
    float b21 = t21 * a11 + t22 * a21;
    float b22 = t21 * a12 + t22 * a22;

    // ab transform
    float ab11 = 1;
    float ab12 = 0;
    float ab21 = -0.5f;
    float ab22 = -_SQRT3_2; // left
    float ab31 = -0.5f;
    float ab32 = _SQRT3_2;  // right
    
    // complex points
    Complex p1(ab11 * b11 + ab12 * b21, ab11 * b12 + ab12 * b22);
    Complex p2(ab21 * b11 + ab22 * b21, ab21 * b12 + ab22 * b22);
    Complex p3(ab31 * b11 + ab32 * b21, ab31 * b12 + ab32 * b22);

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

    if (flip_polarity) {
        p1 = Complex(-p1.a, p1.b);
        p2 = Complex(-p2.a, p2.b);
        p3 = Complex(-p3.a, p3.b);
    }

    Complex rotation(cosf(start_angle), sinf(start_angle));

    return ComplexThreephasePoints{
        .p1 = p1 * rotation * pulse_amplitude,
        .p2 = p2 * rotation * pulse_amplitude,
        .p3 = p3 * rotation * pulse_amplitude
    };
}