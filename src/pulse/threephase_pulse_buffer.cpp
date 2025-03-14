#include "pulse/threephase_pulse_buffer.h"

#include <Arduino.h>
#include <math.h>
#include <cstdint>

#include "foc_utils.h"
#include "config.h"
#include "utils.h"

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

void ThreephasePulseBuffer::create_pulse(
    float current_amplitude,
    float alpha, float beta, float carrier_frequency,
    float pulse_width, float pulse_rise,
    float center_calibration, float up_down_calibration, float left_right_calibration)
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
    float ab22 = -_SQRT3_2; // selects the left electrode

    // ab * (calibration * projection)
    float c11 = ab11 * b11 + ab12 * b21;
    float c12 = ab11 * b12 + ab12 * b22;
    float c21 = ab21 * b11 + ab22 * b21;
    float c22 = ab21 * b12 + ab22 * b22;

    // center calibration
    float ratio = powf(10, (center_calibration / 10));
    if (ratio <= 1)
    {
        current_amplitude *= lerp(r, ratio, 1);
    }
    else
    {
        current_amplitude *= lerp(r, 1, 1 / ratio);
    }

    // variant with no calibration
    // float c11 = a11;
    // float c12 = a12;
    // float c21 = -0.5f * a11 + _SQRT3_2 * a21;
    // float c22 = -0.5f * a12 + _SQRT3_2 * a22;

    static float random_start_angle = 0;
    random_start_angle = random_start_angle + (_2PI * 19 / 97); // ~1/5
    if (random_start_angle > _2PI)
    {
        random_start_angle -= _2PI;
    }

    static bool polarity = 0;
    polarity = !polarity;

#ifdef THREEPHASE_PULSE_DEFEAT_RANDOMIZATION
    random_start_angle = 0;
    polarity = 0;
#endif

    pulse_duration = 1.f / carrier_frequency * pulse_width; // seconds
    pulse_rise = max(pulse_rise, 2.f);
    pulse_rise = min(pulse_rise, pulse_width / 2);
    float rise_end = pulse_duration * (pulse_rise / pulse_width);
    float fall_start = pulse_duration - rise_end;

    for (int i = 0; i < THREEPHASE_PULSE_BUFFER_SIZE; i++)
    {
        float t = (i * pulse_duration) / float(THREEPHASE_PULSE_BUFFER_SIZE);
        float sin, cos;
        _sincos(t * _2PI * carrier_frequency + random_start_angle, &sin, &cos);
        if (polarity)
        {
            sin = -sin;
            cos = -cos;
        }

        // (ab * calibration * projection) * [cos, sin]^T
        float l = c11 * cos + c12 * sin;
        float r = c21 * cos + c22 * sin;

        float pulse_envelope;
        if (t <= rise_end) {    // rising
            // env = sin(t * carrier_frequency / pulse_rise * pi / 2)
            _sincos(t * (carrier_frequency / pulse_rise) * _PI_2, &sin, &cos);
            pulse_envelope = sin;
        } else if (t <= fall_start) {   // steady
            // env = 1
            pulse_envelope = 1;
        } else {    // falling
            // env = sin((t - fall_start + rise_end) * carrier_frequency / pulse_rise * pi / 2)
            _sincos((t - fall_start + rise_end) * (carrier_frequency / pulse_rise) * _PI_2, &sin, &cos);
            pulse_envelope = sin;
        }

        this->a[i] = current_amplitude * (l * pulse_envelope);
        this->b[i] = current_amplitude * (r * pulse_envelope);
    }
}

void ThreephasePulseBuffer::print()
{
    for (int i = 0; i < THREEPHASE_PULSE_BUFFER_SIZE; i++)
    {
        float c = -a[i] - b[i];
        printf("%f % f % f\r\n", a[i], b[i], c);
    }
}