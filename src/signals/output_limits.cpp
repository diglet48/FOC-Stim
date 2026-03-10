#include "output_limits.h"

#include <math.h>
#include <numbers>

#include "vec.h"
#include "bsp/bsp.h"



float OutputLimits::find_v_seconds(
    Complex i1, Complex i2, Complex i3, Complex z1, Complex z2, Complex z3,
    float carrier_frequency)
{
    // simplified equation, assumes impedance angle of low pass filter
    // is close to impedance angle of transformer. This generally is
    // the case for high-resistance electrodes.

    float v1 = (std::abs(z1) - output_resistance) * std::abs(i1);
    float v2 = (std::abs(z2) - output_resistance) * std::abs(i2);
    float v3 = (std::abs(z3) - output_resistance) * std::abs(i3);
    float max_v = std::max({v1, v2, v3});
    float volt_seconds = max_v / (2 * float(M_PI) * carrier_frequency);
    return volt_seconds;
}

float OutputLimits::find_v_seconds(
    Complex i1, Complex i2, Complex i3, Complex i4, Complex z1, Complex z2, Complex z3, Complex z4,
    float carrier_frequency)
{
    float v1 = (std::abs(z1) - output_resistance) * std::abs(i1);
    float v2 = (std::abs(z2) - output_resistance) * std::abs(i2);
    float v3 = (std::abs(z3) - output_resistance) * std::abs(i3);
    float v4 = (std::abs(z4) - output_resistance) * std::abs(i4);
    float max_v = std::max({v1, v2, v3, v4});
    float volt_seconds = max_v / (2 * float(M_PI) * carrier_frequency);
    return volt_seconds;
}

float OutputLimits::find_v_drive(Complex v1, Complex v2, Complex v3)
{
    // TODO: exact method exists ??
    int steps = 500;
    Complex proj(1, 0);
    Complex rotator(cosf(float(2 * M_PI) / float(steps)), sinf(float(2 * M_PI) / float(steps)));
    float v_drive = 0;
    for (int i = 0; i < steps; i++) {
        float a = (v1 * proj).real();
        float b = (v2 * proj).real();
        float c = (v3 * proj).real();
        float range = std::max({a, b, c}) - std::min({a, b, c});
        v_drive = std::max(range, v_drive);
        proj = proj * rotator;
    }

    return v_drive;
}

float OutputLimits::find_v_drive(Complex v1, Complex v2, Complex v3, Complex v4)
{
    // TODO: exact method exists ??
    int steps = 500;
    Complex proj(1, 0);
    Complex rotator(cosf(float(2 * M_PI) / float(steps)), sinf(float(2 * M_PI) / float(steps)));
    float v_drive = 0;
    for (int i = 0; i < steps; i++) {
        float a = (v1 * proj).real();
        float b = (v2 * proj).real();
        float c = (v3 * proj).real();
        float d = (v4 * proj).real();
        float range = std::max({a, b, c, d}) - std::min({a, b, c, d});
        v_drive = std::max(range, v_drive);
        proj = proj * rotator;
    }

    return v_drive;
}
