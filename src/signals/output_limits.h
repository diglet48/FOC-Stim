#ifndef FOC_OUTPUT_LIMITS_H
#define FOC_OUTPUT_LIMITS_H

#include "complex.h"

class OutputLimits {
public:
    OutputLimits(float max_allowed_v_drive, float max_allowed_v_sec, float output_resistance)
        : max_allowed_v_drive(max_allowed_v_drive)
        , max_allowed_v_sec(max_allowed_v_sec)
        , output_resistance(output_resistance)
    { }

    // find the maximum volt*seconds over the transformer
    float find_v_seconds(
        Complex i1, Complex i2, Complex i3,
        Complex z1, Complex z2, Complex z3,
        float carrier_frequency);

    float find_v_seconds(
        Complex i1, Complex i2, Complex i3, Complex i4,
        Complex z1, Complex z2, Complex z3, Complex z4,
        float carrier_frequency);

    float find_v_drive(Complex v1, Complex v2, Complex v3);
    float find_v_drive(Complex v1, Complex v2, Complex v3, Complex v4);

    float max_allowed_v_drive;
    float max_allowed_v_sec;

private:
    // real component of the output impedance of the circuit
    float output_resistance;
};


#endif