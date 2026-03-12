#ifndef FOCSTIM_OUTPUT_STAGE_H
#define FOCSTIM_OUTPUT_STAGE_H

#include "transformers.h"


struct OutputStage {
    float resistance;   // driver Rdson + inductor resistance + traces (Ohm)
    float inductance;   // Inductance of low-pass filter circuit (Henry)
    float capacitance;  // Capacitance of low-pass filter circuit (Farad)
    Transformer transformer;

    // compute the body impedance, from total circuit impedance
    Complex body_impedance(Complex total_impedance, float frequency) const;

    // compute total circuit impedance, from body impedance
    Complex total_impedance(Complex body_impedance, float frequency) const;

    float power_total(float rms_current, Complex total_impedance, float frequency) const;
    float power_skin(float rms_current, Complex total_impedance, float frequency) const;

    Complex convert_impedance(Complex z, float original_frequency, float end_frequency);

    // not all current that enters the transformer makes it to the secondary,
    // some of it is lost inside the magnetizing impedance branch.
    // the ratio of (output current / input current) therefore is slightly higher
    // than the actual winding ratio.
    float apparent_winding_ratio() const;
};


#endif