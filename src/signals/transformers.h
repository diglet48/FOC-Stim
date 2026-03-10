#ifndef FOCSTIM_TRANSFORMERS_H
#define FOCSTIM_TRANSFORMERS_H

#include "complex.h"

#include <functional>

typedef Complex (*magnetizing_impedance_fn)(float);

struct Transformer {
    float resistance_lv; // ohm DC, low-voltage side
    float resistance_hv; // ohm DC, high-voltage side. This is the primary in the datasheet.
    float winding_ratio; // true winding ratio
    float current_ratio; // the ratio of input and output current with average loading. See OutputStage::apparent_winding_ratio()

    float saturation_lv; // V*µs, measured at the low-voltage side

    magnetizing_impedance_fn magnetizing_impedance;
};

Complex xicon_42TLxxx_magnetizing_impedance(float frequency);
Complex xicon_42TUxxx_magnetizing_impedance(float frequency);

static constexpr Transformer XICON_42TL004{   // default on V4
    .resistance_lv = 0.99f, // variance ~ 0.01 ohm
    .resistance_hv = 12.0,  // range 11.75 - 12.5 on few samples
    .winding_ratio = 5.57,  // variance ~ 0.01
    .current_ratio = 6.66f, // selected for historical reasons, close to 220 ohm load.
    .saturation_lv = 900e-6f,
    .magnetizing_impedance = xicon_42TLxxx_magnetizing_impedance,
};

static constexpr Transformer XICON_42TL001{
    .resistance_lv = 1.00f, // variance ~ 0.01 ohm
    .resistance_hv = 35.0,
    .winding_ratio = 8.6,
    .current_ratio = 9.37f, // with 220 ohm load
    .saturation_lv = 900e-6f,
    .magnetizing_impedance = xicon_42TLxxx_magnetizing_impedance,
};

static constexpr Transformer EBAY_42TL004_KNOCKOFF{
    .resistance_lv = 1.1,
    .resistance_hv = 19.0,
    .winding_ratio = 5.0,
    .current_ratio = 6.3f, // with 220 ohm load
    .saturation_lv = 900e-6f,
    .magnetizing_impedance = xicon_42TLxxx_magnetizing_impedance,   // not tested, but probably close
};

static constexpr Transformer XICON_42TU200_MIDDLE_PIN{    // recommended for V1
    .resistance_lv = 0.25,
    .resistance_hv = 12.0,
    .winding_ratio = 10.0,
    .current_ratio = 10.0, // selected for historical reasons.
    .saturation_lv = 3600e-6f,
    .magnetizing_impedance = xicon_42TUxxx_magnetizing_impedance,   // not tested, probably very far off.
};

#endif