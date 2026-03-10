#include "output_stage.h"

/**
 * We model the output stage as:
 * Z_output, driver and inductor
 * Z_cap, low-pass filter capacitor.
 * Z_leak, core loss and magnetizing reluctance of the transformer.
 * Z_windings, copper losses + skin resistance
 *
 * Z_output is in series, Z_cap, Z_leak and Z_body parallel.
 */
struct Impedances {
    Impedances(OutputStage const *stage, float frequency) {
        this->stage = stage;
        Z_output = Complex(stage->resistance, stage->inductance * float(2 * M_PI) * frequency);
        Z_cap = Complex(0, 1 / -(stage->capacitance * float(2 * M_PI) * frequency));
        Z_leak = stage->transformer.magnetizing_impedance(frequency);
    }

    void fill_from_body(Complex body_impedance) {
        Z_body = body_impedance;
        Z_windings = stage->transformer.resistance_lv + (Z_body + stage->transformer.resistance_hv) / powf(stage->transformer.winding_ratio, 2);
        Z_parallel = 1.f/(1.f/Z_leak + 1.f/Z_cap + 1.f/Z_windings);
    }

    void fill_from_total(Complex Z_tot) {
        Z_parallel = Z_tot - Z_output;
        Z_windings = 1.f/(1.f / Z_parallel - 1.f / Z_cap - 1.f / Z_leak);
        Z_body = (Z_windings - stage->transformer.resistance_lv) * powf(stage->transformer.winding_ratio, 2) - stage->transformer.winding_ratio;
    }

    OutputStage const *stage ;
    Complex Z_output;
    Complex Z_cap;
    Complex Z_leak;

    Complex Z_parallel;
    Complex Z_body;
    Complex Z_windings;
};


Complex OutputStage::body_impedance(Complex total_impedance, float frequency) const
{
    Impedances imp(this, frequency);
    imp.fill_from_total(total_impedance);
    return imp.Z_body;
}

Complex OutputStage::total_impedance(Complex body_impedance, float frequency) const
{
    Impedances imp(this, frequency);
    imp.fill_from_body(body_impedance);
    return imp.Z_output + imp.Z_parallel;
}

float OutputStage::power_total(float rms_current, Complex total_impedance, float frequency) const
{
    return powf(rms_current, 2) * total_impedance.real();
}

float OutputStage::power_skin(float rms_current, Complex total_impedance, float frequency) const
{
    Impedances imp(this, frequency);
    imp.fill_from_total(total_impedance);
    float ratio = std::abs(imp.Z_parallel) / std::abs(imp.Z_windings);
    return powf(rms_current * ratio / transformer.winding_ratio, 2) * imp.Z_body.real();
}

float OutputStage::apparent_winding_ratio() const
{
    Impedances imp(this, 1000);
    imp.fill_from_body(220);
    return transformer.winding_ratio * std::abs(imp.Z_windings / imp.Z_parallel);
}
