#include "transformers.h"

Complex xicon_42TLxxx_magnetizing_impedance(float frequency)
{
    // raw data collected from 42TL001. The 42TL004 matches very closely,
    // since it's the same core and number of windings on the primary
    // 2000hz, 2.0 vrms: Z = 45.9 ohm at 36.3deg phase shift.
    // 1500hz, 1.5 vrms: Z = 38.8 ohm at 39.3deg phase shift.
    // 1000hz, 1.0 vrms: Z = 30.6 ohm at 43.2deg phase shift.
    // 500hz,  0.5 vrms: Z = 18.9 ohm at 47.9deg phase shift.
    // 300hz,  0.3 vrms: Z = 13.2 ohm at 53.5deg phase shift.

    // test setup: signal generator (TPA3116) outputting sine wave on low-voltage side,
    // high-voltage side unconnected ('open circuit')
    // multimeter for measuring rms voltage and rms voltage over a current shunt.
    // oscilloscope to inspect the phase shift.

    // This is at approx 25% of power where first signs of saturation shows.
    // At 5% power, the numbers are about 20% lower, but assuming impedance
    // is independent of power makes later code substantially simpler.

    // equation repressents best-fit of above data without the transformer DC resistance.
    float re = .04959f * powf(frequency, .8710f);
    float im = .09598f * powf(frequency, -.4935f) * float(2 * M_PI) * frequency;
    return Complex(re, im);
}

Complex xicon_42TUxxx_magnetizing_impedance(float frequency)
{
    // TODO: verify
    return xicon_42TLxxx_magnetizing_impedance(frequency) * 4.f;
}
