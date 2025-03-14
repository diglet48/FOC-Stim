#ifndef FOCSTIM_MRAC_THREEPHASE_STAR_H
#define FOCSTIM_MRAC_THREEPHASE_STAR_H

#include <stdint.h>
#include <functional>

#include "vec.h"
#include "config.h"


class EmergencyStop;
class ThreephasePulseBuffer;

/*
MRAC model for threephase system with:
star connection with floating neutral
each leg has the same inductance
eacn leg has it's own resistance


Va, Vb, Vc      = driving voltage.
N               = neutral point voltage (unknown)
Ra, Rb, Rc      = leg resistance
L               = leg inductance
Ia, Ib, Ic      = leg current
Iad, Ibd, Icd   = derivative of the leg current


Modelling:
Va - N = Ra * Ia + L * Iad       and vice-versa for b, c
Consider:
(Va - N) - (Vb - N) = Ra * Ia + L * Iad - Rb * Ib - L * Ibd
(Vb - N) - (Vc - N) = Rb * Ib + L * Ibd - Rc * Ic - L * Icd
Substitude C = -(A + B) and Ic = -(Ia + Ib) and Icd = -(Iad + Ibd) then solve:

    Va, Vb, N = symbols('Va, Vb, N')
    Ra, Rb, Rc = symbols('Ra, Rb, Rc')
    Ia, Ib = symbols('Ia, Ib')
    Iad, Ibd = symbols('Iad, Ibd')
    L = symbols('L')
    Vc = -(Va + Vb)
    Ic = -(Ia + Ib)
    Icd = -(Iad + Ibd)
    p1 = Eq(Va - Vb, Ra * Ia + L * Iad - Rb * Ib - L * Ibd)
    p2 = Eq(Vb - Vc, Rb * Ib + L * Ibd - Rc * Ic - L * Icd)
    solution = solve([p1, p2], [Iad, Ibd])
    pprint(solution[Iad])
    pprint(solution[Ibd])

          /------------ matrix A ----------\             /- matrix B -\
[[Iad]  = 1/(3*L) * [[-2Ra - Rc,    Rb - Rc]  * [[Ia]  + 1/L [[1,    0]  * [[Va]
 [Ibd]]              [  Ra - Rc,  -2Rb - Rc]]    [Ib]]        [0,    1]]    [Vb]]

Reference model:
xd = Am * x + Bm * r

Control law:
u = Kr * r - Kx * x

plant model:
xd = A * x + B * u
xd = A * x + B * [Kr * r - Kx * x]

Model matching conditions:
Bm = B * Kr                 Kr a scalar
Am = A - B * Kx             Kx a 2x2 matrix

After solving model matching condition 1 we find:
L = L0 * Kr

We choose Kx = Ka * [2  0] + Kb * [0 -1] + Kc * [1  1]
                    [-1 0]        [0  2]        [1  1]
After solving model matching condition 2 we find:
R0 * Kr - 3 * Ka = Ra
R0 * Kr - 3 * Kb = Rb
R0 * Kr - 3 * Kc = Rc
*/
class MRACThreephaseStar
{
public:
    static constexpr float L0 = MODEL_INDUCTANCE_INIT;
    static constexpr float L_min = MODEL_INDUCTANCE_MIN;
    static constexpr float L_max = MODEL_INDUCTANCE_MAX;
    static constexpr float R0 = MODEL_RESISTANCE_INIT;
    static constexpr float R_min = MODEL_RESISTANCE_MIN;
    static constexpr float R_max = MODEL_RESISTANCE_MAX;

    static constexpr float A = -R0 / L0;
    static constexpr float B = 1 / L0;
    static constexpr float P = -1;

    static constexpr unsigned context_size = 512;
    static constexpr unsigned model_headstart = 10;         // number of model calculations before starting interrupt
    static constexpr unsigned maximum_interrupt_lag = 15;   // maximum distance between model calculations and interrupt
    static constexpr unsigned maximum_update_lag = 10;      // maximum distance between interrupt and model update
    static_assert(context_size >= (maximum_interrupt_lag + maximum_update_lag + 5));

    MRACThreephaseStar();

    void init(std::function<void()> emergency_stop_fn);

    void play_pulse(ThreephasePulseBuffer *pulse, float estop_current_limit);

    void interrupt_fn();

    float estimate_inductance()
    {
        return L0 * Kr;
        // The true inductance is actually dt / ln(1 - dt / (L0 * Kr))
    }

    Vec3f estimate_resistance() {
        return Vec3f(
            (R0 * Kr - 3 * Ka),
            (R0 * Kr - 3 * Kb),
            (R0 * Kr - 3 * Kc)
        );
    }

    Vec3f estimate_rms_current(float dt);

    void print_debug_stats();

    void perform_model_update_step();



    // MRAC variables.
    float Kr = 1;                 // feedforward gain, true inductance = L = L0 * Kr
    float Ka = 0;                 // R0 * Kr - 3 * Ka = Ra
    float Kb = 0;                 // R0 * Kr - 3 * Kb = Rb
    float Kc = 0;                 // R0 * Kr - 3 * Kc = Rc

    // log stats
    float v_drive_max = 0;
    Vec3f current_squared;
    Vec3f current_max;
    float estop_current_limit = 0;

    // TODO: not volatile
    volatile struct {
        // the voltages to be written to pwm by the interrupt
        // indices before 'pwm_write_index' are valid.
        float pwm_voltage_neutral;
        float pwm_voltage_left;
        float pwm_voltage_right;

        // the adc currents read by the interrupt
        // indices before `interrupt_index - 2` are valid.
        float adc_current_neutral;
        float adc_current_left;
        float adc_current_right;

        // variables needed for update step
        float xHat_a;   // current a according to model
        float xHat_b;   // current b according to model
        float r_a;      // reference control signal
        float r_b;      // reference control signal
    } context[context_size];

    volatile int pwm_write_index = 0;
    volatile int interrupt_index = 0;
    int model_update_index = 0;
    int skipped_update_steps = 0;

    volatile bool buffer_underrun_detected = false;
    volatile bool current_limit_exceeded = false;

    std::function<void()> emergency_stop_fn;
};

#endif // FOCSTIM_MRAC_THREEPHASE_STAR_H