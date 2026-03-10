#ifndef FOCSTIM_FOURPHASE_MODEL
#define FOCSTIM_FOURPHASE_MODEL

#include "bsp/bsp.h"

#if defined(BSP_ENABLE_FOURPHASE)

#include "complex.h"
#include "vec.h"
#include "foc_error.h"
#include "signals/output_limits.h"

#include <atomic>

class FourphaseModel {
public:
    FourphaseModel() {};

    void init(std::function<void(FOCError)> emergency_stop_fn);

    void play_pulse(
        Complex p1, Complex p2, Complex p3, Complex p4,
        float carrier_frequency,
        float pulse_width, float rise_time,
        float estop_current_limit, OutputLimits limits);

    Vec4f estimate_rms_current(float dt);

    void debug_stats_teleplot();

    void interrupt_fn();
    void accumulate_errors();
    void model_update(Complex p1, Complex p2, Complex p3, Complex p4);

    // electrode impedance
    Complex z1 = Complex(0, 0);
    Complex z2 = Complex(0, 0);
    Complex z3 = Complex(0, 0);
    Complex z4 = Complex(0, 0);


    // stats for one pulse, reset after every pulse.
    struct {
        Vec4f current_squared = Vec4f(0, 0, 0, 0);
        Vec4f current_max = Vec4f(0, 0, 0, 0);
        float v_bus_min = 0;    // the min/max bus voltage observed during the pulse
        float v_bus_max = 0;
        float v_drive_requested = 0;
        float v_drive_actual = 0;         // v_drive of the pulse
        float volt_seconds = 0;
    } pulse_stats;

    // total stats, can be reset by caller.
    struct {
        Vec4f current_squared = Vec4f(0, 0, 0, 0);
        Vec4f current_max = Vec4f(0, 0, 0, 0);
        float volt_seconds;
    } total_stats;

    static constexpr int CONTEXT_SIZE = 256;
    static constexpr int max_producer_queue_length = 20;    // (producer_index - interrupt_index) % CONTEXT_SIZE
    static constexpr int min_producer_queue_length = 10;    // (producer_index - interrupt_index) % CONTEXT_SIZE
    static constexpr int interrupt_headstart = 20;          // (producer_index - interrupt_index) % CONTEXT_SIZE
    static constexpr int max_updater_lag = 50;              // (interrupt_index - updater_index)  % CONTEXT_SIZE

    int producer_index;
    int updater_index;
    volatile int interrupt_index;
    volatile bool interrupt_finished;
    int skipped_update_steps;
    int pulse_length_samples = 0;

    float current_limit;
    bool current_limit_exceeded;

    struct {
        float sine;     // just sin(theta)
        float cosine;   // just cos(theta)

        float i1_cmd;   // commanded current
        float i2_cmd;
        float i3_cmd;
        float i4_cmd;

        float v1_cmd;   // commanded voltage
        float v2_cmd;
        float v3_cmd;
        float v4_cmd;

        float v1_cmd_quadrature;   // commanded voltage, shifted 90 deg
        float v2_cmd_quadrature;
        float v3_cmd_quadrature;
        float v4_cmd_quadrature;

        float i1_meas;  // measured current
        float i2_meas;
        float i3_meas;
        float i4_meas;
    } context[CONTEXT_SIZE];


    Complex cmd_IQ_1 = {};
    Complex cmd_IQ_2 = {};
    Complex cmd_IQ_3 = {};
    Complex cmd_IQ_4 = {};

    Complex meas_IQ_1 = {};
    Complex meas_IQ_2 = {};
    Complex meas_IQ_3 = {};
    Complex meas_IQ_4 = {};

    Complex phase_IQ_1 = {};
    Complex phase_IQ_2 = {};
    Complex phase_IQ_3 = {};
    Complex phase_IQ_4 = {};

    Complex phase_IQ_avg_1 = {};
    Complex phase_IQ_avg_2 = {};
    Complex phase_IQ_avg_3 = {};
    Complex phase_IQ_avg_4 = {};

    Vec4f integrated_cmd = {};
    Vec4f integrated_meas = {};

    std::function<void(FOCError)> emergency_stop_fn;
};

#endif
#endif