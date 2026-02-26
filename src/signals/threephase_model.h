#ifndef FOCSTIM_THREEPHASE_MODEL
#define FOCSTIM_THREEPHASE_MODEL

#include "complex.h"
#include "vec.h"
#include "foc_error.h"
#include "bsp/bsp.h"

#include <atomic>

class ThreephaseModel {
public:
    ThreephaseModel() {};

    void init(std::function<void(FOCError)> emergency_stop_fn);

    void play_pulse(
        Complex p1, Complex p2, Complex p3,
        float carrier_frequency,
        float pulse_width, float rise_time,
        float estop_current_limit);

    Vec3f estimate_rms_current(float dt);

    void debug_stats_teleplot();

    void interrupt_fn();
    void accumulate_errors();
    void model_update(Complex p1, Complex p2, Complex p3, Complex v1, Complex v2, Complex v4);

    Complex z1 = Complex(0, 0); // impedance of first output
    Complex z2 = Complex(0, 0); // impedance of second output
    Complex z3 = Complex(0, 0); // impedance of third output

    // stats for one pulse, reset after every pulse.
    struct {
        Vec3f current_squared = Vec3f(0, 0, 0);
        Vec3f current_max = Vec3f(0, 0, 0);
        float v_bus_min = 0;    // the min/max bus voltage observed during the pulse
        float v_bus_max = 0;
        float v_drive = 0;      // v_drive of the pulse
        float volt_seconds = 0;
    } pulse_stats;

    // total stats, can be reset by caller.
    struct {
        Vec3f current_squared = Vec3f(0, 0, 0);
        Vec3f current_max = Vec3f(0, 0, 0);
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

        float v1_cmd;   // commanded voltage
        float v2_cmd;
        float v3_cmd;

        float v1_cmd_quadrature;   // commanded voltage, shifted 90 deg
        float v2_cmd_quadrature;
        float v3_cmd_quadrature;

        float i1_meas;  // measured current
        float i2_meas;
        float i3_meas;
    } context[CONTEXT_SIZE];

    Complex cmd_IQ_1 = {};
    Complex cmd_IQ_2 = {};
    Complex cmd_IQ_3 = {};

    Complex meas_IQ_1 = {};
    Complex meas_IQ_2 = {};
    Complex meas_IQ_3 = {};

    Complex phase_IQ_1 = {};
    Complex phase_IQ_2 = {};
    Complex phase_IQ_3 = {};

    Complex phase_IQ_sum_1 = {};
    Complex phase_IQ_sum_2 = {};
    Complex phase_IQ_sum_3 = {};

    Vec3f integrated_cmd = {};
    Vec3f integrated_meas = {};

    std::function<void(FOCError)> emergency_stop_fn;
};

#endif