#ifndef FOCSTIM_THREEPHASE_MODEL
#define FOCSTIM_THREEPHASE_MODEL

#include "complex.h"
#include "vec.h"
#include "bsp/bsp.h"

#include <atomic>

class ThreephaseModel {
public:
    ThreephaseModel() {};

    void init(std::function<void()> emergency_stop_fn);

    void play_pulse(
        Complex p1, Complex p2, Complex p3,
        float carrier_frequency,
        float pulse_width, float rise_time,
        float estop_current_limit);

    Vec3f estimate_rms_current(float dt);

    void debug_stats_teleplot();

    void interrupt_fn();
    void perform_one_update_step();


    // TODO: limits, init
    Complex z1 = Complex(MODEL_RESISTANCE_INIT, 0); // impedance of first output
    Complex z2 = Complex(MODEL_RESISTANCE_INIT, 0); // impedance of second output
    Complex z3 = Complex(MODEL_RESISTANCE_INIT, 0); // impedance of third output


    // log stats
    float v_drive_last = 0;
    float v_min = 0;
    float v_max = 0;
    Vec3f current_squared = Vec3f(0, 0, 0);
    Vec3f current_max = Vec3f(0, 0, 0);

    static constexpr int CONTEXT_SIZE = 512;
    static constexpr int max_producer_queue_length = 20;    // (producer_index - interrupt_index) % CONTEXT_SIZE
    static constexpr int min_producer_queue_length = 10;    // (producer_index - interrupt_index) % CONTEXT_SIZE
    static constexpr int interrupt_headstart = 20;          // (producer_index - interrupt_index) % CONTEXT_SIZE
    static constexpr int max_updater_lag = 50;              // (interrupt_index - updater_index)  % CONTEXT_SIZE

    int producer_index;
    int updater_index;
    volatile int interrupt_index;
    volatile bool interrupt_finished;
    int skipped_update_steps;

    float current_limit;
    bool current_limit_exceeded;

    struct {
        float i1_cmd;   // commanded current
        float i2_cmd;
        float i3_cmd;

        float v1_cmd;   // commanded voltage
        float v2_cmd;
        float v3_cmd;

        float i1_meas;  // measured current
        float i2_meas;
        float i3_meas;
    } context[CONTEXT_SIZE];

    int pulse_length_samples = 0;

    float magnitude_error1 = 0;
    float magnitude_error2 = 0;
    float magnitude_error3 = 0;
    float angle_error1 = 0;
    float angle_error2 = 0;
    float angle_error3 = 0;

    std::function<void()> emergency_stop_fn;
};

#endif