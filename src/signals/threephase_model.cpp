#include "signals/threephase_model.h"

#include "bsp/bsp.h"
#include "foc_utils.h"
#include "utils.h"

#include "stim_clock.h"

float find_v_drive(Complex p1, Complex p2, Complex p3) {
    // TODO: exact method exists.
    int steps = 500;
    Complex proj(1, 0);
    Complex rotator(cosf(_2PI / float(steps)), sinf(_2PI / float(steps)));
    float v_drive = 0;
    for (int i = 0; i < steps; i++) {
        float a = (p1 * proj).a;
        float b = (p2 * proj).a;
        float c = (p3 * proj).a;
        float range = max({a, b, c}) - min({a, b, c});
        v_drive = max(range, v_drive);
        proj = proj * rotator;
    }

    return v_drive;
}

void ThreephaseModel::init(std::function<void()> emergency_stop_fn) {
    this->emergency_stop_fn = emergency_stop_fn;
}

void ThreephaseModel::play_pulse(
    Complex p1, Complex p2, Complex p3, 
    float carrier_frequency,
    float pulse_width, float rise_time,
    float estop_current_limit)
{
    if ((p1 + p2 + p3).norm() > .001f) {
        Serial.printf("Invalid pulse coordinates");
        return;
    }

    // reset variables for pulse playback
    magnitude_error1 = 0;
    magnitude_error2 = 0;
    magnitude_error3 = 0;
    angle_error1 = 0;
    angle_error2 = 0;
    angle_error3 = 0;
    producer_index = 0;
    interrupt_index = 0;
    updater_index = 0;
    interrupt_finished = false;
    skipped_update_steps = 0;
    current_limit_exceeded = 0;
    current_limit = estop_current_limit;

    // make debug easier by clearing out stale data.
    for (int i = 0; i < CONTEXT_SIZE; i++) {
        context[i] = {};
    }
    
    // compute voltage with these equations:
    // p1 * z1 = v1 - N
    // p2 * z2 = v2 - N
    // p3 * z3 = v3 - N
    // v1 + v2 + v3 = 0
    Complex N = (p1 * z1 + p2 * z2 + p3 * z3) * (-1.0f/3);
    Complex v1 = p1 * z1 + N;
    Complex v2 = p2 * z2 + N;
    Complex v3 = p3 * z3 + N;

    // check for max vdrive
    float v_drive = find_v_drive(v1, v2, v3);
    if (v_drive > STIM_PSU_VOLTAGE * STIM_PWM_MAX_DUTY_CYCLE) {
        // if vdrive is too high, reduce current/voltage
        float factor = (STIM_PSU_VOLTAGE * STIM_PWM_MAX_DUTY_CYCLE) / v_drive;
        p1 = p1 * factor;
        p2 = p2 * factor;
        p3 = p3 * factor;
        N = N * factor;
        v1 = v1 * factor;
        v2 = v2 * factor;
        v3 = v3 * factor;
        v_drive = v_drive * factor;
    }
    v_drive_max = max(v_drive, v_drive_max);

    // compute pulse length etc.
    int samples = int(STIM_PWM_FREQ * pulse_width / carrier_frequency);
    pulse_length_samples = samples;

    if (rise_time * 2 >= pulse_width) {
        rise_time = pulse_width / 2;
    }
    int rise_endpoint = int(STIM_PWM_FREQ * rise_time / carrier_frequency);
    int fall_begin = samples - rise_endpoint;

    const float dt = 1 / float(STIM_PWM_FREQ);
    Complex proj(1, 0);
    Complex rotator(cosf(dt * _2PI * carrier_frequency), sinf(dt * _2PI * carrier_frequency));
    Complex envelope(0, -1);
    Complex envelope_rotator(cosf(_2PI / rise_endpoint / 4), sinf(_2PI / rise_endpoint / 4));


    // enable pwm and interrupt
    BSP_SetPWM3Atomic(.5f, .5f, .5f);
    BSP_AttachPWMInterrupt([&]{interrupt_fn();});

    // start consumer, pulse plays in backgroud
    for (int i = 0; i < samples; i++) {
        if (i < rise_endpoint) {
            envelope = envelope * envelope_rotator;
        } else if (i >= fall_begin) {
            envelope = envelope * envelope_rotator;
        } else {
            envelope = Complex(1, 0);
        }
        
        Complex q = proj * envelope.a;
        proj = proj * rotator;
        context[i % CONTEXT_SIZE].i1_cmd = (p1 * q).a;  // desired current, used for update step.
        context[i % CONTEXT_SIZE].i2_cmd = (p2 * q).a;
        context[i % CONTEXT_SIZE].i3_cmd = (p3 * q).a;
        context[i % CONTEXT_SIZE].v1_cmd = (v1 * q).a;  // cmd voltage.
        context[i % CONTEXT_SIZE].v2_cmd = (v2 * q).a;
        context[i % CONTEXT_SIZE].v3_cmd = (v3 * q).a;
        atomic_signal_fence(std::memory_order_release);
        producer_index = i;

        // perform update steps when queue has plenty of items.
        while ((i - interrupt_index) >= max_producer_queue_length && !interrupt_finished) {
            perform_one_update_step();
        }

        if (interrupt_finished) {
            break;
        }
    }
    if (!interrupt_finished) {
        // add little tail of zero-voltage commands at the end of the pulse.
        context[samples + 1 % CONTEXT_SIZE] = {};
        context[samples + 2 % CONTEXT_SIZE] = {};
        producer_index = samples + 2;
    }

    // wait until pulse completion
    while (!interrupt_finished) {
        perform_one_update_step();
    }

    // stop pwm, all phases to ground.
    BSP_AttachPWMInterrupt(nullptr);
    BSP_SetPWM3Atomic(0, 0, 0);

    if (current_limit_exceeded) {
        BSP_DisableOutputs();
        Serial.printf("Current limit exceeded\r\n");
        int i = (interrupt_index - 2) % CONTEXT_SIZE;
        Serial.printf("currents were: %f %f %f\r\n", context[i].i1_meas, context[i].i2_meas, context[i].i3_meas);
        emergency_stop_fn();
        while(1) {}
    }

    if (interrupt_index != producer_index) {
        BSP_DisableOutputs();
        Serial.printf("Producer too slow %i %i\r\n", (int)interrupt_index, (int)producer_index);
        emergency_stop_fn();
        while(1) {
        }
    }

    // process all update steps.
    while (updater_index != (interrupt_index - 2)) {
        perform_one_update_step();
    }

    // apply impedance magnitude error
    // done in special way to avoid drift when the input signal
    // is not sufficienctly exciting.
    Complex d1 = p1 * (1 / max(p1.norm(), 0.01f));
    Complex d2 = p2 * (1 / max(p2.norm(), 0.01f));
    Complex d3 = p3 * (1 / max(p3.norm(), 0.01f));
    {
        float zz1 = abs(dot(d1, p1)) * magnitude_error1 + abs(dot(d2, p1)) * magnitude_error2 + abs(dot(d3, p1)) * magnitude_error3;
        float zz2 = abs(dot(d1, p2)) * magnitude_error1 + abs(dot(d2, p2)) * magnitude_error2 + abs(dot(d3, p2)) * magnitude_error3;
        float zz3 = abs(dot(d1, p3)) * magnitude_error1 + abs(dot(d2, p3)) * magnitude_error2 + abs(dot(d3, p3)) * magnitude_error3;
        z1 = z1 * (1 + zz1);
        z2 = z2 * (1 + zz2);
        z3 = z3 * (1 + zz3);
    }
    
    // apply impedance angle error
    {
        float zz1 = abs(dot(d1, p1)) * angle_error1 + abs(dot(d2, p1)) * angle_error2 + abs(dot(d3, p1)) * angle_error3;
        float zz2 = abs(dot(d1, p2)) * angle_error1 + abs(dot(d2, p2)) * angle_error2 + abs(dot(d3, p2)) * angle_error3;
        float zz3 = abs(dot(d1, p3)) * angle_error1 + abs(dot(d2, p3)) * angle_error2 + abs(dot(d3, p3)) * angle_error3;
        z1 = z1 * Complex(cosf(zz1), sinf(zz1));
        z2 = z2 * Complex(cosf(zz2), sinf(zz2));
        z3 = z3 * Complex(cosf(zz3), sinf(zz3));
    }

    // constrain impedance magnitude/angle
    z1.constrain_in_bound(MODEL_RESISTANCE_MIN, MODEL_RESISTANCE_MAX, MODEL_PHASE_ANGLE_MIN, MODEL_PHASE_ANGLE_MAX);
    z2.constrain_in_bound(MODEL_RESISTANCE_MIN, MODEL_RESISTANCE_MAX, MODEL_PHASE_ANGLE_MIN, MODEL_PHASE_ANGLE_MAX);
    z3.constrain_in_bound(MODEL_RESISTANCE_MIN, MODEL_RESISTANCE_MAX, MODEL_PHASE_ANGLE_MIN, MODEL_PHASE_ANGLE_MAX);
}

Vec3f ThreephaseModel::estimate_rms_current(float dt)
{
    dt = dt * STIM_PWM_FREQ;
#if defined(CURRENT_SENSE_SCALE_FULL)
    return Vec3f(
        sqrtf(current_squared.a / dt),
        sqrtf(current_squared.b / dt),
        sqrtf(current_squared.c / dt)
    );
#elif defined(CURRENT_SENSE_SCALE_HALF)
    return Vec3f(
        sqrtf(current_squared.a / dt) * _SQRT2,
        sqrtf(current_squared.b / dt) * _SQRT2,
        sqrtf(current_squared.c / dt) * _SQRT2
    );
#else
#error unknown current sense method
#endif
}

void ThreephaseModel::debug_stats_teleplot()
{
    int start_index = max(0, producer_index - CONTEXT_SIZE + 1);
    for (int i = start_index; i <= producer_index; i++) {
        const auto &c = context[i % CONTEXT_SIZE];

        Serial.printf("$");
        Serial.printf("v1:%u:%.2f|xy ", i, c.v1_cmd);
        Serial.printf("v2:%u:%.2f|xy ", i, c.v2_cmd);
        Serial.printf("v3:%u:%.2f|xy ", i, c.v3_cmd);
        Serial.printf("i1_cmd:%u:%.4f|xy ", i, c.i1_cmd);
        Serial.printf("i2_cmd:%u:%.4f|xy ", i, c.i2_cmd);
        Serial.printf("i3_cmd:%u:%.4f|xy ", i, c.i3_cmd);
        Serial.printf("i1:%u:%.4f|xy ", i, c.i1_meas);
        Serial.printf("i2:%u:%.4f|xy ", i, c.i2_meas);
        Serial.printf("i3:%u:%.4f|xy ", i, c.i3_meas);
        Serial.println();
    }
}

void ThreephaseModel::interrupt_fn()
{
    Vec3f currents = BSP_ReadPhaseCurrents3();
    int i = interrupt_index;
    int queued_items = producer_index - i;
    std::atomic_signal_fence(std::memory_order_acquire);

    if (interrupt_finished) {
        return; 
    }
    if (queued_items == 0) {
        if (i == 0) {
            // pulse just started, no items produced yet.
            return;
        }

        // end of produced items reached. Either end of pulse or race condition.
        interrupt_finished = true;
        return;
    }

    int read_index = i % CONTEXT_SIZE;
    int write_index = (i - 2) % CONTEXT_SIZE;
    if (i == 0) {
        if (queued_items < interrupt_headstart) {
            return; // wait for the queue to be populated with at least a few items before starting.
        }
    }
    
    // compute duty cycle center
    float v_min = min({
        context[read_index].v1_cmd,
        context[read_index].v2_cmd,
        context[read_index].v3_cmd,
    });
    float v_max = max({
        context[read_index].v1_cmd,
        context[read_index].v2_cmd,
        context[read_index].v3_cmd,
    });
    float center = STIM_PSU_VOLTAGE / 2;
    if (center + v_max > (STIM_PSU_VOLTAGE * STIM_PWM_MAX_DUTY_CYCLE)) {
        center = (STIM_PSU_VOLTAGE * STIM_PWM_MAX_DUTY_CYCLE) - v_max;
    }

    // write pwm
    BSP_SetPWM3(
        (context[read_index].v1_cmd + center) / STIM_PSU_VOLTAGE, // TODO: precompute?
        (context[read_index].v2_cmd + center) / STIM_PSU_VOLTAGE,
        (context[read_index].v3_cmd + center) / STIM_PSU_VOLTAGE
    );
    
    // read currents
    if (i >= 2) {    
        context[write_index].i1_meas = currents.a;
        context[write_index].i2_meas = currents.b;
        context[write_index].i3_meas = currents.c;

        // log stats
        current_squared = Vec3f(
            current_squared.a + currents.a * currents.a,
            current_squared.b + currents.b * currents.b,
            current_squared.c + currents.c * currents.c
        );
        current_max = Vec3f(
            max(current_max.a, abs(currents.a)),
            max(current_max.b, abs(currents.b)),
            max(current_max.c, abs(currents.c))
        );
    }
    
    // check current limits
    if (abs(currents.a) > current_limit ||
        abs(currents.b) > current_limit ||
        abs(currents.c) > current_limit)
    {
        BSP_DisableOutputs();
        current_limit_exceeded = true;
        interrupt_finished = true;
        return;
    }

    atomic_signal_fence(std::memory_order_release);
    interrupt_index = i + 1;
}

void ThreephaseModel::perform_one_update_step()
{
    int samples_available = (interrupt_index - 2) - updater_index;
    if (samples_available <= 0) {
        return;
    }
    if (samples_available >= max_updater_lag) {
        skipped_update_steps++;
        updater_index++;
    }
    int i = updater_index++;

    if (i == 0) {
        return;
    }

    atomic_signal_fence(std::memory_order_acquire);

    int i_plus_one = (i + 1) % CONTEXT_SIZE;
    int i_minus_one = (i - 1) % CONTEXT_SIZE;
    i = i % CONTEXT_SIZE;


    float err1 = context[i].i1_meas - context[i].i1_cmd;
    float err2 = context[i].i2_meas - context[i].i2_cmd;
    float err3 = context[i].i3_meas - context[i].i3_cmd;

    float dx1 = context[i_plus_one].i1_cmd - context[i_minus_one].i1_cmd;    // derivative
    float dx2 = context[i_plus_one].i2_cmd - context[i_minus_one].i2_cmd;
    float dx3 = context[i_plus_one].i3_cmd - context[i_minus_one].i3_cmd;



#if defined(CURRENT_SENSE_SCALE_FULL)
    const float minimum_current = infinityf();
#elif defined(CURRENT_SENSE_SCALE_HALF)
    const float minimum_current = -0.02f;
#else
#error unknown current sense method
#endif

    float gamma1 = -0.1f;
    float gamma2 = -0.1f;
    if (context[i].i1_cmd < minimum_current) {
        magnitude_error1 += gamma1 * (context[i].i1_cmd * err1);
        angle_error1 += gamma2 * (dx1 * err1);
    }
    if (context[i].i2_cmd < minimum_current) {
        magnitude_error2 += gamma1 * (context[i].i2_cmd * err2);
        angle_error2 += gamma2 * (dx2 * err2);
    }
    if (context[i].i3_cmd < minimum_current) {
        magnitude_error3 += gamma1 * (context[i].i3_cmd * err3);
        angle_error3 += gamma2 * (dx3 * err3);
    }
}
