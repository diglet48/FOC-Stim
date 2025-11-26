#include "signals/fourphase_model.h"

#include "bsp/bsp.h"

#if defined(BSP_ENABLE_FOURPHASE)

#include "foc_utils.h"
#include "utils.h"

#include "stim_clock.h"

float find_v_drive(Complex p1, Complex p2, Complex p3, Complex p4) {
    // TODO: exact method exists.
    int steps = 500;
    Complex proj(1, 0);
    Complex rotator(cosf(_2PI / float(steps)), sinf(_2PI / float(steps)));
    float v_drive = 0;
    for (int i = 0; i < steps; i++) {
        float a = (p1 * proj).a;
        float b = (p2 * proj).a;
        float c = (p3 * proj).a;
        float d = (p4 * proj).a;
        float range = max({a, b, c, d}) - min({a, b, c, d});
        v_drive = max(range, v_drive);
        proj = proj * rotator;
    }

    return v_drive;
}

void FourphaseModel::init(std::function<void(FOCError)> emergency_stop_fn) {
    this->emergency_stop_fn = emergency_stop_fn;
}

void FourphaseModel::play_pulse(
    Complex p1, Complex p2, Complex p3, Complex p4,
    float carrier_frequency,
    float pulse_width, float rise_time,
    float estop_current_limit)
{
    if ((p1 + p2 + p3 + p4).norm() > .001f) {
        BSP_PrintDebugMsg("Invalid pulse coordinates");
        return;
    }

    // reset variables for pulse playback
    magnitude_error1 = 0;
    magnitude_error2 = 0;
    magnitude_error3 = 0;
    magnitude_error4 = 0;
    angle_error1 = 0;
    angle_error2 = 0;
    angle_error3 = 0;
    angle_error4 = 0;
    producer_index = 0;
    interrupt_index = 0;
    updater_index = 0;
    interrupt_finished = false;
    skipped_update_steps = 0;
    current_limit_exceeded = 0;
    current_limit = estop_current_limit;
    v_bus_min = 99;
    v_bus_max = 0;

    // make debug easier by clearing out stale data.
    for (int i = 0; i < CONTEXT_SIZE; i++) {
        context[i] = {};
    }

    // compute voltage with these equations:
    // p1 * z1 = v1 - N
    // p2 * z2 = v2 - N
    // p3 * z3 = v3 - N
    // p4 * z4 = v4 - N
    // v1 + v2 + v3 + v4 = 0
    Complex N = (p1 * z1 + p2 * z2 + p3 * z3 + p4 * z4) * (-1.0f/4);
    Complex v1 = p1 * z1 + N;
    Complex v2 = p2 * z2 + N;
    Complex v3 = p3 * z3 + N;
    Complex v4 = p4 * z4 + N;

    // check for max vdrive
    float v_drive = find_v_drive(v1, v2, v3, v4);
    if (v_drive > STIM_PWM_MAX_VDRIVE) {
        // if vdrive is too high, reduce current/voltage
        float factor = STIM_PWM_MAX_VDRIVE / v_drive;
        p1 = p1 * factor;
        p2 = p2 * factor;
        p3 = p3 * factor;
        p4 = p4 * factor;
        N = N * factor;
        v1 = v1 * factor;
        v2 = v2 * factor;
        v3 = v3 * factor;
        v4 = v4 * factor;
        v_drive = v_drive * factor;
    }
    v_drive_last = v_drive;

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
    BSP_SetPWM4Atomic(.5f, .5f, .5f, .5f);
    BSP_AttachPWMInterrupt([&]{interrupt_fn();});

    // start producer, pulse plays in backgroud
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
        context[i % CONTEXT_SIZE].i4_cmd = (p4 * q).a;
        context[i % CONTEXT_SIZE].v1_cmd = (v1 * q).a;  // cmd voltage.
        context[i % CONTEXT_SIZE].v2_cmd = (v2 * q).a;
        context[i % CONTEXT_SIZE].v3_cmd = (v3 * q).a;
        context[i % CONTEXT_SIZE].v4_cmd = (v4 * q).a;

#if defined(DEADTIME_COMPENSATION_ENABLE)
        // note1: This does not result in current flow if all voltages are zero or very close to zero.
        // note2: Assumes current is not discontinuous. (requires R <= 220µh * max(on_time, off_time)).
        // note3: Assumes current does not change sign during pwm operation.
#if defined(STIM_DYNAMIC_VOLTAGE)
        float dtcomp = DEADTIME_COMPENSATION_PERCENTAGE * BSP_ReadVBus();
#elif defined(STIM_STATIC_VOLTAGE)
        float dtcomp = DEADTIME_COMPENSATION_PERCENTAGE * STIM_PSU_VOLTAGE;
#endif
        context[i % CONTEXT_SIZE].v1_cmd += (context[i % CONTEXT_SIZE].i1_cmd > 0 ? 1 : -1) * dtcomp;
        context[i % CONTEXT_SIZE].v2_cmd += (context[i % CONTEXT_SIZE].i2_cmd > 0 ? 1 : -1) * dtcomp;
        context[i % CONTEXT_SIZE].v3_cmd += (context[i % CONTEXT_SIZE].i3_cmd > 0 ? 1 : -1) * dtcomp;
        context[i % CONTEXT_SIZE].v4_cmd += (context[i % CONTEXT_SIZE].i4_cmd > 0 ? 1 : -1) * dtcomp;
#endif

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
        context[(samples + 1) % CONTEXT_SIZE] = {};
        context[(samples + 2) % CONTEXT_SIZE] = {};
        producer_index = samples + 2;
    }

    // wait until pulse completion
    while (!interrupt_finished) {
        perform_one_update_step();
    }

    // stop pwm, all phases to ground.
    BSP_AttachPWMInterrupt(nullptr);
    BSP_SetPWM4Atomic(0, 0, 0, 0);

    if (current_limit_exceeded) {
        BSP_DisableOutputs();
        BSP_PrintDebugMsg("Current limit exceeded");
        int i = (interrupt_index - 2) % CONTEXT_SIZE;
        BSP_PrintDebugMsg("currents were: %f %f %f %f", context[i].i1_meas, context[i].i2_meas, context[i].i3_meas, context[i].i4_meas);
        emergency_stop_fn(FOCError::OUTPUT_OVER_CURRENT);
        while(1) {}
    }

    if (interrupt_index != producer_index) {
        BSP_DisableOutputs();
        BSP_PrintDebugMsg("Producer too slow %i %i", (int)interrupt_index, (int)producer_index);
        emergency_stop_fn(FOCError::MODEL_TIMING_ERROR);
        while(1) {}
    }

    // process all update steps.
    while (updater_index != (interrupt_index - 2)) {
        perform_one_update_step();
    }

    // TODO: if v_boost dropped too much during the pulse, do not perform update step.

    // apply impedance magnitude error
    // done in special way to avoid drift when the input signal
    // is not sufficienctly exciting.
    Complex d1 = p1 * (1 / max(p1.norm(), 0.01f));
    Complex d2 = p2 * (1 / max(p2.norm(), 0.01f));
    Complex d3 = p3 * (1 / max(p3.norm(), 0.01f));
    Complex d4 = p4 * (1 / max(p4.norm(), 0.01f));
    {
        float zz1 = abs(dot(d1, p1)) * magnitude_error1 + abs(dot(d2, p1)) * magnitude_error2 + abs(dot(d3, p1)) * magnitude_error3 + abs(dot(d4, p1)) * magnitude_error4;
        float zz2 = abs(dot(d1, p2)) * magnitude_error1 + abs(dot(d2, p2)) * magnitude_error2 + abs(dot(d3, p2)) * magnitude_error3 + abs(dot(d4, p2)) * magnitude_error4;
        float zz3 = abs(dot(d1, p3)) * magnitude_error1 + abs(dot(d2, p3)) * magnitude_error2 + abs(dot(d3, p3)) * magnitude_error3 + abs(dot(d4, p3)) * magnitude_error4;
        float zz4 = abs(dot(d1, p4)) * magnitude_error1 + abs(dot(d2, p4)) * magnitude_error2 + abs(dot(d3, p4)) * magnitude_error3 + abs(dot(d4, p4)) * magnitude_error4;
        const float step_size =  .005f;
        zz1 = std::clamp<float>(zz1, -step_size, 1/step_size);
        zz2 = std::clamp<float>(zz2, -step_size, 1/step_size);
        zz3 = std::clamp<float>(zz3, -step_size, 1/step_size);
        zz4 = std::clamp<float>(zz4, -step_size, 1/step_size);
        z1 = z1 * (1 + zz1);
        z2 = z2 * (1 + zz2);
        z3 = z3 * (1 + zz3);
        z4 = z4 * (1 + zz4);
    }

    // apply impedance angle error
    {
        float zz1 = abs(dot(d1, p1)) * angle_error1 + abs(dot(d2, p1)) * angle_error2 + abs(dot(d3, p1)) * angle_error3 + abs(dot(d4, p1)) * angle_error4;
        float zz2 = abs(dot(d1, p2)) * angle_error1 + abs(dot(d2, p2)) * angle_error2 + abs(dot(d3, p2)) * angle_error3 + abs(dot(d4, p2)) * angle_error4;
        float zz3 = abs(dot(d1, p3)) * angle_error1 + abs(dot(d2, p3)) * angle_error2 + abs(dot(d3, p3)) * angle_error3 + abs(dot(d4, p3)) * angle_error4;
        float zz4 = abs(dot(d1, p4)) * angle_error1 + abs(dot(d2, p4)) * angle_error2 + abs(dot(d3, p4)) * angle_error3 + abs(dot(d4, p4)) * angle_error4;
        z1 = z1 * Complex(cosf(zz1), sinf(zz1));
        z2 = z2 * Complex(cosf(zz2), sinf(zz2));
        z3 = z3 * Complex(cosf(zz3), sinf(zz3));
        z4 = z4 * Complex(cosf(zz4), sinf(zz4));
    }

    // constrain impedance magnitude/angle
    z1.constrain_in_bound(MODEL_RESISTANCE_MIN, MODEL_RESISTANCE_MAX, MODEL_PHASE_ANGLE_MIN, MODEL_PHASE_ANGLE_MAX);
    z2.constrain_in_bound(MODEL_RESISTANCE_MIN, MODEL_RESISTANCE_MAX, MODEL_PHASE_ANGLE_MIN, MODEL_PHASE_ANGLE_MAX);
    z3.constrain_in_bound(MODEL_RESISTANCE_MIN, MODEL_RESISTANCE_MAX, MODEL_PHASE_ANGLE_MIN, MODEL_PHASE_ANGLE_MAX);
    z4.constrain_in_bound(MODEL_RESISTANCE_MIN, MODEL_RESISTANCE_MAX, MODEL_PHASE_ANGLE_MIN, MODEL_PHASE_ANGLE_MAX);
}

Vec4f FourphaseModel::estimate_rms_current(float dt)
{
    dt = dt * STIM_PWM_FREQ;
#if defined(CURRENT_SENSE_SCALE_FULL)
    return Vec4f(
        sqrtf(current_squared.a / dt),
        sqrtf(current_squared.b / dt),
        sqrtf(current_squared.c / dt),
        sqrtf(current_squared.d / dt)
    );
#elif defined(CURRENT_SENSE_SCALE_HALF)
    return Vec4f(
        sqrtf(current_squared.a / dt) * _SQRT2,
        sqrtf(current_squared.b / dt) * _SQRT2,
        sqrtf(current_squared.c / dt) * _SQRT2,
        sqrtf(current_squared.d / dt) * _SQRT2
    );
#else
#error unknown current sense method
#endif
}

void FourphaseModel::debug_stats_teleplot()
{
    BSP_PrintDebugMsg("    i     V1     V2     V3     V4 cmd_i1 i2_cmd i3_cmd i4_cmd     i1     i2     i3     i4");
    int start_index = max(0, producer_index - CONTEXT_SIZE + 1);
    for (int i = start_index; i <= producer_index; i++) {
        const auto &c = context[i % CONTEXT_SIZE];

        BSP_PrintDebugMsg(
            "%5i %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f",
            i,
            c.v1_cmd, c.v2_cmd, c.v3_cmd, c.v4_cmd,
            c.i1_cmd, c.i2_cmd, c.i3_cmd, c.i4_cmd,
            c.i1_meas, c.i2_meas, c.i3_meas, c.i4_meas
        );
    }
}

void FourphaseModel::interrupt_fn()
{
    Vec4f currents = BSP_ReadPhaseCurrents4();
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
        context[read_index].v4_cmd,
    });
    float v_max = max({
        context[read_index].v1_cmd,
        context[read_index].v2_cmd,
        context[read_index].v3_cmd,
        context[read_index].v4_cmd,
    });
    // float vbus = 15;
#ifdef STIM_DYNAMIC_VOLTAGE
    float vbus = BSP_ReadVBus();
    v_bus_max = max(v_bus_max, vbus);
    v_bus_min = min(v_bus_min, vbus);
    vbus = max(vbus, STIM_BOOST_VOLTAGE_LOW_THRESHOLD);
#else
    float vbus = STIM_PSU_VOLTAGE;
#endif
    float center = vbus / 2;
    if (center + v_max > (vbus * STIM_PWM_MAX_DUTY_CYCLE)) {
        center = (vbus * STIM_PWM_MAX_DUTY_CYCLE) - v_max;
    }

    // write pwm
    BSP_SetPWM4(
        (context[read_index].v1_cmd + center) / vbus,
        (context[read_index].v2_cmd + center) / vbus,
        (context[read_index].v3_cmd + center) / vbus,
        (context[read_index].v4_cmd + center) / vbus
    );

    // read currents
    if (i >= 2) {
        context[write_index].i1_meas = currents.a;
        context[write_index].i2_meas = currents.b;
        context[write_index].i3_meas = currents.c;
        context[write_index].i4_meas = currents.d;

        // log stats
        current_squared = Vec4f(
            current_squared.a + currents.a * currents.a,
            current_squared.b + currents.b * currents.b,
            current_squared.c + currents.c * currents.c,
            current_squared.d + currents.d * currents.d
        );
        current_max = Vec4f(
            max(current_max.a, abs(currents.a)),
            max(current_max.b, abs(currents.b)),
            max(current_max.c, abs(currents.c)),
            max(current_max.d, abs(currents.d))
        );
    }

    // check current limits
    if (abs(currents.a) > current_limit ||
        abs(currents.b) > current_limit ||
        abs(currents.c) > current_limit ||
        abs(currents.d) > current_limit)
    {
        BSP_DisableOutputs();
        current_limit_exceeded = true;
        interrupt_finished = true;
        return;
    }

    atomic_signal_fence(std::memory_order_release);
    interrupt_index = i + 1;
}

void FourphaseModel::perform_one_update_step()
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
    float err4 = context[i].i4_meas - context[i].i4_cmd;

    float dx1 = context[i_plus_one].i1_cmd - context[i_minus_one].i1_cmd;    // derivative
    float dx2 = context[i_plus_one].i2_cmd - context[i_minus_one].i2_cmd;
    float dx3 = context[i_plus_one].i3_cmd - context[i_minus_one].i3_cmd;
    float dx4 = context[i_plus_one].i4_cmd - context[i_minus_one].i4_cmd;

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
    if (context[i].i4_cmd < minimum_current) {
        magnitude_error4 += gamma1 * (context[i].i4_cmd * err4);
        angle_error4 += gamma2 * (dx4 * err4);
    }
}

#endif