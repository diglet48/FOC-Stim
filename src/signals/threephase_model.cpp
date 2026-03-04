#include "threephase_model.h"

#include "bsp/bsp.h"
#include "foc_utils.h"
#include "utils.h"
#include "protobuf_api.h"

#include "stim_clock.h"

static float find_v_drive(Complex p1, Complex p2, Complex p3) {
    // TODO: exact method exists.
    int steps = 500;
    Complex proj(1, 0);
    Complex rotator(cosf(_2PI / float(steps)), sinf(_2PI / float(steps)));
    float v_drive = 0;
    for (int i = 0; i < steps; i++) {
        float a = (p1 * proj).real();
        float b = (p2 * proj).real();
        float c = (p3 * proj).real();
        float range = max({a, b, c}) - min({a, b, c});
        v_drive = max(range, v_drive);
        proj = proj * rotator;
    }

    return v_drive;
}

void ThreephaseModel::init(std::function<void(FOCError)> emergency_stop_fn) {
    this->emergency_stop_fn = emergency_stop_fn;

    phase_IQ_avg_1 = Complex(.1, 0);
    phase_IQ_avg_2 = Complex(.1, 0);
    phase_IQ_avg_3 = Complex(.1, 0);

    z1 = Complex(MODEL_RESISTANCE_INIT, 0); // impedance of first output
    z2 = Complex(MODEL_RESISTANCE_INIT, 0); // impedance of second output
    z3 = Complex(MODEL_RESISTANCE_INIT, 0); // impedance of third output
}

void ThreephaseModel::play_pulse(
    Complex p1, Complex p2, Complex p3,
    float carrier_frequency,
    float pulse_width, float rise_time,
    float estop_current_limit, float max_allowed_vdrive)
{
    if (std::abs(p1 + p2 + p3) > .001f) {
        BSP_PrintDebugMsg("Invalid pulse coordinates");
        return;
    }

    // reset variables for pulse playback
    producer_index = 0;
    interrupt_index = 0;
    updater_index = 0;
    interrupt_finished = false;
    skipped_update_steps = 0;
    current_limit_exceeded = 0;
    current_limit = estop_current_limit;

    cmd_IQ_1 = {};
    cmd_IQ_2 = {};
    cmd_IQ_3 = {};

    meas_IQ_1 = {};
    meas_IQ_2 = {};
    meas_IQ_3 = {};

    phase_IQ_1 = {};
    phase_IQ_2 = {};
    phase_IQ_3 = {};

    // reset pulse stats
    pulse_stats.current_squared = {0, 0, 0};
    pulse_stats.current_max = {0, 0, 0};
    pulse_stats.v_bus_min = 99;
    pulse_stats.v_bus_max = 0;
    pulse_stats.v_drive_requested = 0;
    pulse_stats.v_drive_actual = 0;

    // make debug easier by clearing out stale data.
    for (int i = 0; i < CONTEXT_SIZE; i++) {
        context[i] = {};
    }

    // reduce amplitude if this pulse would result in transformer saturation (volt*seconds)
    {
        // simplified equation, assumes impedance angle of low pass filter
        // is close to impedance angle of transformer. This generally is
        // the case for high-resistance electrodes.
        float v1 = (std::abs(z1) - MODEL_FIXED_RESISTANCE) * std::abs(p1);
        float v2 = (std::abs(z2) - MODEL_FIXED_RESISTANCE) * std::abs(p2);
        float v3 = (std::abs(z3) - MODEL_FIXED_RESISTANCE) * std::abs(p3);
        float max_v = std::max(v1, std::max(v2, v3));
        float volt_seconds = max_v / (2 * float(M_PI) * carrier_frequency);

        // reduce the pulse intensity if needed
        if (volt_seconds >= MODEL_MAXIMUM_VOLT_SECONDS) {
            float factor = MODEL_MAXIMUM_VOLT_SECONDS / volt_seconds;
            p1 *= factor;
            p2 *= factor;
            p3 *= factor;
        }

        pulse_stats.volt_seconds = volt_seconds;
        total_stats.volt_seconds = std::max(total_stats.volt_seconds, volt_seconds);
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
    pulse_stats.v_drive_requested = v_drive;
    if (v_drive > max_allowed_vdrive) {
        // if vdrive is too high, reduce current/voltage
        float factor = max_allowed_vdrive / v_drive;
        p1 = p1 * factor;
        p2 = p2 * factor;
        p3 = p3 * factor;
        N = N * factor;
        v1 = v1 * factor;
        v2 = v2 * factor;
        v3 = v3 * factor;
        v_drive = v_drive * factor;
    }
    pulse_stats.v_drive_actual = v_drive;

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

    // start producer, pulse plays in backgroud
    for (int i = 0; i < samples; i++) {
        if (i < rise_endpoint) {
            envelope = envelope * envelope_rotator;
        } else if (i >= fall_begin) {
            envelope = envelope * envelope_rotator;
        } else {
            envelope = Complex(1, 0);
        }

        Complex q = proj * envelope.real();
        proj = proj * rotator;
        context[i % CONTEXT_SIZE].cosine = proj.real();
        context[i % CONTEXT_SIZE].sine = proj.imag();
        context[i % CONTEXT_SIZE].i1_cmd = (p1 * q).real();  // desired current, used for update step.
        context[i % CONTEXT_SIZE].i2_cmd = (p2 * q).real();
        context[i % CONTEXT_SIZE].i3_cmd = (p3 * q).real();
        context[i % CONTEXT_SIZE].v1_cmd = (v1 * q).real();  // cmd voltage.
        context[i % CONTEXT_SIZE].v2_cmd = (v2 * q).real();
        context[i % CONTEXT_SIZE].v3_cmd = (v3 * q).real();
        context[i % CONTEXT_SIZE].v1_cmd_quadrature = (v1 * q).imag();  // cmd voltage shifted 90 deg.
        context[i % CONTEXT_SIZE].v2_cmd_quadrature = (v2 * q).imag();
        context[i % CONTEXT_SIZE].v3_cmd_quadrature = (v3 * q).imag();

        atomic_signal_fence(std::memory_order_release);
        producer_index = i;

        // perform update steps when queue has plenty of items.
        while ((i - interrupt_index) >= max_producer_queue_length && !interrupt_finished) {
            accumulate_errors();
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
        accumulate_errors();
    }

    // stop pwm, all phases to ground.
    BSP_AttachPWMInterrupt(nullptr);
    BSP_SetPWM3Atomic(0, 0, 0);

    if (current_limit_exceeded) {
        BSP_DisableOutputs();
        BSP_PrintDebugMsg("Current limit exceeded");
        int i = (interrupt_index - 2) % CONTEXT_SIZE;
        BSP_PrintDebugMsg("currents were: %f %f %f", context[i].i1_meas, context[i].i2_meas, context[i].i3_meas);
        emergency_stop_fn(FOCError::OUTPUT_OVER_CURRENT);
        while(1) {}
    }

    if (interrupt_index != producer_index) {
        BSP_DisableOutputs();
        BSP_PrintDebugMsg("Producer too slow %i %i", (int)interrupt_index, (int)producer_index);
        emergency_stop_fn(FOCError::MODEL_TIMING_ERROR);
        while(1) {}
    }

    // process all remaining update steps.
    while (updater_index != (interrupt_index - 2)) {
        accumulate_errors();
    }

    model_update(p1, p2, p3);

    // update stats
    total_stats.current_max = {
        std::max(total_stats.current_max.a, + pulse_stats.current_max.a),
        std::max(total_stats.current_max.b, + pulse_stats.current_max.b),
        std::max(total_stats.current_max.c, + pulse_stats.current_max.c),
    };
    total_stats.current_squared = total_stats.current_squared + pulse_stats.current_squared;
}

Vec3f ThreephaseModel::estimate_rms_current(float dt)
{
    dt = dt * STIM_PWM_FREQ;
#if defined(CURRENT_SENSE_SCALE_FULL)
    return Vec3f(
        sqrtf(total_stats.current_squared.a / dt),
        sqrtf(total_stats.current_squared.b / dt),
        sqrtf(total_stats.current_squared.c / dt)
    );
#elif defined(CURRENT_SENSE_SCALE_HALF)
    return Vec3f(
        sqrtf(total_stats.current_squared.a / dt) * _SQRT2,
        sqrtf(total_stats.current_squared.b / dt) * _SQRT2,
        sqrtf(total_stats.current_squared.c / dt) * _SQRT2
    );
#else
#error unknown current sense method
#endif
}

void ThreephaseModel::debug_stats_teleplot()
{
    BSP_PrintDebugMsg("    i     V1     V2     V3 i1_cmd i2_cmd i3_cmd     i1     i2     i3");
    int start_index = max(0, producer_index - CONTEXT_SIZE + 1);
    for (int i = start_index; i <= producer_index; i++) {
        const auto &c = context[i % CONTEXT_SIZE];

        BSP_PrintDebugMsg(
            "%5i %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f",
            i,
            c.v1_cmd, c.v2_cmd, c.v3_cmd,
            c.i1_cmd, c.i2_cmd, c.i3_cmd,
            c.i1_meas, c.i2_meas, c.i3_meas
        );
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

#ifdef STIM_DYNAMIC_VOLTAGE
    float vbus = BSP_ReadVBus();
    pulse_stats.v_bus_max = max(pulse_stats.v_bus_max, vbus);
    pulse_stats.v_bus_min = min(pulse_stats.v_bus_min, vbus);
#else
    float vbus = STIM_PSU_VOLTAGE;
#endif

#if defined(DEADTIME_COMPENSATION_ENABLE)
    auto dtcomp = [&](float voltage, float current) {
        float comp_percent = 0;
        if (current >= DEADTIME_COMPENSATION_CURRENT_THRESHOLD) {
            comp_percent = DEADTIME_COMPENSATION_PERCENTAGE;
        }
        else if (current <= -DEADTIME_COMPENSATION_CURRENT_THRESHOLD) {
            comp_percent = -DEADTIME_COMPENSATION_PERCENTAGE;
        } else {
            comp_percent = DEADTIME_COMPENSATION_PERCENTAGE * current / DEADTIME_COMPENSATION_CURRENT_THRESHOLD;
        }
        return voltage + comp_percent * vbus;
    };
#else
    auto dtcomp = [&](float voltage, float current) {
        return voltage;
    };
#endif
    float v1 = dtcomp(context[read_index].v1_cmd, context[read_index].i1_cmd);
    float v2 = dtcomp(context[read_index].v2_cmd, context[read_index].i2_cmd);
    float v3 = dtcomp(context[read_index].v3_cmd, context[read_index].i3_cmd);

    // compute duty cycle center
    float v_min = min({v1, v2, v3});
    float v_max = max({v1, v2, v3});

    float center = vbus / 2;
    if (center + v_max > (vbus * STIM_PWM_MAX_DUTY_CYCLE)) {
        center = (vbus * STIM_PWM_MAX_DUTY_CYCLE) - v_max;
    }

    // write pwm
    BSP_SetPWM3(
        (v1 + center) / vbus,
        (v2 + center) / vbus,
        (v3 + center) / vbus
    );

    // read currents
    if (i >= 2) {
        context[write_index].i1_meas = currents.a;
        context[write_index].i2_meas = currents.b;
        context[write_index].i3_meas = currents.c;

        // log stats
        pulse_stats.current_squared = Vec3f(
            pulse_stats.current_squared.a + currents.a * currents.a,
            pulse_stats.current_squared.b + currents.b * currents.b,
            pulse_stats.current_squared.c + currents.c * currents.c
        );
        pulse_stats.current_max = Vec3f(
            max(pulse_stats.current_max.a, abs(currents.a)),
            max(pulse_stats.current_max.b, abs(currents.b)),
            max(pulse_stats.current_max.c, abs(currents.c))
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

void ThreephaseModel::accumulate_errors()
{
    int samples_available = (interrupt_index - 2) - updater_index;
    if (samples_available <= 0) {
        return;
    }
    if (samples_available >= max_updater_lag) {
        skipped_update_steps++;
        updater_index++;
        return;
    }
    int i = updater_index++;

    if (i == 0) {
        return;
    }

    atomic_signal_fence(std::memory_order_acquire);
    i = i % CONTEXT_SIZE;

#if defined(CURRENT_SENSE_SCALE_FULL)
    const float minimum_current = infinityf();
#elif defined(CURRENT_SENSE_SCALE_HALF)
    const float minimum_current = -0.02f;
#else
#error unknown current sense method
#endif

    // use I/Q sampling to find the commanded current and measured current amplitude.
    // skip on devices without bidirectional current sense if commanded current is low,
    // because of poor accuracy.
    if (context[i].i1_cmd < minimum_current) {
        meas_IQ_1 += Complex(context[i].cosine, context[i].sine) * context[i].i1_meas;
        cmd_IQ_1 += Complex(context[i].cosine, context[i].sine) * context[i].i1_cmd;
    }
    if (context[i].i2_cmd < minimum_current) {
        meas_IQ_2 += Complex(context[i].cosine, context[i].sine) * context[i].i2_meas;
        cmd_IQ_2 += Complex(context[i].cosine, context[i].sine) * context[i].i2_cmd;
    }
    if (context[i].i3_cmd < minimum_current) {
        meas_IQ_3 += Complex(context[i].cosine, context[i].sine) * context[i].i3_meas;
        cmd_IQ_3 += Complex(context[i].cosine, context[i].sine) * context[i].i3_cmd;
    }

    // use I/Q sampling to find the phase offset between voltage and current
    phase_IQ_1 += Complex(context[i].v1_cmd, context[i].v1_cmd_quadrature) * context[i].i1_meas;
    phase_IQ_2 += Complex(context[i].v2_cmd, context[i].v2_cmd_quadrature) * context[i].i2_meas;
    phase_IQ_3 += Complex(context[i].v3_cmd, context[i].v3_cmd_quadrature) * context[i].i3_meas;
}

void ThreephaseModel::model_update(Complex p1, Complex p2, Complex p3)
{
// update impedance magnitude
    {
        float multiplier = 1.f / pulse_length_samples * 2;
#if defined(CURRENT_SENSE_SCALE_HALF)
        multiplier *= 2;
#endif

        // calculate measured/commanded current, unit amperes.
        // It is 10-40% below the real value because of rise time.
        float meas_1 = std::abs(meas_IQ_1) * multiplier;
        float meas_2 = std::abs(meas_IQ_2) * multiplier;
        float meas_3 = std::abs(meas_IQ_3) * multiplier;
        float cmd_1 = std::abs(cmd_IQ_1) * multiplier;
        float cmd_2 = std::abs(cmd_IQ_2) * multiplier;
        float cmd_3 = std::abs(cmd_IQ_3) * multiplier;

        // integrate measurement and command
        float decay = .002f;
        integrated_meas = integrated_meas * (1 - decay) + Vec3f(meas_1, meas_2, meas_3) * decay;
        integrated_cmd = integrated_cmd * (1 - decay) + Vec3f(cmd_1, cmd_2, cmd_3) * decay;

        // select gradient descent step size based on the average error
        float error_ratio = (integrated_cmd - integrated_meas).abs_sum() / std::max(integrated_cmd.sum(), 0.05f);
        float step_size = interpolate(error_ratio, 0.01f, 0.3f, .1f, 1.0f); // 1% error = 0.1 step. 30% error = 1.0 step

        // gradient descent update step. Change impedance magnitude only
        z1 = Complex(std::abs(z1) - step_size * std::abs(p1 * z1) * (meas_1 - cmd_1), 0);
        z2 = Complex(std::abs(z2) - step_size * std::abs(p2 * z2) * (meas_2 - cmd_2), 0);
        z3 = Complex(std::abs(z3) - step_size * std::abs(p3 * z3) * (meas_3 - cmd_3), 0);

        // // debug SLOW
        // static int i = 0;
        // if (++i >= 5) {
        //     i = 0;
        //     g_protobuf->transmit_notification_debug_teleplot("meas 1", meas_1);
        //     g_protobuf->transmit_notification_debug_teleplot("cmd 1", cmd_1);
        //     g_protobuf->transmit_notification_debug_teleplot("meas 2", meas_2);
        //     g_protobuf->transmit_notification_debug_teleplot("cmd 2", cmd_2);
        //     g_protobuf->transmit_notification_debug_teleplot("meas 3", meas_3);
        //     g_protobuf->transmit_notification_debug_teleplot("cmd 3", cmd_3);
        //     g_protobuf->transmit_notification_debug_teleplot("err ratio", error_ratio);
        //     g_protobuf->transmit_notification_debug_teleplot("update rate", step_size);
        // }
    }

    // update impedance phase angle
    {
        // skip update < 20mA (about 2% power)
        // because sensor gives poor quality data
        float minimum_current_for_update = .02f;

        // normalize, so unit is roughtly in watt (voltage * current)
        phase_IQ_1 /= pulse_length_samples;
        phase_IQ_2 /= pulse_length_samples;
        phase_IQ_3 /= pulse_length_samples;

        // slowly converge to new parameters
        float learning_rate = .01f;
        if (std::abs(p1) > minimum_current_for_update) {
            phase_IQ_avg_1 = phase_IQ_avg_1 * (1 - learning_rate) + phase_IQ_1 * learning_rate;
        }
        if (std::abs(p2) > minimum_current_for_update) {
            phase_IQ_avg_2 = phase_IQ_avg_2 * (1 - learning_rate) + phase_IQ_2 * learning_rate;
        }
        if (std::abs(p3) > minimum_current_for_update) {
            phase_IQ_avg_3 = phase_IQ_avg_3 * (1 - learning_rate) + phase_IQ_3 * learning_rate;
        }

        // clamp to avoid problems near zero.
        float minimum_magnitude = 0.01f; // ~watt, meaning really low power.
        if (std::abs(phase_IQ_avg_1) <= minimum_magnitude) {
            phase_IQ_avg_1 *= minimum_magnitude / std::abs(phase_IQ_avg_1);
        }
        if (std::abs(phase_IQ_avg_2) <= minimum_magnitude) {
            phase_IQ_avg_2 *= minimum_magnitude / std::abs(phase_IQ_avg_2);
        }
        if (std::abs(phase_IQ_avg_3) <= minimum_magnitude) {
            phase_IQ_avg_3 *= minimum_magnitude / std::abs(phase_IQ_avg_3);
        }

        // overwrite impedance with new phase angle
        z1 = std::polar(std::abs(z1), std::arg(phase_IQ_avg_1));
        z2 = std::polar(std::abs(z2), std::arg(phase_IQ_avg_2));
        z3 = std::polar(std::abs(z3), std::arg(phase_IQ_avg_3));
    }

    // constrain impedance magnitude/angle
    z1 = constrain_in_bound(z1, MODEL_RESISTANCE_MIN, MODEL_RESISTANCE_MAX, MODEL_PHASE_ANGLE_MIN, MODEL_PHASE_ANGLE_MAX);
    z2 = constrain_in_bound(z2, MODEL_RESISTANCE_MIN, MODEL_RESISTANCE_MAX, MODEL_PHASE_ANGLE_MIN, MODEL_PHASE_ANGLE_MAX);
    z3 = constrain_in_bound(z3, MODEL_RESISTANCE_MIN, MODEL_RESISTANCE_MAX, MODEL_PHASE_ANGLE_MIN, MODEL_PHASE_ANGLE_MAX);
}
