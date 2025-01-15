#include "mrac_threephase_star.h"

#include <math.h>

#include <SimpleFOC.h>

#include "threephase_driver.h"
#include "emergency_stop.h"
#include "utils.h"
#include "pulse/threephase_pulse_buffer.h"


MRACThreephaseStar::MRACThreephaseStar()
{
}

void MRACThreephaseStar::init(ThreePhaseDriver *driver, CurrentSense *currentSense, EmergencyStop *emergencyStop)
{
    this->driver = driver;
    this->currentSense = currentSense;
    this->emergencyStop = emergencyStop;
}

void MRACThreephaseStar::play_pulse(ThreephasePulseBuffer *pulse, float estop_current_limit)
{
    this->estop_current_limit = estop_current_limit;

    constexpr float dt = 1.f/STIM_PWM_FREQ;
    float t = 0;
    float xHat_a = 0;
    float xHat_b = 0;

    // take hardware out of safe state
    driver->pwm_to_center();

    // init interrupt variables
    pwm_write_index = 0;
    interrupt_index = 0;
    model_update_index = 2; // model update requires context[t-2]
    skipped_update_steps = 0;
    buffer_underrun_detected = false;

    // attach the interrupt
    driver->attach_interrupt([this]{
        this->interrupt_fn();
    });

    // Run model, feed pwm values to the interrupt
    do {
        float desired_current_neutral, desired_current_left;
        pulse->get(t, &desired_current_neutral, &desired_current_left);

        // calculate r from: desired = xhat + dt * (A * xhat + B * r)
        float r_a = ((desired_current_neutral - xHat_a) / dt - A * xHat_a) * (1/B);
        float r_b = ((desired_current_left - xHat_b) / dt - A * xHat_b) * (1/B);

        // real hardware control signal = u = -Kx * xHat + Kr * r
        float u_ad = (+2 * Ka * xHat_a) - (1 * Kb * xHat_b) + (Kc * xHat_a + Kc * xHat_b);
        float u_bd = (-1 * Ka * xHat_a) + (2 * Kb * xHat_b) + (Kc * xHat_a + Kc * xHat_b);
        float u_a = Kr * r_a - u_ad;
        float u_b = Kr * r_b - u_bd;
        float u_c = -(u_a + u_b);

        float u_high = max(u_a, max(u_b, u_c));
        float u_low = min(u_a, min(u_b, u_c));

        // check voltage limits
        if ((u_high - u_low) > (STIM_PSU_VOLTAGE * STIM_PWM_MAX_DUTY_CYCLE))
        {
            float q = (STIM_PSU_VOLTAGE * STIM_PWM_MAX_DUTY_CYCLE) / (u_high - u_low);
            r_a *= q;
            u_a *= q;
            r_b *= q;
            u_b *= q;
            u_c *= q;
            u_high *= q;
            u_low *= q;
        }
        // log stats
        v_drive_max = max(v_drive_max, u_high - u_low);

        // update the reference model state; x = Ax + By
        xHat_a += dt * (A * xHat_a + B * r_a);
        xHat_b += dt * (A * xHat_b + B * r_b);

        // compute pwm center
        float center = STIM_PSU_VOLTAGE / 2;
        float offset = min(0.f, (STIM_PSU_VOLTAGE * STIM_PWM_MAX_DUTY_CYCLE) - (center + u_high));
        center = center + offset;

        // do not race too far ahead of the interrupt
        while (pwm_write_index > (interrupt_index + maximum_interrupt_lag) && !buffer_underrun_detected && !current_limit_exceeded)
        {
            // if we have some spare cpu cycles, perform model update step.
            perform_model_update_step();
        };

        // store values in buffer
        int index = pwm_write_index % context_size;
        context[index].pwm_voltage_neutral = center + u_a;
        context[index].pwm_voltage_left = center + u_b;
        context[index].pwm_voltage_right = center + u_c;
        context[index].xHat_a = xHat_a;
        context[index].xHat_b = xHat_b;
        context[index].r_a = r_a;
        context[index].r_b = r_b;
        pwm_write_index++;

        t += dt;
    } while (t < pulse->pulse_duration + 100e-6f && !buffer_underrun_detected && !current_limit_exceeded);

    if (buffer_underrun_detected && !current_limit_exceeded) {
        emergencyStop->trigger_emergency_stop();
        Serial.printf("buffer underrun\r\n");
        Serial.printf("interrupt index: %i\r\n", interrupt_index);
        Serial.printf("pwm write index: %i\r\n", pwm_write_index);
        Serial.printf("model update index: %i\r\n", model_update_index);
        Serial.printf("t: %f\r\n", t);
        Serial.printf("pulse length: %f\r\n", pulse->pulse_duration);
        while(1) {}
    }

    // wait until buffer underrun, indicating the pulse is finished.
    while (!buffer_underrun_detected && !current_limit_exceeded) {
        perform_model_update_step();
    }

    if (current_limit_exceeded) {
        emergencyStop->trigger_emergency_stop();
        while(1) {
            int adc_index = (interrupt_index - 3 + context_size) % context_size;
            Serial.printf("current limit exceeded: %f %f %f. Limit was %f. Restart device to proceed\r\n",
            context[adc_index].adc_current_neutral,
            context[adc_index].adc_current_left,
            context[adc_index].adc_current_right,
            estop_current_limit);
            delay(5000);
        }
    }

    // detach interrupt and configure hardware in safe state
    driver->detach_interrupt();
    driver->pwm_to_ground();
}

void MRACThreephaseStar::interrupt_fn()
{
    // wait until at least a few samples have been computed
    if (pwm_write_index < model_headstart) {
        return;
    }

    if (buffer_underrun_detected || current_limit_exceeded) {
        return;
    }

    // buffer underrun: pwm buffer not populated fast enough
    if (interrupt_index >= pwm_write_index) {
        driver->set_pwm_voltage(0, 0, 0);
        buffer_underrun_detected = true;
        return;
    }

    // write the new pwm values
    int pwm_index = interrupt_index % context_size;
    driver->set_pwm_voltage(
        context[pwm_index].pwm_voltage_neutral,
        context[pwm_index].pwm_voltage_left,
        context[pwm_index].pwm_voltage_right
    );

    // read adc values
    int adc_index = (interrupt_index - 2 + context_size) % context_size;    // ADC is lagged by 2 pwm cycles.
    ThreePhaseCurrents currents = driver->get_currents();
    context[adc_index].adc_current_neutral = currents.neutral;
    context[adc_index].adc_current_left = currents.left;
    context[adc_index].adc_current_right = currents.right;

    // log stats
    currents.normalize();
    current_squared_neutral += currents.neutral * currents.neutral * (1.f/STIM_PWM_FREQ);
    current_squared_left += currents.left * currents.left * (1.f/STIM_PWM_FREQ);
    current_squared_right += currents.right * currents.right * (1.f/STIM_PWM_FREQ);
    current_max_neutral = max(current_max_neutral, abs(currents.neutral));
    current_max_left = max(current_max_left, abs(currents.left));
    current_max_right = max(current_max_right, abs(currents.right));

    interrupt_index++;

    // check current limits
    if (abs(currents.neutral) > estop_current_limit ||
        abs(currents.left) > estop_current_limit ||
        abs(currents.right) > estop_current_limit)
    {
        driver->set_pwm_voltage(0, 0, 0);
        current_limit_exceeded = true;
        return;
    }
}


void MRACThreephaseStar::print_debug_stats()
{
    Serial.printf("\r\n");
    Serial.printf("\r\n");
    Serial.printf("\r\n");
    Serial.printf("L %f\r\n", estimate_inductance());
    Serial.printf("R_n %f\r\n", estimate_resistance_neutral());
    Serial.printf("R_l %f\r\n", estimate_resistance_left());
    Serial.printf("R_r %f\r\n", estimate_resistance_right());
    Serial.printf("index Va Vb Vc Ia Ib Ic xHat_a xHat_b xHat_c r_a r_b r_c\r\n");
    for (int i = 0; i < interrupt_index; i++) {
        float mid_voltage = (context[i].pwm_voltage_neutral + context[i].pwm_voltage_left + context[i].pwm_voltage_right) / 3;

        Serial.printf("%i ", i);
        Serial.printf("%f %f %f ", context[i].pwm_voltage_neutral - mid_voltage, context[i].pwm_voltage_left - mid_voltage, context[i].pwm_voltage_right - mid_voltage);
        Serial.printf("%f %f %f ", context[i].adc_current_neutral, context[i].adc_current_left, context[i].adc_current_right);
        Serial.printf("%f %f %f ", context[i].xHat_a, context[i].xHat_b, -(context[i].xHat_a + context[i].xHat_b));
        Serial.printf("%f %f %f ", context[i].r_a, context[i].r_b, -(context[i].r_a + context[i].r_b));
        Serial.printf("\r\n");
    }
}

void MRACThreephaseStar::perform_model_update_step()
{
    // need to wait until interrupt catches up.
    if ((model_update_index + 2) >= interrupt_index) {
        return;
    }

    // Model update too slow. Skip update to save a few cycles.
    if (model_update_index + maximum_update_lag < interrupt_index) {
        model_update_index++;
        skipped_update_steps++;
        return;
    }

    // perform model update
    int index = model_update_index % context_size;
    float x_a = context[index].adc_current_neutral;
    float x_b = context[index].adc_current_left;
    float x_c = context[index].adc_current_right;
    float midpoint = (x_a + x_b + x_c) / 3;
    x_a -= midpoint;
    x_b -= midpoint;
    float xHat_a = context[index].xHat_a;
    float xHat_b = context[index].xHat_b;
    float r_a = context[index].r_a;
    float r_b = context[index].r_b;

    float error_a = x_a - xHat_a;
    float error_b = x_b - xHat_b;

    const float dt = 1.f/STIM_PWM_FREQ;

    const float speed = 1.f;
    const float gamma1 = -.1f * (speed * P * B * dt);
    const float gamma2 = .05f * (speed * P * B * dt);

    // do Kx += dt * gamma * x * err^T * P * B;
    // code below does not follow the textbook equation, but seems to work best....
    Ka += gamma1 * (xHat_a * error_a);
    Kb += gamma1 * (xHat_b * error_b);
    Kc += gamma1 * ((xHat_a + xHat_b) * (error_a + error_b)); // = xHat_c * error_c

    // do kr += dt * gamma * r * err^T * P * B;
    Kr += gamma2 * (r_a * error_a);
    Kr += gamma2 * (r_b * error_b);
    Kr += gamma2 * (r_a + r_b) * (error_a + error_b); // = r_c * error_c

    Kr = _constrain(Kr, L_min / L0, L_max / L0);
    const float minimum = (R0 * Kr - R_max) / 3;
    const float maximum = (R0 * Kr - R_min) / 3;
    Ka = _constrain(Ka, minimum, maximum);
    Kb = _constrain(Kb, minimum, maximum);
    Kc = _constrain(Kc, minimum, maximum);

    model_update_index++;
}
