#include <Arduino.h>
#include <SimpleFOC.h>

#include "tcode.h"
#include "utils.h"
#include "stim_clock.h"
#include "pulse_single.h"
#include "pulse_threephase.h"
#include "mrac_1d.h"
#include "mrac_threephase_star.h"
// #include "mrac_threephase_star_assym.h"
#include "config.h"
#include "emergency_stop.h"
#include "trace.h"

BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
// Gain calculation shown at https://community.simplefoc.com/t/b-g431b-esc1-current-control/521/21
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

MRACThreephaseStar mrac2{};
ThreephasePulse pulse_threephase{};
Trace trace{};

EmergencyStop emergencyStop{};

struct
{
    TCodeAxis alpha{"L0", 0, -1, 1};
    TCodeAxis beta{"L1", 0, -1, 1};
    TCodeAxis volume{"V0", 0, 0, TCODE_MAX_CURRENT};
    TCodeAxis carrier_frequency{"A0", 800, 500, 1000};
    TCodeAxis pulse_frequency{"A1", 50, 1, 100};
    TCodeAxis pulse_width{"A2", 6, 3, 20};
    TCodeAxis calib_center{"C0", 0, -10, 10};
    TCodeAxis calib_ud{"C1", 0, -10, 10};
    TCodeAxis calib_lr{"C2", 0, -10, 10};
    TCodeAxis boot_dummy{"B9", 0, 0, 1};
} axes;
// TODO: special commands?
TCode tcode(reinterpret_cast<TCodeAxis *>(&axes), sizeof(axes) / sizeof(TCodeAxis));

void estop_triggered()
{
    mrac2.print_debug_stats();
    trace.print_mainloop_trace();
}

void setup()
{
    // use monitoring with serial
    Serial.begin(115200);
    Serial.println("- begin setup");
    // enable more verbose output for debugging
    // comment out if not needed
    // SimpleFOCDebug::enable(&Serial);

    Serial.println("- init driver");
    driver.voltage_power_supply = STIM_PSU_VOLTAGE;
    driver.pwm_frequency = STIM_PWM_FREQ;
    driver.init();

    Serial.printf("- init emergency stop\r\n");
    emergencyStop.init(&driver, &currentSense, &estop_triggered);

    Serial.println("- init current sense");
    currentSense.linkDriver(&driver);
    int current_sense_err = currentSense.init();
    if (current_sense_err != 1)
    {
        Serial.println("- init current sense failed!");
        emergencyStop.trigger_emergency_stop();
        while (1)
        {
        }
    }
    driver.setPwm(driver.voltage_power_supply / 2, driver.voltage_power_supply / 2, driver.voltage_power_supply / 2);
    driver.enable();

    Serial.printf("- init mrac\r\n");
    // mrac.init(&driver, &currentSense);
    mrac2.init(&driver, &currentSense, &emergencyStop);

    _delay(100);
    while (1)
    {
        float vbus = read_vbus(&currentSense);
        if (vbus > STIM_PSU_VOLTAGE_MAX || vbus < STIM_PSU_VOLTAGE_MIN)
        {
            Serial.printf("V_BUS:%f waiting for V_BUS to come online...\r\n", vbus);
            Clock clock;
            while (clock.time_seconds < 1)
            {
                tcode.update_from_serial();
                clock.step();
            }
        }
        else
        {
            Serial.printf("V_BUS:%f\r\n", vbus);
            break;
        }
    }
    while (1)
    {
        bool restim_detected = axes.boot_dummy.get_remap(millis()) > 0.5f;
        if (!restim_detected)
        {
            Serial.printf("waiting for restim...\r\n");
            Clock clock;
            while (clock.time_seconds < 1)
            {
                tcode.update_from_serial();
                clock.step();
            }
        }
        else
        {
            break;
        }
    }

    Serial.printf("loop start\r\n");
}

void loop()
{
    static uint32_t loop_counter = 0;
    static float actual_pulse_frequency = 0;
    static uint32_t last_loop_start_time = micros();

    MainLoopTraceLine *traceline = trace.next_main_loop_line();
    traceline->t_start = micros();

    // calculate stats
    loop_counter++;
    uint32_t loop_start_time = micros();
    actual_pulse_frequency = lerp(.05f, actual_pulse_frequency, 1 / (float(loop_start_time - last_loop_start_time) * 1e-6f));
    last_loop_start_time = loop_start_time;

    // grab the new pulse parameters from serial comms
    tcode.update_from_serial();

    float pulse_alpha = axes.alpha.get_remap(loop_start_time);
    float pulse_beta = axes.beta.get_remap(loop_start_time);
    float pulse_amplitude = axes.volume.get_remap(loop_start_time); // pulse amplitude in amps
    float pulse_carrier_frequency = axes.carrier_frequency.get_remap(loop_start_time);
    float pulse_frequency = axes.pulse_frequency.get_remap(loop_start_time);
    float pulse_width = axes.pulse_width.get_remap(loop_start_time);

    float calibration_center = axes.calib_center.get_remap(loop_start_time);
    float calibration_lr = axes.calib_lr.get_remap(loop_start_time);
    float calibration_ud = axes.calib_ud.get_remap(loop_start_time);

    float pulse_active_duration = pulse_width / pulse_carrier_frequency;
    float pulse_pause_duration = max(0.f, 1 / pulse_frequency - pulse_active_duration);
    float pulse_total_duration = pulse_active_duration + pulse_pause_duration;

    traceline->pulse_active_duration = pulse_active_duration;
    traceline->pause_duration = pulse_pause_duration;
    traceline->amplitude = pulse_amplitude;
    emergencyStop.check_vbus();

    // pre-compute the new pulse
    traceline->t_compute_start = micros();
    Clock pause_timer{};
    pulse_threephase.create_pulse(
        pulse_amplitude,
        pulse_alpha,
        pulse_beta,
        pulse_carrier_frequency,
        pulse_width,
        calibration_center,
        calibration_ud,
        calibration_lr);
    pause_timer.step();

    // reset stats
    mrac2.neutral_abs_sum = 0;
    mrac2.left_abs_sum = 0;
    mrac2.right_abs_sum = 0;
    mrac2.neutral_sum = 0;
    mrac2.left_sum = 0;
    mrac2.right_sum = 0;

    static float neutral_abs_sum = 0;
    static float left_abs_sum = 0;
    static float right_abs_sum = 0;
    static float neutral_dc = 0;
    static float left_dc = 0;
    static float right_dc = 0;

    // play the pulse
    traceline->t_pulse_play_start = micros();
    Clock timer{};
    uint32_t iterations_per_pulse = 0;
    while (timer.time_seconds < (pulse_active_duration + 200e-6f))
    {
        traceline->mrac_iters++;
        iterations_per_pulse++;
        timer.step();
        float desired_current_neutral = 0;
        float desired_current_change_neutral = 0;
        float desired_current_right = 0;
        float desired_current_change_right = 0;
        pulse_threephase.get(timer.time_seconds,
                             &desired_current_neutral, &desired_current_right,
                             &desired_current_change_neutral, &desired_current_change_right);
        mrac2.iter(desired_current_neutral, desired_current_change_neutral,
                   desired_current_right, desired_current_change_right);
    }
    timer.step();

    // update stats
    traceline->t_logs = micros();
    traceline->xhat_a1 = mrac2.xHat_a;
    traceline->xhat_b1 = mrac2.xHat_b;
    traceline->pia = mrac2.pid_a_i;
    traceline->pib = mrac2.pid_b_i;
    mrac2.prepare_for_idle();

    // update stats
    neutral_abs_sum = lerp(0.1f, neutral_abs_sum, mrac2.neutral_abs_sum);
    left_abs_sum = lerp(0.1f, left_abs_sum, mrac2.left_abs_sum);
    right_abs_sum = lerp(0.1f, right_abs_sum, mrac2.right_abs_sum);
    neutral_dc = lerp(0.1f, neutral_dc, mrac2.neutral_sum / iterations_per_pulse);
    left_dc = lerp(0.1f, left_dc, mrac2.left_sum / iterations_per_pulse);
    right_dc = lerp(0.1f, right_dc, mrac2.right_sum / iterations_per_pulse);

    // occasionally print some stats..
    if ((loop_counter + 0) % 10 == 0)
    {
        Serial.print("$");
        Serial.printf("R_neutral:%.2f ", mrac2.estimate_r1());
        Serial.printf("R_left:%.2f ", mrac2.estimate_r2());
        Serial.printf("R_right:%.2f ", mrac2.estimate_r3());
        Serial.println();
    }
    if ((loop_counter + 2) % 10 == 0)
    {
        Serial.print("$");
        Serial.printf("L:%.2f ", mrac2.estimate_inductance() * 1e6f);
        Serial.printf("V_drive:%.2f ", mrac2.v_drive_max);
        Serial.printf("I_max_a:%f ", abs(emergencyStop.max_recorded_current_a));
        Serial.printf("I_max_b:%f ", abs(emergencyStop.max_recorded_current_b));
        Serial.printf("I_max_c:%f ", abs(emergencyStop.max_recorded_current_c));
        Serial.println();
        emergencyStop.max_recorded_current_a = 0;
        emergencyStop.max_recorded_current_b = 0;
        emergencyStop.max_recorded_current_c = 0;

    }
    if ((loop_counter + 4) % 10 == 0)
    {
        Serial.print("$");
        Serial.printf("I_abs_neutral:%f ", neutral_abs_sum);
        Serial.printf("I_abs_left:%f ", left_abs_sum);
        Serial.printf("I_abs_right:%f ", right_abs_sum);
        Serial.println();
    }
    if ((loop_counter + 6) % 10 == 0)
    {
        Serial.print("$");
        Serial.printf("DC_neutral:%f ", neutral_dc);
        Serial.printf("DC_left:%f ", left_dc);
        Serial.printf("DC_right:%f ", right_dc);
        Serial.println();
    }
    if ((loop_counter + 8) % 20 == 0)
    {
        Serial.print("$");
        Serial.printf("V_BUS:%.2f ", read_vbus(&currentSense));
        Serial.printf("temp:%.1f ", read_temperature(&currentSense));
        Serial.printf("F_mrac:%.0f ", iterations_per_pulse / timer.time_seconds);
        Serial.printf("F_pulse:%.1f ", actual_pulse_frequency);
        Serial.println();
    }

    traceline->t_idle = micros();
    pause_timer.step();
    // stall out the pulse pause.
    while (pause_timer.time_seconds < pulse_total_duration)
    {
        pause_timer.step();
        mrac2.iter(0, 0, 0, 0);
        emergencyStop.check_current_limits();
    }
    traceline->t_end = micros();
    traceline->xhat_a2 = mrac2.xHat_a;
    traceline->xhat_b2 = mrac2.xHat_b;
    mrac2.prepare_for_idle();
    traceline->v_drive_max = mrac2.v_drive_max;
    mrac2.v_drive_max = 0;
    traceline->max_recorded_current_a = emergencyStop.max_recorded_current_a;
    traceline->max_recorded_current_b = emergencyStop.max_recorded_current_b;
    traceline->max_recorded_current_c = emergencyStop.max_recorded_current_c;
    traceline->Ra = mrac2.estimate_r1();
    traceline->Rb = mrac2.estimate_r2();
    traceline->Rc = mrac2.estimate_r3();
    traceline->L = mrac2.estimate_inductance();
}