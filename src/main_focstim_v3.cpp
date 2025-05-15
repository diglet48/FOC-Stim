#if defined(ARDUINO_FOCSTIM_V3)

#include <Arduino.h>

#include "foc_utils.h"
#include "tcode.h"
#include "utils.h"
#include "stim_clock.h"
#include "trace.h"
#include "complex.h"
#include "signals/threephase_math.h"
#include "signals/threephase_model.h"
#include "signals/fourphase_math.h"
#include "signals/fourphase_model.h"
#include "battery/SparkFunBQ27441.h"
#include "battery/power_manager.h"
#include <Wire.h>

#include "bsp/bsp.h"


Trace trace{};

ThreephaseModel model3{};
FourphaseModel model4{};

PowerManager power_manager{};


static bool status_booted = false;
static bool status_estop = false;

enum PlayStatus{
    NotPlaying,
    PlayingThreephase,
    PlayingFourphase
};
static PlayStatus play_status = PlayStatus::NotPlaying;

static Clock keepalive_clock{};


void print_status() {
    uint32_t status =
        (status_booted << 0)
        | (1 << 1)      // vbus
        | (status_estop << 2)
        | ((play_status != PlayStatus::NotPlaying) << 3);
    Serial.printf("status: %lu\r\n", status);
}


void tcode_D0() {
    Serial.println();
    Serial.printf("version: FOC-Stim 0.5\r\n");
    print_status();
}

void tcode_DSTART3() {
    if (play_status == PlayStatus::NotPlaying) {
        Serial.printf("Pulse generation started (threephase)\r\n");
        BSP_OutputEnable(true, true, true);
        play_status = PlayStatus::PlayingThreephase;
    }
}

void tcode_DSTART4() {
    if (play_status == PlayStatus::NotPlaying) {
        Serial.printf("Pulse generation started (fourphase)\r\n");
        BSP_OutputEnable(true, true, true, true);
        play_status = PlayStatus::PlayingFourphase;
    }
}

void tcode_DSTOP() {
    if (play_status != PlayStatus::NotPlaying) {
        Serial.printf("Pulse generation stopped\r\n");
        BSP_DisableOutputs();
        play_status = PlayStatus::NotPlaying;
        BSP_WriteLedPattern(LedPattern::Idle);
    }
}

void tcode_DPING() {
    keepalive_clock.reset();
}

struct
{
    TCodeAxis alpha{"L0", 0, -1, 1};
    TCodeAxis beta{"L1", 0, -1, 1};
    TCodeAxis gamma{"L2", 0, -1, 1};
    TCodeAxis volume{"V0", 0, 0, BODY_CURRENT_MAX * STIM_WINDING_RATIO};
    TCodeAxis carrier_frequency{"A0", 800, 500, 2000};
    TCodeAxis pulse_frequency{"A1", 50, 1, 100};
    TCodeAxis pulse_width{"A2", 6, 3, 20};
    TCodeAxis pulse_rise{"A3", 5, 2, 10};
    TCodeAxis pulse_interval_random{"A4", 0, 0, 1};
    TCodeAxis calib_center{"C0", 0, -10, 10};
    TCodeAxis calib_ud{"C1", 0, -10, 10};
    TCodeAxis calib_lr{"C2", 0, -10, 10};
    TCodeAxis calib_4a{"C3", 0, -10, 10};
    TCodeAxis calib_4b{"C4", 0, -10, 10};
    TCodeAxis calib_4c{"C5", 0, -10, 10};
    TCodeAxis calib_4d{"C6", 0, -10, 10};
} axes;
struct {
    TCodeDeviceCommand d0{"0", &tcode_D0};
    TCodeDeviceCommand d_start{"START", &tcode_DSTART3};    // todo: remove at next version bump
    TCodeDeviceCommand d_start3{"START3", &tcode_DSTART3};
    TCodeDeviceCommand d_start4{"START4", &tcode_DSTART4};
    TCodeDeviceCommand d_stop{"STOP", &tcode_DSTOP};
    TCodeDeviceCommand d_ping{"PING", &tcode_DPING};
} tcode_device_commands;
TCode tcode(reinterpret_cast<TCodeAxis *>(&axes), sizeof(axes) / sizeof(TCodeAxis),
            reinterpret_cast<TCodeDeviceCommand *>(&tcode_device_commands), sizeof(tcode_device_commands) / sizeof(TCodeDeviceCommand));

void estop_triggered()
{
    BSP_WriteLedPattern(LedPattern::Error);
    trace.print_mainloop_trace();
    if (play_status == PlayStatus::PlayingThreephase) {
        model3.debug_stats_teleplot();
    }
    if (play_status == PlayStatus::PlayingFourphase) {
        model4.debug_stats_teleplot();
    }
}

void setup()
{
    Serial.begin(115200, SERIAL_8E1);
    delay(100);
    Serial.println("BSP init");
    BSP_Init();
    BSP_WriteLedPattern(LedPattern::Idle);

    // print status to let the computer know we have rebooted.
    print_status();

    // power_manager.init();
    // power_manager.adjust_board_voltages();
    // power_manager.print_battery_stats();

    BSP_SetBoostMinimumInputVoltage(4.0);
    BSP_SetBoostVoltage(STIM_BOOST_VOLTAGE);
    BSP_SetBoostEnable(true);
    // wait to fill the boost caps
    while (BSP_ReadVBus() < STIM_BOOST_VOLTAGE_OK_THRESHOLD) {
        delay(10);
    }

    model3.init(&estop_triggered);
    model4.init(&estop_triggered);
    print_status();
}

void loop()
{
    static Clock total_pulse_length_timer;
    static float pulse_total_duration = 0;
    static Clock actual_pulse_frequency_clock;
    static Clock rms_current_clock;
    static Clock status_print_clock;
    static Clock vbus_print_clock;
    static Clock vbus_high_clock;
    static uint32_t pulse_counter = 0;
    static float actual_pulse_frequency = 0;

    // do comms
    bool dirty = tcode.update_from_serial();

    // safety checks
    float temperature = BSP_ReadTemperatureSTM();
    if (temperature >= MAXIMUM_TEMPERATURE) {
        BSP_DisableOutputs();
        while (1)
        {
            BSP_WriteLedPattern(LedPattern::Error);
            Serial.printf("temperature limit exceeded %.2f. Current temperature=%.2f. Restart device to proceed\r\n",
                          temperature, BSP_ReadTemperatureSTM());
            delay(5000);
        }
    }

    float vbus = BSP_ReadVBus();
    // overvoltage threshold
    if (vbus >= STIM_BOOST_OVERVOLTAGE_THRESHOLD) {
        BSP_DisableOutputs();
        while (1)
        {
            BSP_WriteLedPattern(LedPattern::Error);
            Serial.printf("boost overvoltage detected %.2f. Current boost=%.2f. Restart device to proceed\r\n",
                          vbus, BSP_ReadVBus());
            delay(5000);
        }
    }

    // undervoltage threshold
    if (vbus <= STIM_BOOST_UNDERVOLTAGE_THRESHOLD) {
        BSP_DisableOutputs();
        while (1)
        {
            BSP_WriteLedPattern(LedPattern::Error);
            Serial.printf("boost undervoltage detected %.2f. Current boost=%.2f. Restart device to proceed\r\n",
                          vbus, BSP_ReadVBus());
            delay(5000);
        }
    }

    // keepalive timer. Stop playing if no messages have been received for some time.
    if (dirty) {
        keepalive_clock.reset();
    }
    keepalive_clock.step();
    if (keepalive_clock.time_seconds > 2 && play_status != PlayStatus::NotPlaying) {
        Serial.println("Connection lost? Stopping pulse generation.");
        play_status = PlayStatus::NotPlaying;
        BSP_WriteLedPattern(LedPattern::Error);
        BSP_DisableOutputs();
        print_status();
    }

    // DSTART / DSTOP
    if (play_status == PlayStatus::NotPlaying) {
        delay(10);
        return;
    }

    // stall out idle portion of the pulse
    total_pulse_length_timer.step();
    if (total_pulse_length_timer.time_seconds <= pulse_total_duration) {
        return;
    }

    // delay pulse until the boost is completely filled back up, reducing the pulse frequency
    if (vbus <= STIM_BOOST_VOLTAGE_OK_THRESHOLD) {
        return;
    }

    // ready to generate next pulse!
    MainLoopTraceLine *traceline = trace.next_main_loop_line();
    total_pulse_length_timer.reset();
    traceline->t_start = total_pulse_length_timer.last_update_time;

    // calculate stats
    pulse_counter++;
    actual_pulse_frequency_clock.step();
    actual_pulse_frequency = lerp(.05f, actual_pulse_frequency, 1e6f / actual_pulse_frequency_clock.dt_micros);

    // get all the pulse parameters
    uint32_t t0 = micros();
    float pulse_alpha = axes.alpha.get_remap(t0);
    float pulse_beta = axes.beta.get_remap(t0);
    float pulse_gamma = axes.gamma.get_remap(t0);

    float pulse_amplitude = axes.volume.get_remap(t0); // pulse amplitude in amps
    float pulse_carrier_frequency = axes.carrier_frequency.get_remap(t0);
    float pulse_frequency = axes.pulse_frequency.get_remap(t0);
    float pulse_width = axes.pulse_width.get_remap(t0);
    float pulse_rise = axes.pulse_rise.get_remap(t0);
    float pulse_interval_random = axes.pulse_interval_random.get_remap(t0);

    float calibration_center = axes.calib_center.get_remap(t0);
    float calibration_lr = axes.calib_lr.get_remap(t0);
    float calibration_ud = axes.calib_ud.get_remap(t0);

    float calibration_4a = axes.calib_4a.get_remap(t0);
    float calibration_4b = axes.calib_4b.get_remap(t0);
    float calibration_4c = axes.calib_4c.get_remap(t0);
    float calibration_4d = axes.calib_4d.get_remap(t0);

    float pulse_active_duration = pulse_width / pulse_carrier_frequency;
    float pulse_pause_duration = max(0.f, 1 / pulse_frequency - pulse_active_duration);
    pulse_pause_duration *= float_rand(1 - pulse_interval_random, 1 + pulse_interval_random);
    pulse_total_duration = pulse_active_duration + pulse_pause_duration;

    // mix in potmeter
    float potmeter_value = BSP_ReadPotentiometerPercentage();
    potmeter_value = powf(potmeter_value, 1.f/2);
    pulse_amplitude *= potmeter_value;

    float amplitude_percentage = pulse_amplitude / (BODY_CURRENT_MAX * STIM_WINDING_RATIO);
    if (amplitude_percentage < .02f) {
        BSP_WriteLedPattern(LedPattern::PlayingVeryLow);
    } else if (amplitude_percentage < .2) {
        BSP_WriteLedPattern(LedPattern::PlayingLow);
    } else if (amplitude_percentage < .6) {
        BSP_WriteLedPattern(LedPattern::PlayingMedium);
    } else {
        BSP_WriteLedPattern(LedPattern::PlayingHigh);
    }

    // random polarity
    static bool polarity = false;
    static float random_start_angle;
#ifndef THREEPHASE_PULSE_DEFEAT_RANDOMIZATION
    polarity = !polarity;
    random_start_angle = _normalizeAngle(random_start_angle + (_2PI * 19 / 97)); // ~1/5
#endif

    // store stats
    traceline->amplitude = pulse_amplitude;
    total_pulse_length_timer.step();
    traceline->dt_compute = total_pulse_length_timer.dt_micros;


    // play the pulse
    if (play_status == PlayStatus::PlayingThreephase) {
        ComplexThreephasePoints points3 = project_threephase(
            pulse_amplitude,
            pulse_alpha,
            pulse_beta,
            calibration_center,
            calibration_ud,
            calibration_lr,
            polarity,
            random_start_angle);

        model3.play_pulse(points3.p1, points3.p2, points3.p3,
            pulse_carrier_frequency,
            pulse_width, pulse_rise,
            pulse_amplitude + ESTOP_CURRENT_LIMIT_MARGIN);
    } else if (play_status == PlayStatus::PlayingFourphase) {
        ComplexFourphasePoints points4 = project_fourphase(
            pulse_amplitude,
            pulse_alpha, pulse_beta, pulse_gamma,
            calibration_center,
            calibration_4a, calibration_4b, calibration_4c, calibration_4d,
            polarity,
            random_start_angle);

        model4.play_pulse(points4.p1, points4.p2, points4.p3, points4.p4,
            pulse_carrier_frequency,
            pulse_width, pulse_rise,
            pulse_amplitude + ESTOP_CURRENT_LIMIT_MARGIN);
    }

    float vbus_after = BSP_ReadVBus();

    // store stats
    total_pulse_length_timer.step();
    traceline->dt_play = total_pulse_length_timer.dt_micros;

    Complex z1 = play_status == PlayStatus::PlayingThreephase ? model3.z1 : model4.z1;
    Complex z2 = play_status == PlayStatus::PlayingThreephase ? model3.z2 : model4.z2;
    Complex z3 = play_status == PlayStatus::PlayingThreephase ? model3.z3 : model4.z3;
    Complex z4 = play_status == PlayStatus::PlayingThreephase ? Complex() : model4.z4;

    // store trace
    {
        if (play_status == PlayStatus::PlayingThreephase) {
            traceline->skipped_update_steps = model3.skipped_update_steps;
            traceline->v_drive_max = model3.v_drive_max;

            auto current_max = model3.current_max;
            traceline->i_max_a = current_max.a;
            traceline->i_max_b = current_max.b;
            traceline->i_max_c = current_max.c;
            traceline->i_max_d = 0;
        } else {
            traceline->skipped_update_steps = model4.skipped_update_steps;
            traceline->v_drive_max = model4.v_drive_max;

            auto current_max = model4.current_max;
            traceline->i_max_a = current_max.a;
            traceline->i_max_b = current_max.b;
            traceline->i_max_c = current_max.c;
            traceline->i_max_d = current_max.d;

        }
        traceline->Z_a = z1;
        traceline->Z_b = z2;
        traceline->Z_c = z3;
        traceline->Z_d = z4;
    }


    // print debug stats: impedance (xy)
    if (pulse_counter % 10 == 0) {
        Serial.printf("$");
        Serial.printf("Za:%f:%f|xy ", z1.a * STIM_WINDING_RATIO_SQ, z1.b * STIM_WINDING_RATIO_SQ);
        Serial.printf("Zb:%f:%f|xy ", z2.a * STIM_WINDING_RATIO_SQ, z2.b * STIM_WINDING_RATIO_SQ);
        Serial.printf("Zc:%f:%f|xy ", z3.a * STIM_WINDING_RATIO_SQ, z3.b * STIM_WINDING_RATIO_SQ);
        if (play_status == PlayStatus::PlayingFourphase)
            Serial.printf("Zd:%f:%f|xy ", z4.a * STIM_WINDING_RATIO_SQ, z4.b * STIM_WINDING_RATIO_SQ);
        Serial.printf("Z0:%f:%f|xy ", 0.f, 0.f);
        Serial.println();
    }

    // print debug stats: resistance
    if (pulse_counter % 10 == 2)
    {
        Serial.print("$");
        Serial.printf("R_a:%.2f ", z1.norm() * STIM_WINDING_RATIO_SQ);
        Serial.printf("R_b:%.2f ", z2.norm() * STIM_WINDING_RATIO_SQ);
        Serial.printf("R_c:%.2f ", z3.norm() * STIM_WINDING_RATIO_SQ);
        if (play_status == PlayStatus::PlayingFourphase)
            Serial.printf("R_d:%.2f ", z4.norm() * STIM_WINDING_RATIO_SQ);
        Serial.println();
    }

    // print debug stats: max current
    if (pulse_counter % 10 == 4)
    {
        if (play_status == PlayStatus::PlayingThreephase) {
            Serial.print("$");
            Serial.printf("I_max_a:%.1f ", abs(model3.current_max.a) / STIM_WINDING_RATIO * 1000);
            Serial.printf("I_max_b:%.1f ", abs(model3.current_max.b) / STIM_WINDING_RATIO * 1000);
            Serial.printf("I_max_c:%.1f ", abs(model3.current_max.c) / STIM_WINDING_RATIO * 1000);
            Serial.printf("I_max_cmd:%.1f ", abs(pulse_amplitude) / STIM_WINDING_RATIO * 1000);
            Serial.println();
            model3.v_drive_max = 0;
            model3.current_max = {};
        }
        if (play_status == PlayStatus::PlayingFourphase) {
            Serial.print("$");
            Serial.printf("I_max_a:%.1f ", abs(model4.current_max.a) / STIM_WINDING_RATIO * 1000);
            Serial.printf("I_max_b:%.1f ", abs(model4.current_max.b) / STIM_WINDING_RATIO * 1000);
            Serial.printf("I_max_c:%.1f ", abs(model4.current_max.c) / STIM_WINDING_RATIO * 1000);
            Serial.printf("I_max_d:%.1f ", abs(model4.current_max.d) / STIM_WINDING_RATIO * 1000);
            Serial.printf("I_max_cmd:%.1f ", abs(pulse_amplitude) / STIM_WINDING_RATIO * 1000);
            Serial.println();
            model4.v_drive_max = 0;
            model4.current_max = {};
        }
    }

    // print debug stats: rms current
    if (pulse_counter % 10 == 6)
    {
        rms_current_clock.step();

        if (play_status == PlayStatus::PlayingThreephase) {
            auto rms = model3.estimate_rms_current(rms_current_clock.dt_seconds);
            Serial.print("$");
            Serial.printf("rms_a:%.1f ", rms.a / STIM_WINDING_RATIO * 1000);
            Serial.printf("rms_b:%.1f ", rms.b / STIM_WINDING_RATIO * 1000);
            Serial.printf("rms_c:%.1f ", rms.c / STIM_WINDING_RATIO * 1000);
            Serial.println();
            model3.current_squared = {};
        }
        if (play_status == PlayStatus::PlayingFourphase) {
            auto rms = model4.estimate_rms_current(rms_current_clock.dt_seconds);
            Serial.print("$");
            Serial.printf("rms_a:%.1f ", rms.a / STIM_WINDING_RATIO * 1000);
            Serial.printf("rms_b:%.1f ", rms.b / STIM_WINDING_RATIO * 1000);
            Serial.printf("rms_c:%.1f ", rms.c / STIM_WINDING_RATIO * 1000);
            Serial.printf("rms_d:%.1f ", rms.d / STIM_WINDING_RATIO * 1000);
            Serial.println();
            model4.current_squared = {};
        }
    }

    // print debug stats: temp / volts
    if (pulse_counter % 20 == 8)
    {
        Serial.print("$");
        if (play_status == PlayStatus::PlayingThreephase) {
            Serial.printf("V_drive:%.3f ", model3.v_drive_max);
        }
        if (play_status == PlayStatus::PlayingFourphase) {
            Serial.printf("V_drive:%.3f ", model4.v_drive_max);
        }
        Serial.printf("temp_stm32:%.2f ", BSP_ReadTemperatureSTM());
        Serial.printf("V_BUS:%.2f ", vbus);
        Serial.printf("V_SYS:%.2f ", BSP_ReadVSYS());
        Serial.println();
    }

    // print debug stats: misc
    if (pulse_counter % 20 == 18)
    {
        Serial.print("$");
        Serial.printf("F_pulse:%.1f ", actual_pulse_frequency);
        // Serial.printf("model_steps:%i ", model.pulse_length_samples);
        // Serial.printf("model_skips:%i ", model.skipped_update_steps);
        Serial.printf("pot:%f ", potmeter_value);
        // Serial.printf("pot2:%f ", BSP_ReadPotentiometerPercentage());
        Serial.printf("boost_duty:%f ", BSP_BoostDutyCycle());
        Serial.println();

    }

    // // DEBUG
    // if (pulse_interval_random > 0.1) {
    //     model.debug_stats_teleplot();
    // }

    // store stats
    total_pulse_length_timer.step();
    traceline->dt_logs = total_pulse_length_timer.dt_micros;
}

#endif