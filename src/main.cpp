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

#include "bsp/bsp.h"


Trace trace{};

#if defined(BSP_ENABLE_THREEPHASE)
ThreephaseModel model{};
#elif defined(BSP_ENABLE_FOURPHASE)
FourphaseModel model{};
#endif


static bool status_booted = false;
static bool status_vbus = false;
static bool status_estop = false;
static bool status_playing = false;

static Clock keepalive_clock{};


void print_status() {
    uint32_t status =
        (status_booted << 0)
        | (status_vbus << 1)
        | (status_estop << 2)
        | (status_playing << 3);
    Serial.printf("status: %lu\r\n", status);
}


void tcode_D0() {
    Serial.println();
    Serial.printf("version: FOC-Stim 0.5\r\n");
    print_status();
}

void tcode_DSTART() {
    if (status_vbus && !status_playing) {
        Serial.printf("Pulse generation started\r\n");
        status_playing = true;
    }
}

void tcode_DSTOP() {
    if (status_vbus && status_playing) {
        Serial.printf("Pulse generation stopped\r\n");
        status_playing = false;
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
    TCodeAxis volume{"V0", 0, 0, TCODE_MAX_CURRENT};
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
    TCodeDeviceCommand d_start{"START", &tcode_DSTART};
    TCodeDeviceCommand d_stop{"STOP", &tcode_DSTOP};
    TCodeDeviceCommand d_ping{"PING", &tcode_DPING};
} tcode_device_commands;
TCode tcode(reinterpret_cast<TCodeAxis *>(&axes), sizeof(axes) / sizeof(TCodeAxis),
            reinterpret_cast<TCodeDeviceCommand *>(&tcode_device_commands), sizeof(tcode_device_commands) / sizeof(TCodeDeviceCommand));

void estop_triggered()
{
    trace.print_mainloop_trace();
    model.debug_stats_teleplot();
}

void setup()
{
    Serial.begin(115200);
    delay(100);
    Serial.println("BSP init");
    BSP_Init();

    // print status to let the computer know we have rebooted.
    print_status();
    BSP_WriteStatusLED(1);

    // TODO:
    model.init(&estop_triggered);

    status_booted = true;
    float vbus = BSP_ReadVBus();
    if (vbus > STIM_PSU_VOLTAGE_MIN) {
        status_vbus = true;
    }
    print_status();


#if defined(BSP_ENABLE_THREEPHASE)
    BSP_OutputEnable(true, true, true);
#elif defined(BSP_ENABLE_FOURPHASE)
    BSP_OutputEnable(true, true, true, true);
#endif
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
    static bool led_status = true;

    // do comms
    bool dirty = tcode.update_from_serial();

    // safety checks
    float temperature = BSP_ReadTemperatureOnboardNTC();
    if (temperature >= MAXIMUM_TEMPERATURE) {
        BSP_DisableOutputs();
        while (1)
        {
            Serial.printf("temperature limit exceeded %.2f. Current temperature=%.2f. Restart device to proceed\r\n",
                          temperature, BSP_ReadTemperatureOnboardNTC());
            delay(5000);
        }
    }

    float vbus = BSP_ReadVBus();
    if (vbus >= STIM_PSU_VOLTAGE_MAX) {
        BSP_DisableOutputs();
        while (1)
        {
            Serial.printf("V_BUS overvoltage detected %.2f. Current V_BUS=%.2f. Restart device to proceed\r\n",
                          vbus, BSP_ReadVBus());
            delay(5000);
        }
    }
    
    // check vbus, stop playing if vbus is too low.
    if (vbus < STIM_PSU_VOLTAGE_MIN) {
        vbus_high_clock.reset();

        // vbus changed high->low.
        if (status_vbus) {
            status_vbus = false;
            status_playing = false;
            Serial.printf("V_BUS under-voltage detected (V_BUS = %.2f). Stopping pulse generation.\r\n", vbus);
            print_status();
            vbus_print_clock.reset();
        }

        // print something if vbus has been low for a while to alert user they should flip power switch on the device.
        vbus_print_clock.step();
        if (vbus_print_clock.time_seconds > 4) {
            vbus_print_clock.reset();
            Serial.printf("V_BUS too low: %.2f\r\n", vbus);
        }
    } else {
        // if vbus is high and stable
        if (!status_vbus && vbus_high_clock.time_seconds > 0.2f) {
            Serial.printf("V_BUS online. V_BUS: %.2f\r\n", vbus);
            status_vbus = true;
            status_playing = false;
            print_status();
        }
        vbus_high_clock.step();
    }

    // keepalive timer. Stop playing if no messages have been received for some time.
    if (dirty) {
        keepalive_clock.reset();

        // toggle the LED every time serial comms is received.
        led_status = !led_status;
        BSP_WriteStatusLED(led_status);
    }
    keepalive_clock.step();
    if (keepalive_clock.time_seconds > 2 && status_playing) {
        Serial.println("Connection lost? Stopping pulse generation.");
        status_playing = false;
        print_status();
    }

    // correct for drift in the current sense circuit
    BSP_AdjustCurrentSenseOffsets();

    // DSTART / DSTOP
    if (! status_playing) {
        delay(10);
        return;
    }

    // stall out idle portion of the pulse
    total_pulse_length_timer.step();
    if (total_pulse_length_timer.time_seconds <= pulse_total_duration) {
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
    float potmeter_voltage = BSP_ReadPotentiometer();
    float potmeter_value = inverse_lerp(potmeter_voltage, POTMETER_ZERO_PERCENT_VOLTAGE, POTMETER_HUNDRED_PERCENT_VOLTAGE);
    pulse_amplitude *= potmeter_value;

    // store stats
    traceline->amplitude = pulse_amplitude;
    total_pulse_length_timer.step();
    traceline->dt_compute = total_pulse_length_timer.dt_micros;


    static bool polarity = false;
    static float random_start_angle;
#ifndef THREEPHASE_PULSE_DEFEAT_RANDOMIZATION
    polarity = !polarity;
    random_start_angle = _normalizeAngle(random_start_angle + (_2PI * 19 / 97)); // ~1/5
#endif


#if defined(BSP_ENABLE_THREEPHASE)
    ComplexThreephasePoints points3 = project_threephase(
        pulse_amplitude,
        pulse_alpha,
        pulse_beta,
        calibration_center,
        calibration_ud,
        calibration_lr,
        polarity,
        random_start_angle);

    model.play_pulse(points3.p1, points3.p2, points3.p3, 
        pulse_carrier_frequency,
        pulse_width, pulse_rise,
        pulse_amplitude + ESTOP_CURRENT_LIMIT_MARGIN);
#elif defined(BSP_ENABLE_FOURPHASE)
    ComplexFourphasePoints points4 = project_fourphase(
        pulse_amplitude,
        pulse_alpha, pulse_beta, pulse_gamma,
        calibration_center,
        calibration_4a, calibration_4b, calibration_4c, calibration_4d,
        polarity,
        random_start_angle);

    model.play_pulse(points4.p1, points4.p2, points4.p3, points4.p4, 
        pulse_carrier_frequency,
        pulse_width, pulse_rise,
        pulse_amplitude + ESTOP_CURRENT_LIMIT_MARGIN);
#endif

    if (pulse_counter % 3 == 0) {
        Serial.printf("$");
        Serial.printf("Za:%f:%f|xy ", model.z1.a, model.z1.b);
        Serial.printf("Zb:%f:%f|xy ", model.z2.a, model.z2.b);
        Serial.printf("Zc:%f:%f|xy ", model.z3.a, model.z3.b);
#if defined(BSP_ENABLE_FOURPHASE)
        Serial.printf("Zd:%f:%f|xy ", model.z4.a, model.z4.b);
#endif
        Serial.printf("Z0:%f:%f|xy ", 0.f, 0.f);
    
//         Serial.printf("L1:%.1f ", model.z1.b / (_2PI * 1000) * 1e6f);
//         Serial.printf("L2:%.1f ", model.z2.b / (_2PI * 1000) * 1e6f);
//         Serial.printf("L3:%.1f ", model.z3.b / (_2PI * 1000) * 1e6f);
// #if defined(BSP_ENABLE_FOURPHASE)
//         Serial.printf("L4:%.1f ", model.z4.b / (_2PI * 1000) * 1e6f);
// #endif
    
        Serial.println();
    }

    // play the pulse
    total_pulse_length_timer.step();

    // store stats
    Clock stats_timer;
    traceline->dt_play = total_pulse_length_timer.dt_micros;

    // traceline->skipped_update_steps = mrac.skipped_update_steps;
    // traceline->v_drive_max = mrac.v_drive_max;
    // traceline->max_recorded_current_neutral = mrac.current_max.a;
    // traceline->max_recorded_current_left = mrac.current_max.b;
    // traceline->max_recorded_current_right = mrac.current_max.c;

    // auto resistance = mrac.estimate_resistance();
    // traceline->R_neutral = resistance.a;
    // traceline->R_left = resistance.b;
    // traceline->R_right = resistance.c;
    // traceline->L = mrac.estimate_inductance();

    // occasionally print some stats..
    // if ((pulse_counter + 0) % 10 == 0)
    // {
    //     Serial.print("$");
    //     Serial.printf("R_neutral:%.2f ", resistance.a);
    //     Serial.printf("R_left:%.2f ", resistance.b);
    //     Serial.printf("R_right:%.2f ", resistance.c);
    //     // Serial.printf("R_zzz:%.2f ", resistance.d);
    //     Serial.printf("L:%.2f ", mrac.estimate_inductance() * 1e6f);
    //     Serial.println();
    // }
    // if ((pulse_counter + 2) % 10 == 0)
    if ((pulse_counter + 2) % 1 == 0)
    {
        Serial.print("$");
        Serial.printf("V_drive:%.3f ", model.v_drive_max);
        Serial.printf("I_max_a:%f ", abs(model.current_max.a));
        Serial.printf("I_max_b:%f ", abs(model.current_max.b));
        Serial.printf("I_max_c:%f ", abs(model.current_max.c));
#if defined(BSP_ENABLE_FOURPHASE)
        Serial.printf("I_max_d:%f ", abs(model.current_max.d));
#endif
        Serial.printf("I_max_cmd:%f ", abs(pulse_amplitude));
        Serial.println();
        model.v_drive_max = 0;
        model.current_max = {};
    }
    if ((pulse_counter + 4) % 10 == 0)
    {

    }
    if ((pulse_counter + 6) % 10 == 0)
    {   
        rms_current_clock.step();
        auto rms = model.estimate_rms_current(rms_current_clock.dt_seconds);
        Serial.print("$");
        Serial.printf("rms_a:%f ", rms.a);
        Serial.printf("rms_b:%f ", rms.b);
        Serial.printf("rms_c:%f ", rms.c);
#if defined(BSP_ENABLE_FOURPHASE)
        Serial.printf("rms_d:%f ", rms.d);
#endif
        Serial.println();
        model.current_squared = {};
    }
    if ((pulse_counter + 8) % 20 == 0)
    {
        Serial.print("$");
        Serial.printf("V_BUS:%.2f ", BSP_ReadVBus());
        Serial.printf("temp_board:%.2f ", BSP_ReadTemperatureOnboardNTC());
        Serial.printf("temp_stm32:%.2f ", BSP_ReadTemperatureInternal());
        Serial.printf("V_ref:%.5f ", BSP_ReadChipAnalogVoltage());
        Serial.printf("F_pulse:%.1f ", actual_pulse_frequency);
        Serial.printf("model_steps:%i ", model.pulse_length_samples);
        Serial.printf("model_skips:%i ", model.skipped_update_steps);
        Serial.printf("pot:%f ", potmeter_value);
        Serial.println();
    }

    // if (pulse_interval_random > 0.1) {
    //     model.debug_stats_teleplot();
    // }

    stats_timer.step();
    traceline->dt_logs = stats_timer.dt_micros;
}
