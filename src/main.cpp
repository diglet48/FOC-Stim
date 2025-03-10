#include <Arduino.h>

#include "foc_utils.h"
#include "tcode.h"
#include "utils.h"
#include "stim_clock.h"
#include "pulse/threephase_pulse_buffer.h"
#include "mrac_threephase_star.h"
#include "config.h"
#include "trace.h"

#include "bsp/bsp.h"

MRACThreephaseStar mrac{};
ThreephasePulseBuffer pulse_threephase{};
Trace trace{};


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
    TCodeAxis volume{"V0", 0, 0, TCODE_MAX_CURRENT};
    TCodeAxis carrier_frequency{"A0", 800, 500, 2000};
    TCodeAxis pulse_frequency{"A1", 50, 1, 100};
    TCodeAxis pulse_width{"A2", 6, 3, 20};
    TCodeAxis pulse_rise{"A3", 5, 2, 10};
    TCodeAxis pulse_interval_random{"A4", 0, 0, 1};
    TCodeAxis calib_center{"C0", 0, -10, 10};
    TCodeAxis calib_ud{"C1", 0, -10, 10};
    TCodeAxis calib_lr{"C2", 0, -10, 10};
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
    mrac.print_debug_stats();
    trace.print_mainloop_trace();
}

void setup()
{
    Serial.begin(115200);
    Serial.println("BSP init");
    BSP_Init();

    // print status to let the computer know we have rebooted.
    print_status();
    BSP_WriteStatusLED(1);

    mrac.init(&estop_triggered);

    status_booted = true;
    float vbus = BSP_ReadVBus();
    if (vbus > STIM_PSU_VOLTAGE_MIN) {
        status_vbus = true;
    }
    print_status();

    BSP_EnableOutputs();
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
    float pulse_amplitude = axes.volume.get_remap(t0); // pulse amplitude in amps
    float pulse_carrier_frequency = axes.carrier_frequency.get_remap(t0);
    float pulse_frequency = axes.pulse_frequency.get_remap(t0);
    float pulse_width = axes.pulse_width.get_remap(t0);
    float pulse_rise = axes.pulse_rise.get_remap(t0);
    float pulse_interval_random = axes.pulse_interval_random.get_remap(t0);

    float calibration_center = axes.calib_center.get_remap(t0);
    float calibration_lr = axes.calib_lr.get_remap(t0);
    float calibration_ud = axes.calib_ud.get_remap(t0);

    float pulse_active_duration = pulse_width / pulse_carrier_frequency;
    float pulse_pause_duration = max(0.f, 1 / pulse_frequency - pulse_active_duration);
    pulse_pause_duration *= float_rand(1 - pulse_interval_random, 1 + pulse_interval_random);
    pulse_total_duration = pulse_active_duration + pulse_pause_duration;

    // mix in potmeter
    float potmeter_voltage = BSP_ReadPotentiometer();
    float potmeter_value = inverse_lerp(potmeter_voltage, POTMETER_ZERO_PERCENT_VOLTAGE, POTMETER_HUNDRED_PERCENT_VOLTAGE);
    pulse_amplitude *= potmeter_value;

    // pre-compute the new pulse
    pulse_threephase.create_pulse(
        pulse_amplitude,
        pulse_alpha,
        pulse_beta,
        pulse_carrier_frequency,
        pulse_width,
        pulse_rise,
        calibration_center,
        calibration_ud,
        calibration_lr);

    // store stats
    traceline->amplitude = pulse_amplitude;
    total_pulse_length_timer.step();
    traceline->dt_compute = total_pulse_length_timer.dt_micros;

    // play the pulse
    mrac.play_pulse(&pulse_threephase, pulse_amplitude + ESTOP_CURRENT_LIMIT_MARGIN);
    total_pulse_length_timer.step();

    // store stats
    Clock stats_timer;
    traceline->dt_play = total_pulse_length_timer.dt_micros;

    traceline->skipped_update_steps = mrac.skipped_update_steps;
    traceline->v_drive_max = mrac.v_drive_max;
    traceline->max_recorded_current_neutral = mrac.current_max_neutral;
    traceline->max_recorded_current_left = mrac.current_max_left;
    traceline->max_recorded_current_right = mrac.current_max_right;

    traceline->R_neutral = mrac.estimate_resistance_neutral();
    traceline->R_left = mrac.estimate_resistance_left();
    traceline->R_right = mrac.estimate_resistance_right();
    traceline->L = mrac.estimate_inductance();

    // occasionally print some stats..
    if ((pulse_counter + 0) % 10 == 0)
    {
        Serial.print("$");
        Serial.printf("R_neutral:%.2f ", mrac.estimate_resistance_neutral());
        Serial.printf("R_left:%.2f ", mrac.estimate_resistance_left());
        Serial.printf("R_right:%.2f ", mrac.estimate_resistance_right());
        Serial.printf("L:%.2f ", mrac.estimate_inductance() * 1e6f);
        Serial.println();
    }
    if ((pulse_counter + 2) % 10 == 0)
    {
        Serial.print("$");
        Serial.printf("V_drive:%.2f ", mrac.v_drive_max);
        Serial.printf("I_max_a:%f ", abs(mrac.current_max_neutral));
        Serial.printf("I_max_b:%f ", abs(mrac.current_max_left));
        Serial.printf("I_max_c:%f ", abs(mrac.current_max_right));
        Serial.println();
        mrac.v_drive_max = 0;
        mrac.current_max_neutral = 0;
        mrac.current_max_left = 0;
        mrac.current_max_right = 0;
    }
    if ((pulse_counter + 4) % 10 == 0)
    {

    }
    if ((pulse_counter + 6) % 10 == 0)
    {   
        rms_current_clock.step();
        Serial.print("$");
        Serial.printf("rms_neutral:%f ", sqrtf(mrac.current_squared_neutral / rms_current_clock.dt_seconds));
        Serial.printf("rms_left:%f ", sqrtf(mrac.current_squared_left / rms_current_clock.dt_seconds));
        Serial.printf("rms_right:%f ", sqrtf(mrac.current_squared_right / rms_current_clock.dt_seconds));
        Serial.println();
        mrac.current_squared_neutral = 0;
        mrac.current_squared_left = 0;
        mrac.current_squared_right = 0;
    }
    if ((pulse_counter + 8) % 20 == 0)
    {
        Serial.print("$");
        Serial.printf("V_BUS:%.2f ", BSP_ReadVBus());
        Serial.printf("temp:%.1f ", BSP_ReadTemperatureOnboardNTC());
        Serial.printf("F_pulse:%.1f ", actual_pulse_frequency);
        Serial.printf("model_steps:%i ", mrac.pwm_write_index);
        Serial.printf("model_skips:%i ", mrac.skipped_update_steps);
        Serial.printf("pot:%f ", potmeter_value);
        Serial.println();
    }

    stats_timer.step();
    traceline->dt_logs = stats_timer.dt_micros;
}
