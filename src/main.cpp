#include <Arduino.h>
#include <SimpleFOC.h>

#include "tcode.h"
#include "utils.h"
#include "stim_clock.h"
#include "pulse_single.h"
#include "pulse_threephase.h"
#include "mrac_threephase_star.h"
#include "config.h"
#include "emergency_stop.h"
#include "trace.h"

BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
// Gain calculation shown at https://community.simplefoc.com/t/b-g431b-esc1-current-control/521/21
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

MRACThreephaseStar mrac{};
ThreephasePulse pulse_threephase{};
Trace trace{};

EmergencyStop emergencyStop{};

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
    Serial.printf("version: FOC-Stim 0.3\r\n");
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
    TCodeAxis carrier_frequency{"A0", 800, 500, 1000};
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
    // use monitoring with serial
    Serial.begin(115200);
    Serial.println("- begin setup");
    // enable more verbose output for debugging
    // comment out if not needed
    // SimpleFOCDebug::enable(&Serial);

    // print status to let the computer know we have rebooted.
    print_status();

    Serial.println("- init driver");
    driver.voltage_power_supply = STIM_PSU_VOLTAGE;
    driver.pwm_frequency = STIM_PWM_FREQ;
    driver.init();

    Serial.println("- init emergency stop");
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
    driver.enable();

    Serial.printf("- init mrac\r\n");
    mrac.init(&driver, &currentSense, &emergencyStop);

    status_booted = true;
    float vbus = read_vbus(&currentSense);
    if (vbus > STIM_PSU_VOLTAGE_MIN) {
        status_vbus = true;
    }
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
    emergencyStop.check_temperature();
    emergencyStop.check_vbus_overvoltage();

    // check vbus, stop playing if vbus is too low.
    float vbus = read_vbus(&currentSense);
    if (vbus < STIM_PSU_VOLTAGE_MIN) {
        vbus_high_clock.reset();

        // vbus changed high->low.
        if (status_vbus) {
            read_vbus(&currentSense);
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
    }
    keepalive_clock.step();
    if (keepalive_clock.time_seconds > 2 && status_playing) {
        Serial.println("Connection lost? Stopping pulse generation.");
        status_playing = false;
        print_status();
    }

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
    float potmeter_voltage = read_potentiometer(&currentSense);
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

    // reset mrac stats
    mrac.v_drive_max = 0;

    // play the pulse
    mrac.pulse_begin();
    uint32_t number_of_iterations = 0;
    Clock pulse_active_timer;
    while (pulse_active_timer.time_seconds < pulse_threephase.pulse_duration)
    {
        number_of_iterations++;
        pulse_active_timer.step();
        float desired_current_neutral = 0;
        float desired_current_left = 0;
        pulse_threephase.get(pulse_active_timer.time_seconds,
                             &desired_current_neutral, &desired_current_left);
        mrac.iter(desired_current_neutral, desired_current_left);
    }
    pulse_active_timer.step();

    // wait until the lines stabilize
    Clock pulse_stabilize_timer;
    mrac.pulse_end();
    pulse_stabilize_timer.step();

    // store stats
    Clock stats_timer;
    float iters_per_sec = number_of_iterations / pulse_active_timer.time_seconds;
    traceline->mrac_iters = number_of_iterations;
    traceline->dt_play = pulse_active_timer.time_milis;
    traceline->dt_stabilize = pulse_stabilize_timer.time_milis;

    traceline->v_drive_max = mrac.v_drive_max;
    traceline->max_recorded_current_neutral = emergencyStop.max_recorded_current_neutral;
    traceline->max_recorded_current_left = emergencyStop.max_recorded_current_left;
    traceline->max_recorded_current_right = emergencyStop.max_recorded_current_right;

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
        Serial.printf("I_max_a:%f ", abs(emergencyStop.max_recorded_current_neutral));
        Serial.printf("I_max_b:%f ", abs(emergencyStop.max_recorded_current_left));
        Serial.printf("I_max_c:%f ", abs(emergencyStop.max_recorded_current_right));
        Serial.println();
        emergencyStop.max_recorded_current_neutral = 0;
        emergencyStop.max_recorded_current_left = 0;
        emergencyStop.max_recorded_current_right = 0;
    }
    if ((pulse_counter + 4) % 10 == 0)
    {

    }
    if ((pulse_counter + 6) % 10 == 0)
    {   
        rms_current_clock.step();
        Serial.print("$");
        Serial.printf("rms_neutral:%f ", sqrtf(mrac.current_squared_a / rms_current_clock.dt_seconds));
        Serial.printf("rms_left:%f ", sqrtf(mrac.current_squared_b / rms_current_clock.dt_seconds));
        Serial.printf("rms_right:%f ", sqrtf(mrac.current_squared_c / rms_current_clock.dt_seconds));
        Serial.println();
        mrac.current_squared_a = 0;
        mrac.current_squared_b = 0;
        mrac.current_squared_c = 0;
    }
    if ((pulse_counter + 8) % 20 == 0)
    {
        Serial.print("$");
        Serial.printf("V_BUS:%.2f ", read_vbus(&currentSense));
        Serial.printf("temp:%.1f ", read_temperature(&currentSense));
        Serial.printf("F_mrac:%.0f ", iters_per_sec);
        Serial.printf("F_pulse:%.1f ", actual_pulse_frequency);
        Serial.printf("pot:%f ", potmeter_value);
        Serial.printf("vrefint:%f ", read_vrefint(&currentSense));
        Serial.printf("temp_int:%f ", read_temperature_internal(&currentSense));
        Serial.printf("Ia:%f ", currentSense.getPhaseCurrents().a);
        Serial.printf("Ib:%f ", currentSense.getPhaseCurrents().b);
        Serial.printf("Ic:%f ", currentSense.getPhaseCurrents().c);
        Serial.println();
    }

    stats_timer.step();
    traceline->dt_logs = stats_timer.dt_micros;
}
