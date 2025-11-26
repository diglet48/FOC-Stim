#if defined(ARDUINO_B_G431B_ESC1)

#include <Arduino.h>

#include "foc_utils.h"
#include "utils.h"
#include "stim_clock.h"
#include "trace.h"
#include "complex.h"
#include "signals/threephase_math.h"
#include "signals/threephase_model.h"

#include "bsp/bsp.h"

#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "focstim_rpc.pb.h"

#include "protobuf_api.h"

Trace trace{};
ThreephaseModel model3{};

enum PlayStatus{
    NotPlaying,
    PlayingThreephase,
};
static PlayStatus play_status = PlayStatus::NotPlaying;


class ESC1ProtobufAPI : public ProtobufAPI {
public:
    ESC1ProtobufAPI() {};

    focstim_rpc_Errors signal_start_threephase()
    {
        if (play_status != PlayStatus::NotPlaying) {
            return focstim_rpc_Errors_ERROR_ALREADY_PLAYING;
        }

        time_since_signal_start.reset();
        BSP_OutputEnable(true, true, true);
        play_status = PlayStatus::PlayingThreephase;
        return focstim_rpc_Errors_ERROR_UNKNOWN;
    }

    void signal_stop()
    {
        // transmit_notification_debug_string("STOP");

        BSP_OutputEnable(false, false, false);
        play_status = PlayStatus::NotPlaying;
    }

    virtual bool capability_threephase() {return true;};
    virtual bool capability_fourphase() {return false;};
    virtual bool capability_potmeter() {return true;};
    virtual bool capability_battery() {return false;};

    void transmit_notification_system_stats() {
        focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
        message.which_message = focstim_rpc_RpcMessage_notification_tag;
        message.message.notification.which_notification = focstim_rpc_Notification_notification_system_stats_tag;
        message.message.notification.notification.notification_system_stats.which_system = focstim_rpc_NotificationSystemStats_esc1_tag;
        message.message.notification.notification.notification_system_stats.system.esc1 = {
            .temp_stm32 = BSP_ReadTemperatureSTM(),
            .temp_board = BSP_ReadTemperatureOnboardNTC(),
            .v_bus = BSP_ReadVBus(),
            .v_ref = BSP_ReadChipAnalogVoltage(),
        };
        transmit_message(message);
    }

    Clock time_since_signal_start;
};

ESC1ProtobufAPI protobuf{};
ProtobufAPI* g_protobuf = &protobuf;

struct {
    SimpleAxis alpha{focstim_rpc_AxisType_AXIS_POSITION_ALPHA, 0, -1, 1};
    SimpleAxis beta{focstim_rpc_AxisType_AXIS_POSITION_BETA, 0, -1, 1};
    SimpleAxis waveform_amplitude_amps{focstim_rpc_AxisType_AXIS_WAVEFORM_AMPLITUDE_AMPS, 0, 0, BODY_CURRENT_MAX};
    SimpleAxis carrier_frequency{focstim_rpc_AxisType_AXIS_CARRIER_FREQUENCY_HZ, 800, 500, 2000};
    SimpleAxis pulse_frequency{focstim_rpc_AxisType_AXIS_PULSE_FREQUENCY_HZ, 50, 1, 100};
    SimpleAxis pulse_width{focstim_rpc_AxisType_AXIS_PULSE_WIDTH_IN_CYCLES, 6, 3, 20};
    SimpleAxis pulse_rise{focstim_rpc_AxisType_AXIS_PULSE_RISE_TIME_CYCLES, 5, 2, 10};
    SimpleAxis pulse_interval_random{focstim_rpc_AxisType_AXIS_PULSE_INTERVAL_RANDOM_PERCENT, 0, 0, 1};
    SimpleAxis calib_center{focstim_rpc_AxisType_AXIS_CALIBRATION_3_CENTER, 0, -10, 10};
    SimpleAxis calib_ud{focstim_rpc_AxisType_AXIS_CALIBRATION_3_UP, 0, -10, 10};
    SimpleAxis calib_lr{focstim_rpc_AxisType_AXIS_CALIBRATION_3_LEFT, 0, -10, 10};
} simple_axes;

void estop_triggered(FOCError error)
{
    trace.print_mainloop_trace();
    model3.debug_stats_teleplot();
}

void setup()
{
    Serial.begin(115200);
    delay(100);
    protobuf.init();
    protobuf.set_simple_axis(
        reinterpret_cast<SimpleAxis *>(&simple_axes),
        sizeof(simple_axes) / sizeof(SimpleAxis));
    BSP_PrintDebugMsg("BSP init");
    BSP_Init();

    protobuf.transmit_notification_boot();
    model3.init(&estop_triggered);
}

void loop()
{
    static Clock total_pulse_length_timer;
    static float pulse_total_duration = 0;
    static Clock actual_pulse_frequency_clock;
    static Clock rms_current_clock;
    static Clock status_print_clock;
    static Clock vbus_print_clock;
    static uint32_t pulse_counter = 0;
    static float actual_pulse_frequency = 0;
    static bool led_status = true;
    static Clock print_system_stats_clock;
    static float v_drive_max = 0;

    static Clock potentiometer_notification_nospam;
    static float potentiometer_notification_lastvalue = 0;

    // TODO: led toggling

    // do comms
    bool any_frames_received = protobuf.process_incoming_messages();

    // toggle LED
    if (any_frames_received) {
        led_status = ! led_status;
        BSP_WriteStatusLED(led_status);
    }

    // checks: board temperature
    float board_temperature = BSP_ReadTemperatureOnboardNTC();
    float stm_temperature = BSP_ReadTemperatureSTM();
    if ((board_temperature >= MAXIMUM_TEMPERATURE) || (stm_temperature >= MAXIMUM_TEMPERATURE)) {
        protobuf.signal_stop();
        while (1)
        {
            BSP_PrintDebugMsg(
                "temperature limit exceeded %.2f. Current board=%.2f stm32=%.2f. Restart device to proceed.",
                std::max(board_temperature, stm_temperature), board_temperature, stm_temperature);
            delay(5000);
        }
    }

    float vbus = BSP_ReadVBus();
    if (vbus >= STIM_PSU_VOLTAGE_MAX) {
        protobuf.signal_stop();
        while (1)
        {
            BSP_PrintDebugMsg("V_BUS overvoltage detected %.2f. Current V_BUS=%.2f. Restart device to proceed.",
                          vbus, BSP_ReadVBus());
            delay(5000);
        }
    }

    // check vbus, stop playing if vbus is too low.
    if (vbus < STIM_PSU_VOLTAGE_MIN) {
        // vbus changed high->low.
        if (play_status == PlayStatus::PlayingThreephase) {
            BSP_PrintDebugMsg("V_BUS under-voltage detected (V_BUS = %.2f). Stopping pulse generation.", vbus);
            protobuf.signal_stop();
            vbus_print_clock.reset();
        }

        // print something if vbus has been low for a while to alert user they should flip power switch on the device.
        vbus_print_clock.step();
        if (vbus_print_clock.time_seconds > 4) {
            vbus_print_clock.reset();
            BSP_PrintDebugMsg("V_BUS too low: %.2f. Turn ON the power.", vbus);
        }
    }

    // transmit potmeter notification
    potentiometer_notification_nospam.step();
    bool do_transmit_potmeter = false;
    do_transmit_potmeter |= (potentiometer_notification_nospam.time_seconds > 1);
    do_transmit_potmeter |= (potentiometer_notification_nospam.time_seconds > 0.1f
        && abs(BSP_ReadPotentiometerPercentage() - potentiometer_notification_lastvalue) >= 0.001f);
    if (do_transmit_potmeter) {
        float pot = BSP_ReadPotentiometerPercentage();
        protobuf.transmit_notification_potentiometer(powf(pot, 1.f/2));
        potentiometer_notification_nospam.reset();
        potentiometer_notification_lastvalue = pot;
    }

    // every few seconds, print system stats
    print_system_stats_clock.step();
    if (print_system_stats_clock.time_seconds > 5) {
        print_system_stats_clock.reset();
        protobuf.transmit_notification_system_stats();
    }

    // correct for drift in the current sense circuit
    BSP_AdjustCurrentSenseOffsets();

    // DSTART / DSTOP
    if (play_status == PlayStatus::NotPlaying) {
        delay(5);
        return;
    }

    // keepalive timer. Stop playing if no messages have been received for some time.
    if (protobuf.time_since_last_axis_command.time_seconds > 4) {
        BSP_PrintDebugMsg("Comms lost? Stopping.");
        play_status = PlayStatus::NotPlaying;
        protobuf.signal_stop();
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
    uint32_t now_ms = millis();
    float pulse_alpha = simple_axes.alpha.get(now_ms);
    float pulse_beta = simple_axes.beta.get(now_ms);
    float body_current_amps = simple_axes.waveform_amplitude_amps.get(now_ms);

    float pulse_carrier_frequency = simple_axes.carrier_frequency.get(now_ms);
    float pulse_frequency = simple_axes.pulse_frequency.get(now_ms);
    float pulse_width = simple_axes.pulse_width.get(now_ms);
    float pulse_rise = simple_axes.pulse_rise.get(now_ms);
    float pulse_interval_random = simple_axes.pulse_interval_random.get(now_ms);

    float calibration_center = simple_axes.calib_center.get(now_ms);
    float calibration_lr = simple_axes.calib_lr.get(now_ms);
    float calibration_ud = simple_axes.calib_ud.get(now_ms);

    float pulse_active_duration = pulse_width / pulse_carrier_frequency;
    float pulse_pause_duration = max(0.f, 1 / pulse_frequency - pulse_active_duration);
    pulse_pause_duration *= float_rand(1 - pulse_interval_random, 1 + pulse_interval_random);
    pulse_total_duration = pulse_active_duration + pulse_pause_duration;
    traceline->dt_next = pulse_total_duration * 1e6f;

    // mix in potmeter
    float potmeter_value = BSP_ReadPotentiometerPercentage();
    potmeter_value = powf(potmeter_value, 1.f/2);
    body_current_amps *= potmeter_value;

    // calculate amplitude in amperes (driving current)
    float driving_current_amps = body_current_amps * STIM_WINDING_RATIO;

    // random polarity
    static bool polarity = false;
    static float random_start_angle;
#ifndef THREEPHASE_PULSE_DEFEAT_RANDOMIZATION
    polarity = !polarity;
    random_start_angle = _normalizeAngle(random_start_angle + (_2PI * 19 / 97)); // ~1/5
#endif

    // store stats
    traceline->i_max_cmd = driving_current_amps;
    total_pulse_length_timer.step();

    // play the pulse
    ComplexThreephasePoints points3 = project_threephase(
        driving_current_amps,
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
        driving_current_amps + ESTOP_CURRENT_LIMIT_MARGIN);

    // store stats
    total_pulse_length_timer.step();
    traceline->dt_play = total_pulse_length_timer.dt_micros;

    // store trace
    {
        traceline->skipped_update_steps = model3.skipped_update_steps;
        traceline->v_drive = model3.v_drive_last;
        v_drive_max = max(v_drive_max, model3.v_drive_last);
        auto current_max = model3.current_max;
        traceline->i_max_a = current_max.a;
        traceline->i_max_b = current_max.b;
        traceline->i_max_c = current_max.c;

        traceline->Z_a = model3.z1;
        traceline->Z_b = model3.z2;
        traceline->Z_c = model3.z3;
    }

     // send notification: pulse current
    if (pulse_counter % 50 == 0) {
    // if (pulse_counter % 1 == 0) {
        rms_current_clock.step();
        auto rms = model3.estimate_rms_current(rms_current_clock.dt_seconds);
        auto r_a = model3.z1.a;
        auto r_b = model3.z2.a;
        auto r_c = model3.z3.a;
        float p =
            (rms.a * rms.a) * r_a +
            (rms.b * rms.b) * r_b +
            (rms.c * rms.c) * r_c;

        protobuf.transmit_notification_currents(
            rms.a / STIM_WINDING_RATIO,
            rms.b / STIM_WINDING_RATIO,
            rms.c / STIM_WINDING_RATIO,
            0,
            abs(model3.current_max.a) / STIM_WINDING_RATIO,
            abs(model3.current_max.b) / STIM_WINDING_RATIO,
            abs(model3.current_max.c) / STIM_WINDING_RATIO,
            0,
            p, 0,
            abs(driving_current_amps) / STIM_WINDING_RATIO
        );
        model3.current_max = {};
        model3.current_squared = {};
    }

    // send notification: skin resistance estimation
    if (pulse_counter % 50 == 20) {
    // if (pulse_counter % 1 == 0) {
        float m = STIM_WINDING_RATIO_SQ;
        protobuf.transmit_notification_model_estimation(
            model3.z1.a * m, model3.z1.b * m,
            model3.z2.a * m, model3.z2.b * m,
            model3.z3.a * m, model3.z3.b * m,
            0, 0);
    }

    // send notification: pulse stats
    if (pulse_counter % 50 == 40) {
        // transmit_notification
        protobuf.transmit_notification_signal_stats(actual_pulse_frequency, v_drive_max);
        v_drive_max = 0;
    }
}

#endif