#if defined(ARDUINO_FOCSTIM_V3)

#include <Arduino.h>

#include "foc_utils.h"
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
#include "axis/buffered_axis.h"
#include "axis/simple_axis.h"
#include "timestamp_sync.h"
#include "sensors/as5311.h"

#include "bsp/bsp.h"
#include "bsp/bootloader.h"

#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "focstim_rpc.pb.h"

#include "protobuf_api.h"


Trace trace{};
ThreephaseModel model3{};
FourphaseModel model4{};
PowerManager power_manager{};

enum PlayStatus{
    NotPlaying,
    PlayingThreephase,
    PlayingFourphase
};
static PlayStatus play_status = PlayStatus::NotPlaying;


class FocstimV3ProtobufAPI : public ProtobufAPI {
public:
    FocstimV3ProtobufAPI() {};

    focstim_rpc_Errors signal_start_threephase()
    {
        if (play_status != PlayStatus::NotPlaying) {
            return focstim_rpc_Errors_ERROR_ALREADY_PLAYING;
        }
        // transmit_notification_debug_string("START 3");

        time_since_signal_start.reset();
        BSP_OutputEnable(true, true, true);
        BSP_SetBoostEnable(true);
        BSP_WriteLedPattern(LedPattern::PlayingVeryLow);
        play_status = PlayStatus::PlayingThreephase;
        return focstim_rpc_Errors_ERROR_UNKNOWN;
    }

    focstim_rpc_Errors signal_start_fourphase()
    {
        if (play_status != PlayStatus::NotPlaying) {
            return focstim_rpc_Errors_ERROR_ALREADY_PLAYING;
        }

        // transmit_notification_debug_string("START 4");

        time_since_signal_start.reset();
        BSP_OutputEnable(true, true, true, true);
        BSP_SetBoostEnable(true);
        BSP_WriteLedPattern(LedPattern::PlayingVeryLow);
        play_status = PlayStatus::PlayingFourphase;
        return focstim_rpc_Errors_ERROR_UNKNOWN;
    }

    void signal_stop()
    {
        // transmit_notification_debug_string("STOP");

        BSP_OutputEnable(false, false, false);
        BSP_SetBoostEnable(false);
        BSP_WriteLedPattern(LedPattern::Idle);
        play_status = PlayStatus::NotPlaying;
    }

    virtual bool capability_threephase() {return true;};
    virtual bool capability_fourphase() {return true;};
    virtual bool capability_potmeter() {return true;};
    virtual bool capability_battery() {return power_manager.is_battery_present;};

    void transmit_notification_system_stats() {
        focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
        message.which_message = focstim_rpc_RpcMessage_notification_tag;
        message.message.notification.which_notification = focstim_rpc_Notification_notification_system_stats_tag;
        message.message.notification.notification.notification_system_stats.which_system = focstim_rpc_NotificationSystemStats_focstimv3_tag;
        message.message.notification.notification.notification_system_stats.system.focstimv3 = {
            .temp_stm32 = BSP_ReadTemperatureSTM(),
            .v_sys = BSP_ReadVSYS(),
            .v_ref = BSP_ReadChipAnalogVoltage(),
            .v_boost = BSP_ReadVBus(),
            .boost_duty_cycle = BSP_BoostDutyCycle(),
        };
        transmit_message(message);
    }

    Clock time_since_signal_start;
};

FocstimV3ProtobufAPI protobuf{};
ProtobufAPI* g_protobuf = &protobuf;


// TODO: implement
// struct {
//     BufferedAxis alpha{-1, 1, 0};
//     BufferedAxis beta{-1, 1, 0};
//     BufferedAxis gamma{-1, 1, 0};
//     BufferedAxis volume_percent{0, 1, 0};
// } new_axes;

struct {
    SimpleAxis alpha{focstim_rpc_AxisType_AXIS_POSITION_ALPHA, 0, -1, 1};
    SimpleAxis beta{focstim_rpc_AxisType_AXIS_POSITION_BETA, 0, -1, 1};
    SimpleAxis gamma{focstim_rpc_AxisType_AXIS_POSITION_GAMMA, 0, -1, 1};
    SimpleAxis waveform_amplitude_amps{focstim_rpc_AxisType_AXIS_WAVEFORM_AMPLITUDE_AMPS, 0, 0, BODY_CURRENT_MAX};
    SimpleAxis carrier_frequency{focstim_rpc_AxisType_AXIS_CARRIER_FREQUENCY_HZ, 800, 500, 2000};
    SimpleAxis pulse_frequency{focstim_rpc_AxisType_AXIS_PULSE_FREQUENCY_HZ, 50, 1, 100};
    SimpleAxis pulse_width{focstim_rpc_AxisType_AXIS_PULSE_WIDTH_IN_CYCLES, 6, 3, 20};
    SimpleAxis pulse_rise{focstim_rpc_AxisType_AXIS_PULSE_RISE_TIME_CYCLES, 5, 2, 10};
    SimpleAxis pulse_interval_random{focstim_rpc_AxisType_AXIS_PULSE_INTERVAL_RANDOM_PERCENT, 0, 0, 1};
    SimpleAxis calib_center{focstim_rpc_AxisType_AXIS_CALIBRATION_3_CENTER, 0, -10, 10};
    SimpleAxis calib_ud{focstim_rpc_AxisType_AXIS_CALIBRATION_3_UP, 0, -10, 10};
    SimpleAxis calib_lr{focstim_rpc_AxisType_AXIS_CALIBRATION_3_LEFT, 0, -10, 10};
    SimpleAxis calib_4_center{focstim_rpc_AxisType_AXIS_CALIBRATION_4_CENTER, 0, -10, 10};
    SimpleAxis calib_4a{focstim_rpc_AxisType_AXIS_CALIBRATION_4_A, 0, -10, 10};
    SimpleAxis calib_4b{focstim_rpc_AxisType_AXIS_CALIBRATION_4_B, 0, -10, 10};
    SimpleAxis calib_4c{focstim_rpc_AxisType_AXIS_CALIBRATION_4_C, 0, -10, 10};
    SimpleAxis calib_4d{focstim_rpc_AxisType_AXIS_CALIBRATION_4_D, 0, -10, 10};
} simple_axes;

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

AS5311 as5311{};

void setup()
{
    BSP_CheckJumpToBootloader();
    Serial.begin(115200, SERIAL_8E1);
    protobuf.init();
    protobuf.set_simple_axis(
        reinterpret_cast<SimpleAxis *>(&simple_axes),
        sizeof(simple_axes) / sizeof(SimpleAxis));
    BSP_PrintDebugMsg("BSP init");
    BSP_Init();
    BSP_WriteLedPattern(LedPattern::Idle);

    protobuf.transmit_notification_boot();

    power_manager.init();
    power_manager.adjust_board_voltages();

    BSP_SetBoostVoltage(STIM_BOOST_VOLTAGE);

    model3.init(&estop_triggered);
    model4.init(&estop_triggered);

    as5311.init(0.001f, 0.01f);
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
    static Clock adjust_board_voltage_clock;
    static Clock print_system_stats_clock;

    static Clock potentiometer_notification_nospam;
    static float potentiometer_notification_lastvalue = 0;

    // do comms
    protobuf.process_incoming_messages();

    // safety: temperature
    float temperature = BSP_ReadTemperatureSTM();
    if (temperature >= MAXIMUM_TEMPERATURE) {
        protobuf.signal_stop();
        while (1)
        {
            BSP_WriteLedPattern(LedPattern::Error);
            BSP_PrintDebugMsg(
                "temperature limit exceeded %.2f. Current temperature=%.2f. Restart device to proceed.",
                temperature, BSP_ReadTemperatureSTM());
            delay(5000);
        }
    }

    float vbus = BSP_ReadVBus();
    // safety: boost overvoltage
    if (vbus >= STIM_BOOST_OVERVOLTAGE_THRESHOLD) {
        protobuf.signal_stop();
        while (1)
        {
            BSP_WriteLedPattern(LedPattern::Error);
            BSP_PrintDebugMsg(
                "boost overvoltage detected %.2f. Current boost=%.2f. Restart device to proceed.",
                vbus, BSP_ReadVBus());
            delay(5000);
        }
    }

    // safety: boost undervoltage
    if (vbus <= STIM_BOOST_UNDERVOLTAGE_THRESHOLD) {
        if (play_status != PlayStatus::NotPlaying) {
            protobuf.time_since_signal_start.step();
            if (protobuf.time_since_signal_start.time_seconds >= 0.5f) {
                BSP_DisableOutputs();
                while (1)
                {
                    BSP_WriteLedPattern(LedPattern::Error);
                    BSP_PrintDebugMsg(
                        "boost undervoltage detected %.2f. Current boost=%.2f. Restart device to proceed.",
                        vbus, BSP_ReadVBus());
                        delay(5000);
                }
            }
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

    // adjust boost duty cycle
    adjust_board_voltage_clock.step();
    if (adjust_board_voltage_clock.time_seconds > 1) {
        adjust_board_voltage_clock.reset();
        power_manager.adjust_board_voltages();
    }

    // every few seconds, print system stats
    print_system_stats_clock.step();
    if (print_system_stats_clock.time_seconds > 5) {
        print_system_stats_clock.reset();
        protobuf.transmit_notification_system_stats();

        if (power_manager.is_battery_present) {
            float voltage = lipo.voltage() * .001f;
            float power_watt = lipo.power() * .001f;
            unsigned int fullCapacity = lipo.capacity(FULL); // Read full capacity (mAh)
            unsigned int capacity = lipo.capacity(REMAIN); // Read remaining capacity (mAh)
            float soc = float(capacity) / float(fullCapacity);
            float temperature = lipo.temperature(INTERNAL_TEMP) * 0.1f - 273.15f;
            protobuf.transmit_notification_battery(voltage, soc, power_watt, temperature, !BSP_ReadPGood());
        }
    }

    as5311.update();

    // DSTART / DSTOP
    if (play_status == PlayStatus::NotPlaying) {
        BSP_WriteLedPattern(LedPattern::Idle);
        delay(5);
        return;
    }

    // keepalive timer. Stop playing if no messages have been received for some time.
    if (protobuf.time_since_last_axis_command.time_seconds > 2) {
        BSP_PrintDebugMsg("Comms lost? Stopping.");
        play_status = PlayStatus::NotPlaying;
        BSP_WriteLedPattern(LedPattern::Idle);
        BSP_DisableOutputs();
        BSP_SetBoostEnable(false);
        return;
    }

    // stall out idle portion of the pulse
    total_pulse_length_timer.step();
    if (total_pulse_length_timer.time_seconds <= pulse_total_duration) {
        return;
    }

    // delay pulse until the boost capacitors are filled up, reducing the pulse frequency if neccesairy
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
    uint32_t now_ms = millis();
    float pulse_alpha = simple_axes.alpha.get(now_ms);
    float pulse_beta = simple_axes.beta.get(now_ms);
    float pulse_gamma = simple_axes.gamma.get(now_ms);
    float body_current_amps = simple_axes.waveform_amplitude_amps.get(now_ms);

    float pulse_carrier_frequency = simple_axes.carrier_frequency.get(now_ms);
    float pulse_frequency = simple_axes.pulse_frequency.get(now_ms);
    float pulse_width = simple_axes.pulse_width.get(now_ms);
    float pulse_rise = simple_axes.pulse_rise.get(now_ms);
    float pulse_interval_random = simple_axes.pulse_interval_random.get(now_ms);

    float calibration_center = simple_axes.calib_center.get(now_ms);
    float calibration_lr = simple_axes.calib_lr.get(now_ms);
    float calibration_ud = simple_axes.calib_ud.get(now_ms);

    float calibration_4a = simple_axes.calib_4a.get(now_ms);
    float calibration_4b = simple_axes.calib_4b.get(now_ms);
    float calibration_4c = simple_axes.calib_4c.get(now_ms);
    float calibration_4d = simple_axes.calib_4d.get(now_ms);

    float pulse_active_duration = pulse_width / pulse_carrier_frequency;
    float pulse_pause_duration = max(0.f, 1 / pulse_frequency - pulse_active_duration);
    pulse_pause_duration *= float_rand(1 - pulse_interval_random, 1 + pulse_interval_random);
    pulse_total_duration = pulse_active_duration + pulse_pause_duration;

    // mix in potmeter
    float potmeter_value = BSP_ReadPotentiometerPercentage();
    potmeter_value = powf(potmeter_value, 1.f/2);
    body_current_amps *= potmeter_value;

    // calculate amplitude in amperes (driving current)
    float driving_current_amps = body_current_amps * STIM_WINDING_RATIO;

    float volume_percent = body_current_amps / BODY_CURRENT_MAX;
    if (volume_percent < .02f) {
        BSP_WriteLedPattern(LedPattern::PlayingVeryLow);
    } else if (volume_percent < .2) {
        BSP_WriteLedPattern(LedPattern::PlayingLow);
    } else if (volume_percent < .6) {
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
    traceline->i_max_cmd = driving_current_amps;
    total_pulse_length_timer.step();
    traceline->dt_compute = total_pulse_length_timer.dt_micros;

    // play the pulse
    if (play_status == PlayStatus::PlayingThreephase) {
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
    } else if (play_status == PlayStatus::PlayingFourphase) {
        ComplexFourphasePoints points4 = project_fourphase(
            driving_current_amps,
            pulse_alpha, pulse_beta, pulse_gamma,
            calibration_center,
            calibration_4a, calibration_4b, calibration_4c, calibration_4d,
            polarity,
            random_start_angle);

        model4.play_pulse(points4.p1, points4.p2, points4.p3, points4.p4,
            pulse_carrier_frequency,
            pulse_width, pulse_rise,
            driving_current_amps + ESTOP_CURRENT_LIMIT_MARGIN);
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

    // send notification: pulse current
    if (pulse_counter % 50 == 0) {
        rms_current_clock.step();
        if (play_status == PlayStatus::PlayingThreephase) {
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
        if (play_status == PlayStatus::PlayingFourphase) {
            auto rms = model4.estimate_rms_current(rms_current_clock.dt_seconds);
            auto r_a = model4.z1.a;
            auto r_b = model4.z2.a;
            auto r_c = model4.z3.a;
            auto r_d = model4.z4.a;
            float p =
                (rms.a * rms.a) * r_a +
                (rms.b * rms.b) * r_b +
                (rms.c * rms.c) * r_c;

            protobuf.transmit_notification_currents(
                rms.a / STIM_WINDING_RATIO,
                rms.b / STIM_WINDING_RATIO,
                rms.c / STIM_WINDING_RATIO,
                0,
                abs(model4.current_max.a) / STIM_WINDING_RATIO,
                abs(model4.current_max.b) / STIM_WINDING_RATIO,
                abs(model4.current_max.c) / STIM_WINDING_RATIO,
                0,
                p, 0,
                abs(driving_current_amps) / STIM_WINDING_RATIO
            );
            model4.current_max = {};
            model4.current_squared = {};
        }
    }

    // send notification: skin resistance estimation
    if (pulse_counter % 50 == 20) {
        float m = STIM_WINDING_RATIO_SQ;
        if (play_status == PlayStatus::PlayingThreephase) {
            protobuf.transmit_notification_model_estimation(
                model3.z1.a * m, model3.z1.b * m,
                model3.z2.a * m, model3.z2.b * m,
                model3.z3.a * m, model3.z3.b * m,
                0, 0);
        }
        if (play_status == PlayStatus::PlayingFourphase) {
            protobuf.transmit_notification_model_estimation(
                model4.z1.a * m, model4.z1.b * m,
                model4.z2.a * m, model4.z2.b * m,
                model4.z3.a * m, model4.z3.b * m,
                model4.z4.a * m, model4.z4.b * m);
        }
    }

    // send notification: pulse stats
    if (pulse_counter % 50 == 40) {
        float v_drive = 0;
        if (play_status == PlayStatus::PlayingThreephase) {
            v_drive = model3.v_drive_max;
            model3.v_drive_max = 0;
        }
        if (play_status == PlayStatus::PlayingFourphase) {
            v_drive = model4.v_drive_max;
            model4.v_drive_max = 0;
        }

        // transmit_notification
        protobuf.transmit_notification_signal_stats(actual_pulse_frequency, v_drive);
    }

    // store stats
    total_pulse_length_timer.step();
    traceline->dt_logs = total_pulse_length_timer.dt_micros;
}

#endif