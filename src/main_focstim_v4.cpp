#if defined(BOARD_FOCSTIM_V4)

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
#include "battery/power_manager.h"
#include <Wire.h>
#include "axis/buffered_axis.h"
#include "axis/simple_axis.h"
#include "timestamp_sync.h"
#include "sensors/as5311.h"
#include "user_interface/user_interface.h"
#include "esp32.h"
#include "foc_error.h"

#include "bsp/bsp.h"
#include "bsp/bootloader.h"

#include "focstim_rpc.pb.h"
#include "protobuf_api.h"


Trace trace{};
ThreephaseModel model3{};
FourphaseModel model4{};
PowerManager power_manager{};
UserInterface user_interface{};
ESP32 esp32;

enum PlayStatus{
    NotPlaying,
    PlayingThreephase,
    PlayingFourphase
};
static PlayStatus play_status = PlayStatus::NotPlaying;


class FocstimV4ProtobufAPI : public ProtobufAPI {
public:
    FocstimV4ProtobufAPI() {};

    focstim_rpc_Errors wifi_parameters_set(focstim_rpc_RequestWifiParametersSet &params) {
        // TODO: error checks
        esp32.set_wifi_ssid(params.ssid.bytes, params.ssid.size);
        esp32.set_wifi_password(params.password.bytes, params.password.size);
        esp32.wifi_reconnect();

        return focstim_rpc_Errors_ERROR_UNKNOWN;
    }

    focstim_rpc_Errors wifi_ip_get(focstim_rpc_RequestWifiIPGet& params, uint32_t *ip_out) {
        // TOOD: error checks
        *ip_out = esp32.ip();
        return focstim_rpc_Errors_ERROR_UNKNOWN;
    }

    focstim_rpc_Errors signal_start_threephase()
    {
        if (play_status != PlayStatus::NotPlaying) {
            return focstim_rpc_Errors_ERROR_ALREADY_PLAYING;
        }
        // transmit_notification_debug_string("START 3");

        time_since_signal_start.reset();
        BSP_SetBoostEnable(true);
        BSP_WriteLedPattern(LedPattern::PlayingVeryLow);
        play_status = PlayStatus::PlayingThreephase;
        user_interface.setState(UserInterface::Playing);
        return focstim_rpc_Errors_ERROR_UNKNOWN;
    }

    focstim_rpc_Errors signal_start_fourphase()
    {
        if (play_status != PlayStatus::NotPlaying) {
            return focstim_rpc_Errors_ERROR_ALREADY_PLAYING;
        }

        // transmit_notification_debug_string("START 4");

        time_since_signal_start.reset();
        BSP_SetBoostEnable(true);
        BSP_WriteLedPattern(LedPattern::PlayingVeryLow);
        play_status = PlayStatus::PlayingFourphase;
        user_interface.setState(UserInterface::Playing);
        return focstim_rpc_Errors_ERROR_UNKNOWN;
    }

    void signal_stop()
    {
        // transmit_notification_debug_string("STOP");

        BSP_SetBoostEnable(false);
        BSP_WriteLedPattern(LedPattern::Idle);
        user_interface.setState(UserInterface::Idle);   // TOOD: force display update
        play_status = PlayStatus::NotPlaying;
    }

    virtual bool capability_threephase() {return true;};
    virtual bool capability_fourphase() {return true;};
    virtual bool capability_potmeter() {return true;};
    virtual bool capability_battery() {return power_manager.is_battery_present;};

    void transmit_notification_system_stats(float v_boost_min, float v_boost_max) {
        focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
        message.which_message = focstim_rpc_RpcMessage_notification_tag;
        message.message.notification.which_notification = focstim_rpc_Notification_notification_system_stats_tag;
        message.message.notification.notification.notification_system_stats.which_system = focstim_rpc_NotificationSystemStats_focstimv3_tag;
        Vec2f vsys_range = BSP_ReadVSYSRange();
        message.message.notification.notification.notification_system_stats.system.focstimv3 = {
            .temp_stm32 = BSP_ReadTemperatureSTM(),
            .v_sys_min = vsys_range.a,
            .v_ref = BSP_ReadChipAnalogVoltage(),
            .v_boost_min = v_boost_min,
            .boost_duty_cycle = BSP_BoostDutyCycle(),
            .v_sys_max = vsys_range.b,
            .v_boost_max = v_boost_max,
        };
        transmit_message(message);
    }

    Clock time_since_signal_start;
};

FocstimV4ProtobufAPI protobuf{};
ProtobufAPI* g_protobuf = &protobuf;

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

void trigger_emergency_stop(FOCError error)
{
    BSP_DisableOutputs();
    BSP_SetBoostEnable(false);
    BSP_WriteLedPattern(LedPattern::Error);

    switch (error) {
        case OUTPUT_OVER_CURRENT:
        case MODEL_TIMING_ERROR:
        trace.print_mainloop_trace();
        if (play_status == PlayStatus::PlayingThreephase) {
            model3.debug_stats_teleplot();
        }
        if (play_status == PlayStatus::PlayingFourphase) {
            model4.debug_stats_teleplot();
        }
        break;
        case BOOST_VOLTAGE_NOT_RISING:
        case BOOST_UNDER_VOLTAGE:
        case BOOST_OVER_VOLTAGE:
        case BOARD_OVER_TEMPERATURE:
        break;
    }

    play_status = PlayStatus::NotPlaying;
    user_interface.setState(UserInterface::Error);
    user_interface.repaint();
    user_interface.full_update();
}

AS5311 as5311{};

void self_test() {
    auto check_value = [](bool passfail, const char* test_string) {
        if (passfail) {
            // pass
            BSP_PrintDebugMsg("PASS: %s", test_string);
        } else {
            // fail
            BSP_PrintDebugMsg("FAIL: %s", test_string);

            user_interface.setState(UserInterface::SelfTestError);
            user_interface.setSelfTestErrorString(test_string);
            user_interface.repaint();
            user_interface.full_update();

            while (1) {
                delay(1000);
                BSP_PrintDebugMsg("FAIL: %s", test_string);
            }
        }
    };

    auto is_within = [](float value, float minimum, float maximum) {
        return (value >= minimum) && (value <= maximum);
    };

    auto current_draw = [](float v1, float v2, float dt) {
        float v_avg = (v1 + v2) / 2;
        float capacitance = 230e-6;
        float energy = 0.5f * capacitance * (v1 * v1 - v2 * v2);    // joules = 0.5 * capacitance * voltage^2
        float watt = energy / dt;
        return watt / v_avg;
    };

    char buffer[255];

    BSP_PrintDebugMsg("self test starting");

    // check if board temperature is in valid range
    {
        float temperature = BSP_ReadTemperatureSTM();
        float minimum = 1.0f;
        float maximum = MAXIMUM_TEMPERATURE;
        snprintf(buffer, sizeof(buffer), "stm32 temperature %.2f deg C (valid range %.0f to %.0f)", temperature, minimum, maximum);
        check_value(is_within(temperature, minimum, maximum), buffer);
    }

    // check if board analog value is in valid range
    {
        float temperature = BSP_ReadChipAnalogVoltage();
        float minimum = 2.9f - 0.05f;
        float maximum = 2.9f + 0.05f;
        snprintf(buffer, sizeof(buffer), "stm32 analog voltage %.2fV (valid range %.2fV to %.2fV)", temperature, minimum, maximum);
        check_value(is_within(temperature, minimum, maximum), buffer);
    }

    // Check if VSYS is in valid range
    {
        float vsys = BSP_ReadVSYS();
        float minimum = 3.0f;
        float maximum = 5.4f;
        snprintf(buffer, sizeof(buffer), "vsys %.2fV (valid range %.2fV to %.2fV)", vsys, minimum, maximum);
        check_value(is_within(vsys, minimum, maximum), buffer);
    }

    // check if boost caps are in valid range
    {
        float vbus = BSP_ReadVBus();
        float minimum = 3.0f;
        float maximum = 30.0f;
        snprintf(buffer, sizeof(buffer), "vbus %.2fV (valid range %.2fV to %.2fV)", vbus, minimum, maximum);
        check_value(is_within(vbus, minimum, maximum), buffer);
    }

    // discharge the boost caps to <= 6v
    {
        // enable drivers for faster draining
        BSP_SetPWM4Atomic(0, 0, 0, 0);
        BSP_OutputEnable(true, true, true, true);

        Clock k;
        float vbus_initial = BSP_ReadVBus();
        float target = 6.0f;
        float maximum_time = 1.0f;  // draiting from the maximum voltage of 30V takes 600ms
        float vbus = vbus_initial;
        while ((vbus >= target) && (k.time_seconds <= maximum_time)) {
            vbus = BSP_ReadVBus();
            k.step();
            delay(1);
        }
        bool passfail = k.time_seconds <= maximum_time;
        BSP_DisableOutputs();

        snprintf(buffer, sizeof(buffer), "drain vbus from %.2fV to %.2fV in %.0fms (target < %.2fV)", vbus_initial, vbus, k.time_seconds * 1000, target);
        check_value(passfail, buffer);
    }

    // charge the boost caps to 10V
    {
        Clock k;
        float vbus_initial = BSP_ReadVBus();
        float target = 10.0f;
        float minimum = 9.5f;
        float maximum = 10.5f;
        float maximum_time = 0.05f;
        BSP_SetBoostVoltage(target);
        BSP_SetBoostEnable(true);
        delay(int(maximum_time * 1000));
        float vbus = BSP_ReadVBus();

        snprintf(buffer, sizeof(buffer), "charge vbus to %.2fV (vbus: %.2f, valid range: %.2fV to %.2fV)", target, vbus, minimum, maximum);
        check_value(is_within(vbus, minimum, maximum), buffer);
    }

    // measure the current draw from boost caps
    // with various combinations of enabled drivers
    // should be able to detect shorted or malfunctioning drivers.
    {
        auto test_current_draw = [&](float voltage, float dt, float minimum_draw, float maximum_draw, Vec4f driver_state, float pwm, const char* test_name) {
            BSP_OutputEnable(bool(driver_state.a), bool(driver_state.b), bool(driver_state.c), bool(driver_state.d));
            BSP_SetPWM4Atomic(pwm, pwm, pwm, pwm);
            BSP_SetBoostVoltage(voltage);
            BSP_SetBoostEnable(true);
            delay(20);  // charge
            BSP_SetBoostEnable(false);
            float v1 = BSP_ReadVBus();
            delay(int(dt * 1000)); // discharge
            float v2 = BSP_ReadVBus();
            BSP_DisableOutputs();
            BSP_SetPWM4(0, 0, 0, 0);
            float current = current_draw(v1, v2, dt);

            snprintf(buffer, sizeof(buffer), "%s draw %5.2fmA (valid range %.1fmA to %.1fmA)", test_name, current*1000, minimum_draw*1000, maximum_draw*1000);
            check_value(is_within(current, minimum_draw, maximum_draw), buffer);
        };

        float mA = 1e-3f;
        float uA = 1e-6f;
        // 'real' value here should be ~100µA, but not very accurate because of short test duration.
        test_current_draw(10, 0.20f, -100 * uA, 400 * uA,  {0, 0, 0, 0}, 0,  "drv OFF");
        test_current_draw(10, 0.05f, 1.5f * mA, 3 * mA,    {1, 0, 0, 0}, 0,  "drv A");
        test_current_draw(10, 0.05f, 1.5f * mA, 3 * mA,    {0, 1, 0, 0}, 0,  "drv B");
        test_current_draw(10, 0.05f, 1.5f * mA, 3 * mA,    {0, 0, 1, 0}, 0,  "drv C");
        test_current_draw(10, 0.05f, 1.5f * mA, 3 * mA,    {0, 0, 0, 1}, 0,  "drv D");
        test_current_draw(10, 0.02f, 7 * mA, 12 * mA,      {1, 1, 1, 1}, 0,  "drv ABCD/0%");
        test_current_draw(10, 0.02f, 7 * mA, 12 * mA,      {1, 1, 1, 1}, .5, "drv ABCD/50%");
    }

    BSP_PrintDebugMsg("self test completed");
}


void setup()
{
    BSP_CheckJumpToBootloader();
    Serial.begin(115200, SERIAL_8E1);   // match STM32 bootloader setting
    Wire.setSCL(PA15);
    Wire.setSDA(PB9);

    user_interface.init();
    user_interface.setState(UserInterface::InitBSP);
    user_interface.repaint();
    user_interface.full_update();

    protobuf.init();
    protobuf.set_simple_axis(
        reinterpret_cast<SimpleAxis *>(&simple_axes),
        sizeof(simple_axes) / sizeof(SimpleAxis));
    BSP_PrintDebugMsg("BSP init");
    BSP_Init();
    BSP_WriteLedPattern(LedPattern::Idle);
    protobuf.transmit_notification_boot();
    user_interface.repaint();
    user_interface.full_update();

    user_interface.setState(UserInterface::InitBattery);
    user_interface.repaint();
    user_interface.full_update();

    power_manager.init();
    user_interface.setBatteryPresent(power_manager.is_battery_present);
    if (power_manager.is_battery_present) {
        user_interface.setBatterySoc(power_manager.cached_soc());
    }
    esp32.init();

    user_interface.setState(UserInterface::InitSelfTest);
    user_interface.repaint();
    user_interface.full_update();

    self_test();

    user_interface.setState(UserInterface::Idle);
    user_interface.repaint();
    user_interface.full_update();

    // TODO: dynamic
    BSP_SetBoostVoltage(STIM_BOOST_VOLTAGE);

    model3.init(&trigger_emergency_stop);
    model4.init(&trigger_emergency_stop);

    // as5311.init(0.001f, 0.01f);
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
    static Clock print_system_stats_clock;
    static float v_drive_max = 0;
    static float v_boost_min = 99;
    static float v_boost_max = 0;
    static Clock ip_refresh_clock;

    static Clock potentiometer_notification_nospam;
    static float potentiometer_notification_lastvalue = 0;

    // do comms
    protobuf.process_incoming_messages();

    // safety: temperature
    float temperature = BSP_ReadTemperatureSTM();
    if (temperature >= MAXIMUM_TEMPERATURE) {
        trigger_emergency_stop(FOCError::BOARD_OVER_TEMPERATURE);
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
    v_boost_min = min(v_boost_min, vbus);
    v_boost_max = max(v_boost_max, vbus);
    // safety: boost overvoltage
    if (vbus >= STIM_BOOST_OVERVOLTAGE_THRESHOLD) {
        trigger_emergency_stop(FOCError::BOOST_OVER_VOLTAGE);
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
                trigger_emergency_stop(FOCError::BOOST_UNDER_VOLTAGE);
                trace.print_mainloop_trace();
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

        user_interface.setPowerLevel(powf(pot, 1.f/2) * 100);
    }

    // every few seconds, print system stats
    print_system_stats_clock.step();
    if (print_system_stats_clock.time_seconds > .5) {
        print_system_stats_clock.reset();
        protobuf.transmit_notification_system_stats(v_boost_min, v_boost_max);
        v_boost_min = 99;
        v_boost_max = 0;

        if (power_manager.is_battery_present) {
            protobuf.transmit_notification_battery(
                power_manager.cached_voltage,
                power_manager.cached_soc(),
                power_manager.cached_power,
                power_manager.cached_temperature,
                !BSP_ReadPGood());
            user_interface.setBatterySoc(power_manager.cached_soc());
        }
    }

    // if not playing, regularly query the IP address from the esp32
    ip_refresh_clock.step();
    if (ip_refresh_clock.time_seconds > 0.4f && play_status == PlayStatus::NotPlaying) {
        user_interface.setIP(esp32.ip());
        ip_refresh_clock.reset();
    }

    as5311.update();
    power_manager.update();

    // update small slice of the display
    user_interface.repaint();
    user_interface.partial_update();

    // start/stop
    if (play_status == PlayStatus::NotPlaying) {
        BSP_WriteLedPattern(LedPattern::Idle);
        return;
    }

    // keepalive timer. Stop playing if no messages have been received for some time.
    if (protobuf.time_since_last_axis_command.time_seconds > 4) {
        BSP_PrintDebugMsg("Comms lost? Stopping.");
        play_status = PlayStatus::NotPlaying;
        BSP_WriteLedPattern(LedPattern::Idle);
        user_interface.setState(UserInterface::Idle);
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
        if (protobuf.time_since_signal_start.time_seconds >= 0.5f) {
            BSP_PrintDebugMsg("boost too low: %u %f %f", micros(), vbus, BSP_ReadVSYS());
        }
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
    traceline->dt_next = pulse_total_duration * 1e6f;

    // mix in potmeter
    float potmeter_value = BSP_ReadPotentiometerPercentage();
    potmeter_value = powf(potmeter_value, 1.f/2);
    body_current_amps *= potmeter_value;

    // calculate amplitude in amperes (driving current)
    float driving_current_amps = body_current_amps * STIM_WINDING_RATIO;

    float volume_percent = body_current_amps / BODY_CURRENT_MAX;
    // TODO: remove
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

        BSP_OutputEnable(true, true, true);
        delayMicroseconds(300); // the minimum of DRV8231A turnon time (datasheet: 250µs) and triac turnon time (experimental: 300µs)
        BSP_AdjustCurrentSenseOffsets();

        model3.play_pulse(points3.p1, points3.p2, points3.p3,
            pulse_carrier_frequency,
            pulse_width, pulse_rise,
            driving_current_amps + ESTOP_CURRENT_LIMIT_MARGIN);

        BSP_DisableOutputs();
    } else if (play_status == PlayStatus::PlayingFourphase) {
        ComplexFourphasePoints points4 = project_fourphase(
            driving_current_amps,
            pulse_alpha, pulse_beta, pulse_gamma,
            calibration_center,
            calibration_4a, calibration_4b, calibration_4c, calibration_4d,
            polarity,
            random_start_angle);

        BSP_OutputEnable(true, true, true, true);
        delayMicroseconds(300); // the minimum of DRV8231A turnon time (datasheet: 250µs) and triac turnon time (experimental: 300µs)
        BSP_AdjustCurrentSenseOffsets();

        model4.play_pulse(points4.p1, points4.p2, points4.p3, points4.p4,
            pulse_carrier_frequency,
            pulse_width, pulse_rise,
            driving_current_amps + ESTOP_CURRENT_LIMIT_MARGIN);

        BSP_DisableOutputs();
    }

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
            traceline->v_drive = model3.v_drive_last;
            v_drive_max = max(v_drive_max, model3.v_drive_last);

            auto current_max = model3.current_max;
            traceline->i_max_a = current_max.a;
            traceline->i_max_b = current_max.b;
            traceline->i_max_c = current_max.c;
            traceline->i_max_d = 0;

            traceline->v_boost_min = model3.v_bus_min;
            traceline->v_boost_max = model3.v_bus_max;
            v_boost_min = min(v_boost_min, model3.v_bus_min);
            v_boost_max = max(v_boost_max, model3.v_bus_max);

        } else {
            traceline->skipped_update_steps = model4.skipped_update_steps;
            traceline->v_drive = model4.v_drive_last;
            v_drive_max = max(v_drive_max, model4.v_drive_last);

            auto current_max = model4.current_max;
            traceline->i_max_a = current_max.a;
            traceline->i_max_b = current_max.b;
            traceline->i_max_c = current_max.c;
            traceline->i_max_d = current_max.d;

            traceline->v_boost_min = model4.v_bus_min;
            traceline->v_boost_max = model4.v_bus_max;
            v_boost_min = min(v_boost_min, model4.v_bus_min);
            v_boost_max = max(v_boost_max, model4.v_bus_max);
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
                rms.d / STIM_WINDING_RATIO,
                abs(model4.current_max.a) / STIM_WINDING_RATIO,
                abs(model4.current_max.b) / STIM_WINDING_RATIO,
                abs(model4.current_max.c) / STIM_WINDING_RATIO,
                abs(model4.current_max.d) / STIM_WINDING_RATIO,
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
        // transmit_notification
        protobuf.transmit_notification_signal_stats(actual_pulse_frequency, v_drive_max);
        v_drive_max = 0;
    }
}

#endif