#ifndef FOCSTIM_PROTOBUF_API_H
#define FOCSTIM_PROTOBUF_API_H

#include "focstim_rpc.pb.h"
#include "axis/simple_axis.h"
#include "timestamp_sync.h"
#include "stim_clock.h"

class ProtobufAPI;

extern ProtobufAPI *g_protobuf;

class ProtobufAPI {
public:
    ProtobufAPI();
    virtual ~ProtobufAPI() {};

    void init();
    void set_simple_axis(SimpleAxis *axis, int num_axis);
    // TODO: buffered axis

    void process_incoming_messages();

    void transmit_message(focstim_rpc_RpcMessage &message);
    void transmit_notification(focstim_rpc_Notification &notification);

    void transmit_notification_boot();
    void transmit_notification_potentiometer(float value);
    void transmit_notification_currents(
        float rms_a, float rms_b, float rms_c, float rms_d,
        float peak_a, float peak_b, float peak_c, float peak_d,
        float output_power, float output_power_skin,
        float peak_cmd);
    void transmit_notification_model_estimation(
        float resistance_a, float reluctance_a,
        float resistance_b, float reluctance_b,
        float resistance_c, float reluctance_c,
        float resistance_d, float reluctance_d);
    void transmit_notification_signal_stats(float actual_pulse_frequency, float v_drive);
    void transmit_notification_battery(
        float voltage, float soc, float charge_rate,
        float temperature, bool usb5v_present);

    void transmit_error_response(focstim_rpc_Errors errorcode, uint32_t id);

    void transmit_notification_debug_string(const char* fmt, ...);
    void transmit_notification_debug_string(const char* fmt, va_list args);

    void handle_request_firmware_version(focstim_rpc_RequestFirmwareVersion &request, uint32_t id);
    // void handle_request_axis_set(focstim_rpc_RequestAxisSet &request, uint32_t id);
    void handle_request_axis_move_to(focstim_rpc_RequestAxisMoveTo &request, uint32_t id);
    void handle_request_timestamp_set(focstim_rpc_RequestTimestampSet &request, uint32_t id);
    void handle_request_timestamp_get(focstim_rpc_RequestTimestampGet &request, uint32_t id);
    void handle_request_signal_start(focstim_rpc_RequestSignalStart &request, uint32_t id);
    void handle_request_signal_stop(focstim_rpc_RequestSignalStop &request, uint32_t id);
    void handle_request_capabilities_get(focstim_rpc_RequestCapabilitiesGet &request, uint32_t id);

    void handle_request_debug_stm32_deep_sleep(focstim_rpc_RequestDebugStm32DeepSleep &request, uint32_t id);
    void handle_request_debug_enter_bootloader(focstim_rpc_RequestDebugEnterBootloader &request, uint32_t id);

    void handle_request(focstim_rpc_Request &request);
    void handle_frame(const uint8_t* data, size_t data_len);

    // override me!
    virtual focstim_rpc_Errors signal_start_threephase() {return focstim_rpc_Errors_ERROR_OUTPUT_NOT_SUPPORTED;}
    virtual focstim_rpc_Errors signal_start_fourphase() {return focstim_rpc_Errors_ERROR_OUTPUT_NOT_SUPPORTED;}
    virtual void signal_stop() {}

    virtual bool capability_threephase() = 0;
    virtual bool capability_fourphase() = 0;
    virtual bool capability_potmeter() = 0;
    virtual bool capability_battery() = 0;


    SimpleAxis *simple;
    int simple_num;

    TimestampSync time_sync;
    Clock time_since_last_axis_command;
};

#endif