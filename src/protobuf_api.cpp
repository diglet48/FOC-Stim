#include "protobuf_api.h"

#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "focstim_rpc.pb.h"

#include "HDLC.h"
#include "CRC16_CCITT.h"

#include <Arduino.h>
#include <cstdint>

#include "bsp/bsp.h"


int16_t read_byte() {
    return Serial.read();
};
void write_byte(uint8_t ch) {
    Serial.write(ch);
};
HDLC<read_byte, write_byte, 32, CRC16_CCITT> hdlc;


static bool encode_string(pb_ostream_t* stream, const pb_field_t* field, void* const* arg)
{
    if (!pb_encode_tag_for_field(stream, field))
    return false;

    const char* str = (const char *)(*arg);
    return pb_encode_string(stream, (const uint8_t*)str, strlen(str));
}


ProtobufAPI::ProtobufAPI()
{
}

void ProtobufAPI::init()
{
    time_sync.init();
}

void ProtobufAPI::set_simple_axis(SimpleAxis *axis, int num_axis)
{
    simple = axis;
    simple_num = num_axis;
}

void ProtobufAPI::process_incoming_messages() {
    time_sync.step();
    time_since_last_axis_command.step();
    while (Serial.available()) {
        int retv = hdlc.receive();
        if (retv > 0) {
            handle_frame(hdlc.data, hdlc.len);
        }
    }
}

void ProtobufAPI::transmit_message(focstim_rpc_RpcMessage &message)
{
    uint8_t out_buffer[256];
    pb_ostream_t oStream = pb_ostream_from_buffer(out_buffer, 256);
    if (!pb_encode(&oStream, &focstim_rpc_RpcMessage_msg, &message))
    {
        const char *error = PB_GET_ERROR(&oStream);
        BSP_PrintDebugMsg("pb_encode error: %s\n", error);
    } else {
        size_t total_bytes_encoded = oStream.bytes_written;
        // printf("Encoded size: %ld\n", total_bytes_encoded);
        hdlc.transmitBlock(out_buffer, oStream.bytes_written);
    }
}

void ProtobufAPI::transmit_notification(focstim_rpc_Notification &notification)
{
    focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
    message.which_message = focstim_rpc_RpcMessage_notification_tag;
    message.message.notification = notification;
    transmit_message(message);
}

void ProtobufAPI::transmit_notification_boot()
{
    focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
    message.which_message = focstim_rpc_RpcMessage_notification_tag;
    message.message.notification.which_notification = focstim_rpc_Notification_notification_boot_tag;
    transmit_message(message);
};

void ProtobufAPI::transmit_notification_potentiometer(float value)
{
    focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
    message.which_message = focstim_rpc_RpcMessage_notification_tag;
    message.message.notification.which_notification = focstim_rpc_Notification_notification_potentiometer_tag;
    message.message.notification.notification.notification_potentiometer.value = value;
    transmit_message(message);
};

void ProtobufAPI::transmit_notification_currents(
    float rms_a, float rms_b, float rms_c, float rms_d,
    float peak_a, float peak_b, float peak_c, float peak_d,
    float output_power, float output_power_skin,
    float peak_cmd)
{
    focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
    message.which_message = focstim_rpc_RpcMessage_notification_tag;
    message.message.notification.which_notification = focstim_rpc_Notification_notification_currents_tag;
    message.message.notification.notification.notification_currents.rms_a = rms_a;
    message.message.notification.notification.notification_currents.rms_b = rms_b;
    message.message.notification.notification.notification_currents.rms_c = rms_c;
    message.message.notification.notification.notification_currents.rms_d = rms_d;
    message.message.notification.notification.notification_currents.peak_a = peak_a;
    message.message.notification.notification.notification_currents.peak_b = peak_b;
    message.message.notification.notification.notification_currents.peak_c = peak_c;
    message.message.notification.notification.notification_currents.peak_d = peak_d;
    message.message.notification.notification.notification_currents.output_power = output_power;
    message.message.notification.notification.notification_currents.output_power_skin = output_power_skin;
    message.message.notification.notification.notification_currents.peak_cmd = peak_cmd;
    transmit_message(message);
}

void ProtobufAPI::transmit_notification_model_estimation(
    float resistance_a, float reluctance_a,
    float resistance_b, float reluctance_b,
    float resistance_c, float reluctance_c,
    float resistance_d, float reluctance_d)
{
    focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
    message.which_message = focstim_rpc_RpcMessage_notification_tag;
    message.message.notification.which_notification = focstim_rpc_Notification_notification_model_estimation_tag;
    message.message.notification.notification.notification_model_estimation.resistance_a = resistance_a;
    message.message.notification.notification.notification_model_estimation.reluctance_a = reluctance_a;
    message.message.notification.notification.notification_model_estimation.resistance_b = resistance_b;
    message.message.notification.notification.notification_model_estimation.reluctance_b = reluctance_b;
    message.message.notification.notification.notification_model_estimation.resistance_c = resistance_c;
    message.message.notification.notification.notification_model_estimation.reluctance_c = reluctance_c;
    message.message.notification.notification.notification_model_estimation.resistance_d = resistance_d;
    message.message.notification.notification.notification_model_estimation.reluctance_d = reluctance_d;
    transmit_message(message);
}

void ProtobufAPI::transmit_notification_signal_stats(float actual_pulse_frequency, float v_drive) {
    focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
    message.which_message = focstim_rpc_RpcMessage_notification_tag;
    message.message.notification.which_notification = focstim_rpc_Notification_notification_signal_stats_tag;
    message.message.notification.notification.notification_signal_stats.actual_pulse_frequency = actual_pulse_frequency;
    message.message.notification.notification.notification_signal_stats.v_drive = v_drive;
    transmit_message(message);
}

void ProtobufAPI::transmit_notification_battery(
        float voltage, float soc, float charge_rate,
        float temperature, bool usb5v_present)
{
    focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
    message.which_message = focstim_rpc_RpcMessage_notification_tag;
    message.message.notification.which_notification = focstim_rpc_Notification_notification_battery_tag;
    message.message.notification.notification.notification_battery.battery_voltage = voltage,
    message.message.notification.notification.notification_battery.battery_soc = soc;
    message.message.notification.notification.notification_battery.battery_charge_rate_watt = charge_rate;
    message.message.notification.notification.notification_battery.wall_power_present = usb5v_present;
    message.message.notification.notification.notification_battery.chip_temperature = temperature;
    transmit_message(message);
}

void ProtobufAPI::transmit_error_response(focstim_rpc_Errors errorcode, uint32_t id)
{
    focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
    message.which_message = focstim_rpc_RpcMessage_response_tag;
    message.message.response.id = id;
    message.message.response.has_error = true;
    message.message.response.error.code = errorcode;
    transmit_message(message);
}

void ProtobufAPI::transmit_notification_debug_string(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    transmit_notification_debug_string(fmt, args);
    va_end(args);
}

void ProtobufAPI::transmit_notification_debug_string(const char *fmt, va_list args)
{
    char buffer[512];
    int rc = vsnprintf(buffer, sizeof(buffer), fmt, args);

    focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
    message.which_message = focstim_rpc_RpcMessage_notification_tag;
    message.message.notification.which_notification = focstim_rpc_Notification_notification_debug_string_tag;
    message.message.notification.notification.notification_debug_string.message.arg = (char* )buffer;
    message.message.notification.notification.notification_debug_string.message.funcs.encode = encode_string;
    transmit_message(message);
}

void ProtobufAPI::handle_request_firmware_version(focstim_rpc_RequestFirmwareVersion &request, uint32_t id)
{
    const char* FIRMWARE_VERSION = "1.0";

    focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
    message.which_message = focstim_rpc_RpcMessage_response_tag;
    message.message.response.id = id;
    message.message.response.which_result = focstim_rpc_Response_response_firmware_version_tag;
    message.message.response.result.response_firmware_version.board = focstim_rpc_BoardIdentifier_BOARD_FOCSTIM_V3;
    message.message.response.result.response_firmware_version.stm32_firmware_version.arg = (char*)FIRMWARE_VERSION;
    message.message.response.result.response_firmware_version.stm32_firmware_version.funcs.encode = encode_string;
    transmit_message(message);
}

void ProtobufAPI::handle_request_axis_move_to(focstim_rpc_RequestAxisMoveTo &request, uint32_t id) {
    float value = request.value;
    uint32_t interval = request.interval;
    uint32_t now = millis();

    for (int i = 0; i < simple_num; i++) {
        SimpleAxis* target = simple + i;
        if (target->id == request.axis) {
            target->move_to(now, value, interval);
        }
    }
    // todo: error out if not found?

    time_since_last_axis_command.reset();

    focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
    message.which_message = focstim_rpc_RpcMessage_response_tag;
    message.message.response.id = id;
    message.message.response.which_result = focstim_rpc_Response_response_axis_move_to_tag;
    transmit_message(message);
}

// void ProtobufAPI::handle_request_axis_set(focstim_rpc_RequestAxisSet &request, uint32_t id)
// {
//     // TODO

//     // uint32_t ts = request.timestamp_ms;
//     // float value = request.value;
//     // bool clear_flag = request.clear;
//     // uint64_t current_timestamp = time_sync.unix_timestamp;
//     // uint64_t coordinate_timestamp = time_sync.timestamp_mod32_to_unix(request.timestamp_ms);

//     // switch (request.axis) {
//     focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
//     message.which_message = focstim_rpc_RpcMessage_response_tag;
//     message.message.response.id = id;
//     message.message.response.which_result = focstim_rpc_Response_response_axis_set_tag;
//     transmit_message(message);
// }

void ProtobufAPI::handle_request_timestamp_set(focstim_rpc_RequestTimestampSet &request, uint32_t id) {
    // TODO: improve smarts
    int64_t delta = time_sync.set_unix_timestamp(request.timestamp_ms);

    focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
    message.which_message = focstim_rpc_RpcMessage_response_tag;
    message.message.response.id = id;
    message.message.response.which_result = focstim_rpc_Response_response_timestamp_set_tag;
    message.message.response.result.response_timestamp_set.change_ms = delta;
    transmit_message(message);
}

void ProtobufAPI::handle_request_timestamp_get(focstim_rpc_RequestTimestampGet &request, uint32_t id) {
    focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
    message.which_message = focstim_rpc_RpcMessage_response_tag;
    message.message.response.id = id;
    message.message.response.which_result = focstim_rpc_Response_response_timestamp_get_tag;
    message.message.response.result.response_timestamp_get.timestamp_ms = millis();
    message.message.response.result.response_timestamp_get.unix_timestamp_ms = time_sync.unix_timestamp;
    transmit_message(message);
}

void ProtobufAPI::handle_request_signal_start(focstim_rpc_RequestSignalStart &request, uint32_t id) {
    focstim_rpc_Errors error = focstim_rpc_Errors_ERROR_UNKNOWN;

    switch (request.mode) {
        default:
        case focstim_rpc_OutputMode_OUTPUT_UNKNOWN:
        break;
        case focstim_rpc_OutputMode_OUTPUT_THREEPHASE:
        error = signal_start_threephase();
        break;
        case focstim_rpc_OutputMode_OUTPUT_FOURPHASE:
        error = signal_start_fourphase();
        break;
    }
    focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
    message.which_message = focstim_rpc_RpcMessage_response_tag;
    message.message.response.id = id;
    if (error == focstim_rpc_Errors_ERROR_UNKNOWN) {
        message.message.response.which_result = focstim_rpc_Response_response_signal_start_tag;
    } else {
        message.message.response.has_error = true;
        message.message.response.error.code = error;
    }
    transmit_message(message);
}

void ProtobufAPI::handle_request_signal_stop(focstim_rpc_RequestSignalStop &request, uint32_t id) {
    signal_stop();
    focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
    message.which_message = focstim_rpc_RpcMessage_response_tag;
    message.message.response.id = id;
    message.message.response.which_result = focstim_rpc_Response_response_signal_stop_tag;
    transmit_message(message);
}

void ProtobufAPI::handle_request_capabilities_get(focstim_rpc_RequestCapabilitiesGet &request, uint32_t id)
{
    focstim_rpc_RpcMessage message = focstim_rpc_RpcMessage_init_zero;
    message.which_message = focstim_rpc_RpcMessage_response_tag;
    message.message.response.id = id;
    message.message.response.which_result = focstim_rpc_Response_response_capabilities_get_tag;
    message.message.response.result.response_capabilities_get.threephase = capability_threephase();
    message.message.response.result.response_capabilities_get.fourphase = capability_fourphase();
    message.message.response.result.response_capabilities_get.battery = capability_battery();
    message.message.response.result.response_capabilities_get.potentiometer = capability_potmeter();
    message.message.response.result.response_capabilities_get.maximum_waveform_amplitude_amps = BODY_CURRENT_MAX;
    transmit_message(message);
}

void ProtobufAPI::handle_request_debug_stm32_deep_sleep(focstim_rpc_RequestDebugStm32DeepSleep &request, uint32_t id)
{
    while (1) {
        HAL_PWREx_EnterSHUTDOWNMode();
    }
}

void ProtobufAPI::handle_request(focstim_rpc_Request &request)
{
    // Serial.printf("transaction id: %u\r\n", request.id);
    uint32_t id = request.id;

    switch (request.which_params) {
        case focstim_rpc_Request_request_firmware_version_tag:
            handle_request_firmware_version(request.params.request_firmware_version, id);
        break;

        // case focstim_rpc_Request_request_axis_set_tag:
        //     handle_request_axis_set(request.params.request_axis_set, id);
        // break;

        case focstim_rpc_Request_request_timestamp_set_tag:
            handle_request_timestamp_set(request.params.request_timestamp_set, id);
        break;

        case focstim_rpc_Request_request_timestamp_get_tag:
            handle_request_timestamp_get(request.params.request_timestamp_get, id);
        break;

        case focstim_rpc_Request_request_signal_start_tag:
            handle_request_signal_start(request.params.request_signal_start, id);
        break;

        case focstim_rpc_Request_request_signal_stop_tag:
            handle_request_signal_stop(request.params.request_signal_stop, id);
        break;

        case focstim_rpc_Request_request_axis_move_to_tag:
            handle_request_axis_move_to(request.params.request_axis_move_to, id);
        break;

        case focstim_rpc_Request_request_capabilities_get_tag:
            handle_request_capabilities_get(request.params.request_capabilities_get, id);
        break;

        case focstim_rpc_Request_request_debug_stm32_deep_sleep_tag:
            handle_request_debug_stm32_deep_sleep(request.params.request_debug_stm32_deep_sleep, id);
        break;

        default:
            transmit_error_response(focstim_rpc_Errors_ERROR_UNKNOWN_REQUEST, id);
        // TOOD: error response
        break;
    }
}

void ProtobufAPI::handle_frame(const uint8_t* data, size_t data_len)
{
    // Serial.printf("received message of length: %u\r\n", data_len);

    pb_istream_t istream;
    istream = pb_istream_from_buffer(data, data_len);
    focstim_rpc_RpcMessage rpc_message = focstim_rpc_RpcMessage_init_zero;
    if (pb_decode(&istream, &focstim_rpc_RpcMessage_msg, &rpc_message)) {
        // Serial.printf("succesfully decoded message\r\n");
        switch (rpc_message.which_message) {
            case focstim_rpc_RpcMessage_request_tag:
            handle_request(rpc_message.message.request);
            break;
            case focstim_rpc_RpcMessage_response_tag:
            case focstim_rpc_RpcMessage_notification_tag:
            default:
            // do nothing
            break;
        }
    } else {
        BSP_PrintDebugMsg("error decoding proto %s", istream.errmsg);
    }
}

