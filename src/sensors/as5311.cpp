#include "sensors/as5311.h"
#include <Arduino.h>

#include "protobuf_api.h"
#include "as5311.h"


void AS5311::init(float read_interval_s, float notification_interval_s)
{
    this->read_interval_s = read_interval_s;
    this->notification_interval_s = notification_interval_s;

    pinMode(PIN_CSN, OUTPUT);
    pinMode(PIN_CLK, OUTPUT);
    pinMode(PIN_DO, INPUT_PULLDOWN);

    digitalWrite(PIN_CSN, 1);
    digitalWrite(PIN_CLK, 1);

    last_raw = 0;
    position = 0;
    flags = 0;

    read_sensor();
    is_sensor_detected = flags != 0;
}

void AS5311::update()
{
    if (! is_sensor_detected) {
        return;
    }

    read_clock.step();
    if (read_clock.time_seconds > read_interval_s) {
        read_clock.reset();
        read_sensor();
    }

    notification_clock.step();
    if (notification_clock.time_seconds > notification_interval_s) {
        notification_clock.reset();
        transmit_sensor_position();
    }
}

void AS5311::read_sensor() {

    digitalWrite(PIN_CSN, 0);   // CSN
    digitalWrite(PIN_CLK, 1);
    delayMicroseconds(10);

    digitalWrite(PIN_CLK, 0);
    delayMicroseconds(1);

    int result = 0;
    for (int i = 0; i < 18; i++) {
        digitalWrite(PIN_CLK, 1);
        delayMicroseconds(1);
        result = (result << 1) | digitalRead(PIN_DO);
        digitalWrite(PIN_CLK, 0);
        delayMicroseconds(1);
    }

    delayMicroseconds(1);
    digitalWrite(PIN_CLK, 1);
    digitalWrite(PIN_CSN, 1);   // CSN

    int raw = result >> (18 - 12);
    int status = result & 0x03F;

    if (!(status & FLAG_OCF_MASK)) {
        // OCF bit not set, data invalid.
        return;
    }

    if (status & FLAG_COF_MASK) {
        // COF bit not set, data invalid.
        return;
    }

    int offset = ((raw - last_raw) + 4096) % 4096;
    if (offset >= 2048) {
        offset -= 4096;
    }
    position += offset;
    last_raw = raw;
    flags = status;

    // protobuf.transmit_notification_debug_string("sensor read: %#010x", result);
    // protobuf.transmit_notification_debug_string("sensor read: %i, %#02x", pos, status);
}

void AS5311::transmit_sensor_position() {
    if (! is_sensor_detected) {

    }

    g_protobuf->transmit_notification_debug_as5311(last_raw, position, flags);
}