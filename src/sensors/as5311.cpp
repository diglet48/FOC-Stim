#include "sensors/as5311.h"
#include <Arduino.h>

#include "protobuf_api.h"
#include "as5311.h"


void AS5311::init(float read_interval_s, float notification_interval_s)
{
    this->read_interval_s = read_interval_s;
    this->notification_interval_s = notification_interval_s;

    LL_GPIO_SetPinMode(PORT_DO, PIN_DO, GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(PORT_DO, PIN_DO, GPIO_PULLDOWN);

    LL_GPIO_SetPinMode(PORT_CSN, PIN_CSN, GPIO_MODE_OUTPUT_PP);
    LL_GPIO_SetPinSpeed(PORT_CSN, PIN_CSN, GPIO_SPEED_FREQ_LOW);

    LL_GPIO_SetPinMode(PORT_CLK, PIN_CLK, GPIO_MODE_OUTPUT_PP);
    LL_GPIO_SetPinSpeed(PORT_CLK, PIN_CLK, GPIO_SPEED_FREQ_LOW);

    LL_GPIO_SetOutputPin(PORT_CSN, PIN_CSN);
    LL_GPIO_SetOutputPin(PORT_CLK, PIN_CLK);

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
    LL_GPIO_ResetOutputPin(PORT_CSN, PIN_CSN);
    LL_GPIO_SetOutputPin(PORT_CLK, PIN_CLK);
    delayMicroseconds(10);
    LL_GPIO_ResetOutputPin(PORT_CLK, PIN_CLK);
    delayMicroseconds(1);

    int result = 0;
    for (int i = 0; i < 18; i++) {
        LL_GPIO_SetOutputPin(PORT_CLK, PIN_CLK);

        delayMicroseconds(1);
        result = (result << 1) | LL_GPIO_IsInputPinSet(PORT_DO, PIN_DO);
        LL_GPIO_ResetOutputPin(PORT_CLK, PIN_CLK);
        delayMicroseconds(1);
    }

    delayMicroseconds(5);

    LL_GPIO_SetOutputPin(PORT_CSN, PIN_CSN);
    LL_GPIO_SetOutputPin(PORT_CLK, PIN_CLK);

    int raw = result >> (18 - 12);
    int status = result & 0x03F;

    flags = status;

    if (!(status & FLAG_OCF_MASK)) {
        // OCF bit not set, data invalid.
        return;
    }

    if (status & FLAG_COF_MASK) {
        // COF bit set, data invalid.
        return;
    }

    int offset = ((raw - last_raw) + 4096) % 4096;
    if (offset >= 2048) {
        offset -= 4096;
    }
    position += offset;
    last_raw = raw;

    // protobuf.transmit_notification_debug_string("sensor read: %#010x", result);
    // protobuf.transmit_notification_debug_string("sensor read: %i, %#02x", pos, status);
}

void AS5311::transmit_sensor_position() {
    g_protobuf->transmit_notification_debug_as5311(last_raw, position, flags);
}