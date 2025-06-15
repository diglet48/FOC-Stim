#ifndef FOCSTIM_TIMESTAMP_SYNC_H
#define FOCSTIM_TIMESTAMP_SYNC_H

#include <cstdint>

#include <Arduino.h>

class TimestampSync {
public:
    void init() {
        last_local_timestamp = millis();
    }

    int64_t set_unix_timestamp(uint64_t new_unix_timestamp) {
        last_local_timestamp = millis();
        int64_t delta = unix_timestamp - new_unix_timestamp;
        unix_timestamp = new_unix_timestamp;
        return delta;
    }

    uint64_t timestamp_mod32_to_unix(uint32_t timestamp) {
        int32_t diff = (int32_t)(timestamp - (uint32_t)unix_timestamp);
        return unix_timestamp + diff;
    }

    void step() {
        uint32_t new_local_timestamp = millis();
        uint32_t elapsed = new_local_timestamp - last_local_timestamp;
        last_local_timestamp = new_local_timestamp;

        unix_timestamp += (uint64_t)elapsed;
    }

    uint64_t unix_timestamp;
    uint32_t last_local_timestamp;

};

#endif