#ifndef FOCSTIM_POWER_MANAGER_H
#define FOCSTIM_POWER_MANAGER_H

#include <stdint.h>

#define BATTERY_DETECTION_TIME_MS 500

// Set BATTERY_CAPACITY to the design capacity of your battery.
const unsigned int BATTERY_CAPACITY = 1200; // e.g. 850mAh battery

const uint8_t SOCI_SET = 10; // battery low threshold, turn the system OFF (in percent)
const uint8_t SOCI_CLR = 15; // battery low threshold clear.
const uint8_t SOCF_SET = 10; // not used
const uint8_t SOCF_CLR = 20; // not used



class PowerManager {
public:
    PowerManager() = default;

    void init();

    bool detect_battery();

    void adjust_board_voltages();

    void print_battery_stats();


    bool is_battery_present;
};

#endif