#ifndef FOCSTIM_POWER_MANAGER_H
#define FOCSTIM_POWER_MANAGER_H

#include "bsp/bsp.h"

#ifdef BATTERY_ENABLE

#include <stdint.h>

#define BATTERY_DETECTION_TIME_MS 500

// Set BATTERY_CAPACITY to the design capacity of your battery.
const unsigned int BATTERY_CAPACITY = 2000; // e.g. 850mAh battery

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

    // bool pgood;                 // cached value of 'batPGood' pin
    // float volts;                // battery voltage [V]
    // float current;              // battery charge or discharge current [A]
    // float power;                // battery charge or discharge power [W]
    // float capacity_full;        // battery full capacity
    // float capacity_remain;      //
    // float health;               // battery health [% 0-1]

};

#endif
#endif