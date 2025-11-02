#ifndef FOCSTIM_POWER_MANAGER_H
#define FOCSTIM_POWER_MANAGER_H

#include "bsp/bsp.h"

#ifdef BATTERY_ENABLE

#include "stim_clock.h"
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
    void update();
    void update_all();

    bool is_battery_present;

    float cached_temperature;               // deg c
    float cached_voltage;                   // V
    int cached_soh;                         // percent
    float cached_remaning_capacity_uf;      // mAh
    float cached_full_charge_capacity_uf;   // mAh
    float cached_power;                     // mW
    float cached_soc() {
        return cached_remaning_capacity_uf / cached_full_charge_capacity_uf;
    }

    void print_battery_stats();
private:

    bool detect_battery();

    void read_temperature();
    void read_voltage();
    void read_soh();
    void read_remaining_capacity_uf();
    void read_full_charge_capacity_uf();
    void read_power();

    Clock update_clock;
    int update_step;
};

#endif
#endif