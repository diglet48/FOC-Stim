#include "power_manager.h"

#include "SparkFunBQ27441.h"
#include "BQ27441_Definitions.h"
#include <Wire.h>
#include "stim_clock.h"

#include "bsp/bsp.h"

void PowerManager::init() {
    Wire.setSCL(PB8);
    Wire.setSDA(PB7);
    Wire.setTimeout(10);
    Wire.begin();

    Serial.printf("detecting battery...\r\n");
    if (detect_battery()) {
        is_battery_present = lipo.begin();
    } else {
        is_battery_present = false;
    }

    if (is_battery_present) {
        Serial.printf("Battery detected!\r\n");
    } else {
        Serial.printf("Battery NOT detected\r\n");
        return;
    }

    Serial.printf("Check battery configuration.\r\n");
    bool gpout_polarity = lipo.GPOUTPolarity();
    if (gpout_polarity == 0) {
        Serial.printf("No battery configuration. Programming...\r\n");

        lipo.enterConfig(); // To configure the values below, you must be in config mode
        lipo.setCapacity(BATTERY_CAPACITY);     // Set the battery capacity
        lipo.setGPOUTPolarity(HIGH);            // Set GPOUT to active-high
        lipo.setGPOUTFunction(BAT_LOW);         // Set GPOUT to BAT_LOW mode
        lipo.setSOCFThresholds(SOCF_SET, SOCF_CLR); // Set SOCF set and clear thresholds
        lipo.setSOC1Thresholds(SOCI_SET, SOCI_CLR); // Set SOCI set and clear thresholds
        lipo.exitConfig();

    } else {
        Serial.printf("Battery already configured.\r\n");
    }


}

bool PowerManager::detect_battery()
{
    // Poll the BQ27441 a few times
    // If there is no battery, the BQ2407x charger performs a battery detection routine.
    // This power cycles the fuel gauge evert 250ms, resulting in intermittent connection.
    uint32_t start_time = millis();

    while (millis() - start_time < BATTERY_DETECTION_TIME_MS)
    {
        uint16_t dt = lipo.deviceType();
        if (dt != BQ27441_DEVICE_ID) {
            // connection unstable, battery must not be present.
            return false;
        }
        delay(50);
    }

    // connection stable. Battery must be present.
    return true;
}

void PowerManager::adjust_board_voltages()
{
    // battery               -> vbat - 0.4
    // no battery, usb power -> 4.3v
    if (is_battery_present) {
        float voltage = lipo.voltage() * 0.001f;
        // battery present, set boost enable threshold
        // slightly below battery voltage
        BSP_SetBoostMinimumInputVoltage(voltage - 0.4f);
    } else {
        // no battery present, set static boost enable threshold
        // to avoid starving the 3v3 buck/boost.
        BSP_SetBoostMinimumInputVoltage(4.3);
    }
}

void PowerManager::print_battery_stats()
{
    // Read battery stats from the BQ27441-G1A
    unsigned int soc = lipo.soc();  // Read state-of-charge (%)
    unsigned int volts = lipo.voltage(); // Read battery voltage (mV)
    int current = lipo.current(AVG); // Read average current (mA)
    unsigned int fullCapacity = lipo.capacity(FULL); // Read full capacity (mAh)
    unsigned int capacity = lipo.capacity(REMAIN); // Read remaining capacity (mAh)
    int power = lipo.power(); // Read average power draw (mW)
    int health = lipo.soh(); // Read state-of-health (%)

    // Assemble a string to print
    String toPrint = String(soc) + "% | ";
    toPrint += String(volts) + " mV | ";
    toPrint += String(current) + " mA | ";
    toPrint += String(capacity) + " / ";
    toPrint += String(fullCapacity) + " mAh | ";
    toPrint += String(power) + " mW | ";
    toPrint += String(health) + "%";
    toPrint += " | 0x" + String(lipo.flags(), HEX);

    Serial.println(toPrint);
}
