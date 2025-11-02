#include "power_manager.h"

#ifdef BATTERY_ENABLE

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

    BSP_PrintDebugMsg("detecting battery...");
    if (detect_battery()) {
        is_battery_present = lipo.begin();
    } else {
        is_battery_present = false;
    }

    if (is_battery_present) {
        BSP_PrintDebugMsg("Battery detected!");
    } else {
        BSP_PrintDebugMsg("Battery NOT detected.");
        return;
    }

    BSP_PrintDebugMsg("Check battery configuration.");
    bool gpout_polarity = lipo.GPOUTPolarity();
    if (gpout_polarity == 0) {
        BSP_PrintDebugMsg("No battery configuration. Programming...");

        lipo.enterConfig(); // To configure the values below, you must be in config mode
        lipo.setCapacity(BATTERY_CAPACITY);     // Set the battery capacity
        lipo.setGPOUTPolarity(HIGH);            // Set GPOUT to active-high
        lipo.setGPOUTFunction(BAT_LOW);         // Set GPOUT to BAT_LOW mode
        lipo.setSOCFThresholds(SOCF_SET, SOCF_CLR); // Set SOCF set and clear thresholds
        lipo.setSOC1Thresholds(SOCI_SET, SOCI_CLR); // Set SOCI set and clear thresholds
        lipo.exitConfig();

    } else {
        BSP_PrintDebugMsg("Battery already configured.");
    }

    if (is_battery_present) {
        BSP_SetBoostMinimumInputVoltage(3.3f);
    } else {
        BSP_SetBoostMinimumInputVoltage(4.3f);
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


#endif