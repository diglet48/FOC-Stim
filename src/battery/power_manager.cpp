#include "power_manager.h"

#ifdef BATTERY_ENABLE

#include "SparkFunBQ27441.h"
#include "BQ27441_Definitions.h"
#include <Wire.h>
#include "stim_clock.h"

#include "bsp/bsp.h"

void PowerManager::init() {
    Wire.setTimeout(10);
    Wire.begin();

    Wire.setClock(I2C_CLOCK_BQ27411);
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
        Wire.setClock(I2C_CLOCK_NORMAL);
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

    update_all();
    Wire.setClock(I2C_CLOCK_NORMAL);
}

void PowerManager::update()
{
    if (!is_battery_present) {
        return;
    }

    // update one parameter every so often to avoid blocking the mainloop.
    update_clock.step();
    if (update_clock.time_seconds <= 0.25f) {
        return;
    }
    update_clock.reset();

    Wire.setClock(I2C_CLOCK_BQ27411);
    update_step = (update_step + 1) % 6;
    switch (update_step) {
        case 0:
        read_temperature();
        break;
        case 1:
        read_voltage();
        break;
        case 2:
        read_soh();
        break;
        case 3:
        read_remaining_capacity_uf();
        break;
        case 4:
        read_full_charge_capacity_uf();
        break;
        case 5:
        read_power();
        break;
    }
    Wire.setClock(I2C_CLOCK_NORMAL);
}

void PowerManager::update_all()
{
    read_temperature();
    read_voltage();
    read_soh();
    read_remaining_capacity_uf();
    read_full_charge_capacity_uf();
    read_power();
}

void PowerManager::print_battery_stats()
{
    BSP_PrintDebugMsg("soc %i", lipo.soc());
    BSP_PrintDebugMsg("DesignCapacity %i", lipo.capacity(DESIGN));

    BSP_PrintDebugMsg("RemainingCapacity %i", lipo.capacity(REMAIN));               // compensated battery capacity remaining, compensated for load and temp
    BSP_PrintDebugMsg("RemainingCapacityFiltered %i", lipo.capacity(REMAIN_F));
    BSP_PrintDebugMsg("RemainingCapacityUnfiltered %i", lipo.capacity(REMAIN_UF));

    BSP_PrintDebugMsg("FullChargeCapacity %i", lipo.capacity(FULL));                // compensated battery capacity when fully charged, compensated for load and temp
    BSP_PrintDebugMsg("FullChargeCapacityFiltered %i", lipo.capacity(FULL_F));
    BSP_PrintDebugMsg("FullChargeCapacityUnFiltered %i", lipo.capacity(FULL_UF));

    BSP_PrintDebugMsg("NominalAvailableCapacity %i", lipo.capacity(AVAIL));         // uncompensated battery capacity remaining
    BSP_PrintDebugMsg("FullAvailableCapacity %i", lipo.capacity(AVAIL_FULL));       // uncompensated battery capacity when fully charged

    BSP_PrintDebugMsg("soh %% %i", lipo.soh(PERCENT));
    BSP_PrintDebugMsg("soh %i", lipo.soh(SOH_STAT));
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

void PowerManager::read_temperature()
{
    cached_temperature = lipo.temperature(INTERNAL_TEMP) * 0.1f - 273.15f;
}

void PowerManager::read_voltage()
{
    cached_voltage = lipo.voltage() * 0.001f;
}

void PowerManager::read_soh()
{
    cached_soh = lipo.soh() * 0.01;
}

void PowerManager::read_remaining_capacity_uf()
{
    cached_remaning_capacity_uf = lipo.capacity(REMAIN_UF); // RemainingCapacityUnfiltered()
}

void PowerManager::read_full_charge_capacity_uf()
{
    cached_full_charge_capacity_uf = lipo.capacity(FULL_UF); // FullChargeCapacityUnfiltered()
}

void PowerManager::read_power()
{
    cached_power = lipo.power() * 0.001f; // AveragePower()
}

#endif