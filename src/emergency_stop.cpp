#include "emergency_stop.h"

#include <SimpleFOC.h>
#include <Arduino.h>

#include "utils.h"
#include "config.h"

EmergencyStop::EmergencyStop()
{
}

void EmergencyStop::init(BLDCDriver6PWM *driver, CurrentSense *current_sense, void (*debug_fn)())
{
    this->driver = driver;
    this->currentSense = current_sense;
    this->debug_fn = debug_fn;
}

void EmergencyStop::check_vbus_overvoltage()
{
    float vbus = read_vbus(currentSense);
    if (vbus > STIM_PSU_VOLTAGE_MAX)
    {
        trigger_emergency_stop();
        while (1)
        {
            Serial.printf("V_BUS overvoltage detected %.2f. Current V_BUS=%.2f. Restart device to proceed\r\n",
                          vbus, read_vbus(currentSense));
            delay(5000);
        }
    }
}

void EmergencyStop::check_temperature()
{
    float temperature = read_temperature(currentSense);
    if (temperature > MAXIMUM_TEMPERATURE)
    {
        trigger_emergency_stop();
        while (1)
        {
            Serial.printf("temperature limit exceeded %.2f. Current temperature=%.2f. Restart device to proceed\r\n",
                          temperature, read_temperature(currentSense));
            delay(5000);
        }
    }
}


void EmergencyStop::trigger_emergency_stop()
{
    // drain the inductors as fast as possible.
    driver->setPwm(0, 0, 0);
    delayMicroseconds(200);
    // disable all phases just in case one of the mosfets blew up. Not sure if implemented on the B-G431B?
    driver->setPhaseState(PHASE_OFF, PHASE_OFF, PHASE_OFF);
    driver->setPwm(0, 0, 0);
    debug_fn();
}
