#ifndef FOCSTIM_EMERGENCY_STOP_H
#define FOCSTIM_EMERGENCY_STOP_H

class BLDCDriver6PWM;
class CurrentSense;

class EmergencyStop
{
public:
    EmergencyStop();

    void init(BLDCDriver6PWM *driver, CurrentSense *current_sense, void (*debug_fn)());

    // check if VBUS is within safe limits
    void check_vbus_overvoltage();

    // check if temperature is within safe limits
    void check_temperature();

    // Put hardware in safe state, then call the debug function
    void trigger_emergency_stop();

    BLDCDriver6PWM *driver = nullptr;
    CurrentSense *currentSense = nullptr;
    void (*debug_fn)();
};

#endif // FOCSTIM_EMERGENCY_STOP_H