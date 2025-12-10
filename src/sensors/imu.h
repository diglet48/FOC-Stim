#ifndef FOCSTIM_IMU_H
#define FOCSTIM_IMU_H
#ifdef BOARD_FOCSTIM_V4

#include "LSM6DSOXSensor.h"

// one update is around 46 bytes.
#define IMU_SAMPLERATE 26     // Sample rate. Options are: 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333 and 6667 Hz.

class IMU {
public:
    IMU();

    void init();

    void update();

private:
    LSM6DSOXSensor lsm6dsoxSensor;

    bool is_sensor_detected;

    int32_t acceleration[3];      // X, Y, Z accelerometer values in mg
    int32_t rotation[3];          // X, Y, Z giroscope values in mdps

};


#endif
#endif