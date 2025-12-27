#ifndef FOCSTIM_IMU_H
#define FOCSTIM_IMU_H
#ifdef BOARD_FOCSTIM_V4

#include "LSM6DSOXSensor.h"


#define IMU_MAX_SAMPLES_IN_BUFFER   10

class IMU {
public:
    IMU();

    void init();

    void start_stream(int imu_samplerate, int x_fullscale, int g_fullscale);

    void stop_stream();

    void update();

    float acc_sensitivity;
    float gyr_sensitivity;

    bool is_sensor_detected;

private:
    LSM6DSOXStatusTypeDef read_fifo_data(int16_t* data_out);

    LSM6DSOXSensor lsm6dsoxSensor;

    // int32_t acceleration[3];      // X, Y, Z accelerometer values in mg
    // int32_t rotation[3];          // X, Y, Z giroscope values in mdps

    int16_t acceleration[3];      // X, Y, Z accelerometer values in mg
    int16_t rotation[3];          // X, Y, Z giroscope values in mdps

};


#endif
#endif