#ifdef BOARD_FOCSTIM_V4
#include "imu.h"

#include "protobuf_api.h"
#include "bsp/bsp.h"


IMU::IMU()
    :   lsm6dsoxSensor(&Wire, LSM6DSOX_I2C_ADD_L)
    ,   is_sensor_detected(false)
{

}

void IMU::init()
{
    Wire.setClock(I2C_CLOCK_LSM6DSOX);
    // Initialize sensors
    lsm6dsoxSensor.begin();
    if (lsm6dsoxSensor.Enable_G() == LSM6DSOX_OK && lsm6dsoxSensor.Enable_X() == LSM6DSOX_OK)
    {
        BSP_PrintDebugMsg("accelero and gyro enabled.");
    }
    else
    {
        BSP_PrintDebugMsg("accelero and gyro NOT enabled.");
        Wire.setClock(I2C_CLOCK_NORMAL);
        return;
    }

    // Check device id
    uint8_t id;
    lsm6dsoxSensor.ReadID(&id);
    if (id != LSM6DSOX_ID)
    {
        BSP_PrintDebugMsg("LSM6DSOX sensor NOT detected.");
        Wire.setClock(I2C_CLOCK_NORMAL);
        return;
    }
    else
    {
        BSP_PrintDebugMsg("LSM6DSOX sensor detected.");
    }

    is_sensor_detected = true;
    Wire.setClock(I2C_CLOCK_NORMAL);
}

void IMU::start_stream(int imu_samplerate, int acc_fullscale, int gyr_fullscale)
{
    if (is_sensor_detected) {

        Wire.setClock(I2C_CLOCK_LSM6DSOX);

        // Set accelerometer Output Data Rate. Available values are: 1.6, 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
        lsm6dsoxSensor.Set_X_ODR(imu_samplerate);
        // Set gyroscope Output Data Rate. Available values are 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
        lsm6dsoxSensor.Set_G_ODR(imu_samplerate);

        // Set FIFO Batch Data Rate for accelerometer and gyroscope. Available values are: 0, 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
        lsm6dsoxSensor.Set_FIFO_X_BDR(imu_samplerate);
        lsm6dsoxSensor.Set_FIFO_G_BDR(imu_samplerate);

        // Set accelerometer scale. Available values are: 2, 4, 8, 16 G
        lsm6dsoxSensor.Set_X_FS(acc_fullscale);
        // Set gyroscope scale. Available values are: 125, 250, 500, 1000, 2000 dps
        lsm6dsoxSensor.Set_G_FS(gyr_fullscale);

        // refresh sensitivity
        lsm6dsoxSensor.Get_X_Sensitivity(&acc_sensitivity);
        lsm6dsoxSensor.Get_G_Sensitivity(&gyr_sensitivity);

        lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE); // flush any previous value in FIFO before start
        lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_STREAM_MODE); // start batching in continous mode

        Wire.setClock(I2C_CLOCK_NORMAL);
    }
}

void IMU::stop_stream()
{
    if (is_sensor_detected) {

        Wire.setClock(I2C_CLOCK_LSM6DSOX);

        lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE);

        Wire.setClock(I2C_CLOCK_NORMAL);
    }
}

void IMU::update()
{
    if (!is_sensor_detected) {
        return;
    }

    Wire.setClock(I2C_CLOCK_LSM6DSOX);

    uint8_t fullStatus = 0;       // FIFO full status
    uint16_t numSamples = 0;      // number of samples in FIFO
    uint8_t tag;                  // FIFO data sensor identifier

    // Get number of samples in buffer
    lsm6dsoxSensor.Get_FIFO_Num_Samples(&numSamples);

    if (numSamples > IMU_MAX_SAMPLES_IN_BUFFER) {
        BSP_PrintDebugMsg("LSM6DSOX FIFO full");
        lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE); // flush FIFO data
        lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_STREAM_MODE); // continue batching
        numSamples = 0;
    }

    // fetch data from FIFO
    for (uint16_t i = 0; i < numSamples; i++)
    {
        lsm6dsoxSensor.Get_FIFO_Tag(&tag); // get data identifier

        // Get gyroscope data
        if (tag == 1)
        {
            read_fifo_data(rotation);

        }

        // Get accelerometer data
        else if (tag == 2)
        {
            read_fifo_data(acceleration);
            // one update is around 19 bytes including protocol overhead.
            g_protobuf->transmit_notification_lsm6dsox(
                acceleration[0], acceleration[1], acceleration[2],
                rotation[0], rotation[1], rotation[2]);
        }
    }

    Wire.setClock(I2C_CLOCK_NORMAL);
}

LSM6DSOXStatusTypeDef IMU::read_fifo_data(int16_t *data_out)
{
    uint8_t data[6];

    if (lsm6dsoxSensor.Get_FIFO_Data(data) != LSM6DSOX_OK)
    {
        data_out[0] = 0;
        data_out[1] = 0;
        data_out[2] = 0;
        return LSM6DSOX_ERROR;
    }

    data_out[0] = ((int16_t)data[1] << 8) | data[0];
    data_out[1] = ((int16_t)data[3] << 8) | data[2];
    data_out[2] = ((int16_t)data[5] << 8) | data[4];

    return LSM6DSOX_OK;
}

#endif