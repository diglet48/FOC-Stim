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

    // Set accelerometer scale. Available values are: 2, 4, 8, 16 G
    lsm6dsoxSensor.Set_X_FS(4);
    // Set gyroscope scale. Available values are: 125, 250, 500, 1000, 2000 dps
    lsm6dsoxSensor.Set_G_FS(500);

    // Set accelerometer Output Data Rate. Available values are: 1.6, 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
    lsm6dsoxSensor.Set_X_ODR(IMU_SAMPLERATE);
    // Set gyroscope Output Data Rate. Available values are 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
    lsm6dsoxSensor.Set_G_ODR(IMU_SAMPLERATE);

    // Set FIFO Batch Data Rate for accelerometer and gyroscope. Available values are: 0, 12.5, 26, 52, 104, 208, 417, 833, 1667, 3333, 6667 Hz
    lsm6dsoxSensor.Set_FIFO_X_BDR(IMU_SAMPLERATE);
    lsm6dsoxSensor.Set_FIFO_G_BDR(IMU_SAMPLERATE);

    /** Set FIFO operation mode. Available values are:
     * LSM6DSOX_BYPASS_MODE: FIFO is not used, the buffer content is cleared
     * LSM6DSOX_FIFO_MODE: bufer continues filling until it becomes full. Then it stops collecting data.
     * LSM6DSOX_STREAM_MODE: continuous mode. Older data are replaced by the new data.
     * LSM6DSOX_STREAM_TO_FIFO_MODE: FIFO buffer starts operating in Continuous mode and switches to FIFO mode when an event condition occurs.
     * LSM6DSOX_BYPASS_TO_STREAM_MODE: FIFO buffer starts operating in Bypass mode and switches to Continuous mode when an event condition occurs.
     * */
    lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_BYPASS_MODE); // flush any previous value in FIFO before start
    lsm6dsoxSensor.Set_FIFO_Mode(LSM6DSOX_STREAM_MODE); // start batching in continous mode

    is_sensor_detected = true;
    Wire.setClock(I2C_CLOCK_NORMAL);
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
            lsm6dsoxSensor.Get_FIFO_G_Axes(rotation);
        }

        // Get accelerometer data
        else if (tag == 2)
        {
            lsm6dsoxSensor.Get_FIFO_X_Axes(acceleration);
            g_protobuf->transmit_notification_lsm6dsox(
                acceleration[0], acceleration[1], acceleration[2],
                rotation[0], rotation[1], rotation[2]);
        }
    }

    Wire.setClock(I2C_CLOCK_NORMAL);
}

#endif