#ifdef BOARD_FOCSTIM_V4
#include "pressure.h"

#include "Wire.h"
#include "protobuf_api.h"
#include "bsp/bsp.h"

#define DEFAULT_ADDRESS 0x18
#define MAXIMUM_PSI     25
#define MINIMUM_PSI     0

#define WAIT_TIME_AFTER_MEASUREMENT_MS 5


#define BUSY_FLAG       0x20
#define INTEGRITY_FLAG  0x04
#define MATH_SAT_FLAG   0x01

#define OUTPUT_MAX 0xE66666
#define OUTPUT_MIN 0x19999A

/**
 * Tested with sparkfun qwiic micropressure sensor (SEN-16476).
 * Should be compatible with Adafruit MPRLS Ported Pressure Sensor Breakout.
 * Both use MPRLS0025PA00001A sensors
 *
 */
PressureSensor::PressureSensor()
{

}

void PressureSensor::init()
{
    read_clock.reset();

    Wire.setClock(I2C_CLOCK_MICROPRESSURE);
    Wire.beginTransmission(DEFAULT_ADDRESS);
    uint8_t error = Wire.endTransmission();
    is_sensor_detected = error == 0;
    Wire.setClock(I2C_CLOCK_NORMAL);
}

void PressureSensor::update()
{
    if (!is_sensor_detected) {
        return;
    }

    if (is_sampling) {
        sample_clock.step();
        if (sample_clock.time_seconds >= (WAIT_TIME_AFTER_MEASUREMENT_MS / 1000.f)) {
            is_sampling = false;

            // median filter to filter out spikes, this sensor seems very suspectible to
            // noise from nearby 2.4ghz radios
            float pressure = read_sample();
            std::rotate(samples, samples + 1, samples + PRESSURE_SENSOR_MEDIAN_FILTER_SAMPLES);
            samples[PRESSURE_SENSOR_MEDIAN_FILTER_SAMPLES - 1] = pressure;

            // subsample messages
            message_counter++;
            if (message_counter >= (PRESSURE_SENSOR_MESSAGE_SUBSAMPLE - 1)) {
                float median = median_filter();
                g_protobuf->transmit_notification_pressure(median);
                message_counter = 0;
            }
        }
    }

    if (! is_sampling) {
        read_clock.step();
        if (read_clock.time_seconds >= (1.f / PRESSURE_SENSOR_UPDATE_RATE_HZ)) {
            start_sample();
            is_sampling = true;
            sample_clock.reset();
            read_clock.reset();
        }
    }
}

void PressureSensor::start_sample()
{
    // Tell the sensor to start sampling.
    // We must wait 5ms or wait for the busy flag to clear before the data can be read.
    Wire.setClock(I2C_CLOCK_MICROPRESSURE);
    Wire.beginTransmission(DEFAULT_ADDRESS);
    Wire.write((uint8_t)0xAA);
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)0x00);
    Wire.endTransmission();
    Wire.setClock(I2C_CLOCK_NORMAL);
}

float PressureSensor::read_sample()
{
    Wire.setClock(I2C_CLOCK_MICROPRESSURE);

    Wire.requestFrom(DEFAULT_ADDRESS, 4);

    uint8_t status = Wire.read();

    //check memory integrity and math saturation bits
    if((status & INTEGRITY_FLAG) || (status & MATH_SAT_FLAG))
    {
        Wire.setClock(I2C_CLOCK_NORMAL);
        return -1;
    }

    //read 24-bit pressure
    uint32_t reading = 0;
    for(uint8_t i=0;i<3;i++)
    {
        reading |= Wire.read();
        if(i != 2) reading = reading<<8;
    }

    //convert from 24-bit to float psi value
    float pressure;
    pressure = (reading - OUTPUT_MIN) * (MAXIMUM_PSI - MINIMUM_PSI);
    pressure = (pressure / (OUTPUT_MAX - OUTPUT_MIN)) + MINIMUM_PSI;
    pressure = pressure * 6894.7573f;  // PSI to Pa

    Wire.setClock(I2C_CLOCK_NORMAL);

    return pressure;
}

float PressureSensor::median_filter()
{
    float sorted[PRESSURE_SENSOR_MEDIAN_FILTER_SAMPLES];
    std::copy(samples, samples + PRESSURE_SENSOR_MEDIAN_FILTER_SAMPLES, sorted);
    std::sort(sorted, sorted + PRESSURE_SENSOR_MEDIAN_FILTER_SAMPLES);
    return sorted[(PRESSURE_SENSOR_MEDIAN_FILTER_SAMPLES - 1) / 2];
}

#endif