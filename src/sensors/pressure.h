#ifndef FOCSTIM_PRESSURE_H
#define FOCSTIM_PRESSURE_H
#ifdef BOARD_FOCSTIM_V4

#include "stim_clock.h"

#define PRESSURE_SENSOR_UPDATE_RATE_HZ 100
#define PRESSURE_SENSOR_MESSAGE_SUBSAMPLE 2
#define PRESSURE_SENSOR_MEDIAN_FILTER_SAMPLES 5


class PressureSensor {
public:
    PressureSensor();

    void init();

    void update();

    bool is_sensor_detected;

private:

    void start_sample();
    float read_sample();
    float median_filter();

    Clock read_clock;
    Clock sample_clock;
    bool is_sampling = false;
    int message_counter = 0;

    float samples[PRESSURE_SENSOR_MEDIAN_FILTER_SAMPLES];
};


#endif
#endif