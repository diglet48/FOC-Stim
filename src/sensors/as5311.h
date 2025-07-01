#ifndef FOCSTIM_AS5311_H
#define FOCSTIM_AS5311_H

#include "stim_clock.h"


#define PIN_DO  PA1
#define PIN_CSN PA0
#define PIN_CLK PA5

class AS5311 {
    public:

    void init(float read_interval_s, float notification_interval_s);
    void update();

    void read_sensor();
    void transmit_sensor_position();


    Clock read_clock;
    Clock notification_clock;
    float read_interval_s;
    float notification_interval_s;

    bool is_sensor_detected;
    int last_raw;
    int position;
    int flags;
};

#endif