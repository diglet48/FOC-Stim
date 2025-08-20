#ifndef FOCSTIM_AS5311_H
#define FOCSTIM_AS5311_H

#include "stim_clock.h"


#define PORT_DO GPIOA
#define PIN_DO  LL_GPIO_PIN_1
#define PORT_CSN GPIOA
#define PIN_CSN LL_GPIO_PIN_0
#define PORT_CLK GPIOA
#define PIN_CLK LL_GPIO_PIN_5


#define FLAG_OCF_MASK (1 << 5)  // offset compensation.
#define FLAG_COF_MASK (1 << 4)  // cordic overflow
#define FLAG_LIN_MASK (1 << 3)
#define FLAG_INC_MASK (1 << 2)
#define FLAG_DEC_MASK (1 << 1)
#define FLAG_PAR_MASK (1 << 0)  // parity

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