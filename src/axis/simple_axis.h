#ifndef FOCSTIM_SIMPLE_AXIS
#define FOCSTIM_SIMPLE_AXIS

#include <Arduino.h>


struct SimpleAxis
{
    SimpleAxis(int id, float init, float min_value, float max_value)
        : id(id), T0(0), T1(0), value0(init), value1(init), min_value(min_value), max_value(max_value)

    {
    }

    float get(uint32_t now_ms) {
        float p;
        if (T1 == T0)
        {
            p = 1;
        }
        else
        {
            p = float(now_ms - T0) / float(T1 - T0);
            p = max(min(p, 1.f), 0.f);
        }
        float value = value0 + p * (value1 - value0);
        return max(min(value, max_value), min_value);
    }

    void move_to(uint32_t now_ms, float value, uint32_t interval) {
        if (interval == 0) {
            value0 = value;
            value1 = value;
            T0 = now_ms;
            T1 = now_ms;
        } else {
            value0 = this->get(now_ms);
            value1 = value;
            T0 = now_ms;
            T1 = now_ms + interval;
        }
    }

    uint32_t id;
    uint32_t T0;
    uint32_t T1;
    float value0;
    float value1;
    float min_value;
    float max_value;

};

#endif