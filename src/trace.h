#ifndef FOCSTIM_TRACE_H
#define FOCSTIM_TRACE_H

#include <stdint.h>
#include <Arduino.h>

#include "complex.h"

struct MainLoopTraceLine
{
    uint32_t t_start;
    uint32_t dt_compute;
    uint32_t dt_play;
    uint32_t dt_logs;
    int skipped_update_steps;
    float v_drive_max;
    float i_max_a;
    float i_max_b;
    float i_max_c;
    float i_max_d;
    float amplitude;
    Complex Z_a;
    Complex Z_b;
    Complex Z_c;
    Complex Z_d;
};

class Trace
{
    static const int MAINLOOP_NUM_ENTRIES = 32;

public:
    Trace() {}

    MainLoopTraceLine *next_main_loop_line()
    {
        MainLoopTraceLine *p = &main_loop_trace[main_loop_trace_index];
        main_loop_trace_index = (main_loop_trace_index + 1) % MAINLOOP_NUM_ENTRIES;
        *p = {};
        return p;
    }

    void print_mainloop_trace()
    {
        Serial.printf("mainloop timings:\r\n");
        Serial.printf("     start|   compute|      play|      logs|     skips|\r\n");
        for (int i = 0; i < MAINLOOP_NUM_ENTRIES; i++)
        {
            MainLoopTraceLine *p = &main_loop_trace[(i + main_loop_trace_index) % MAINLOOP_NUM_ENTRIES];
            Serial.printf("%10lu %10lu %10lu %10lu %10i\r\n",
                          p->t_start,
                          p->dt_compute,
                          p->dt_play,
                          p->dt_logs,
                          p->skipped_update_steps);
        }
        Serial.println();
        Serial.printf("mainloop signals:\r\n");
        Serial.printf("   v_drive|   max I_a|   max I_b|   max I_c|   max I_d| amplitude|\r\n");
        for (int i = 0; i < MAINLOOP_NUM_ENTRIES; i++)
        {
            MainLoopTraceLine *p = &main_loop_trace[(i + main_loop_trace_index) % MAINLOOP_NUM_ENTRIES];
            Serial.printf("%10f %10f %10f %10f %10f %10f\r\n",
                          p->v_drive_max,
                          p->i_max_a,
                          p->i_max_b,
                          p->i_max_c,
                          p->i_max_d,
                          p->amplitude);
        }
        Serial.println();
        Serial.printf("mainloop MRAC:\r\n");
        Serial.printf("       Z_a|       Z_b|       Z_c|       Z_d|         L|\r\n");
        for (int i = 0; i < MAINLOOP_NUM_ENTRIES; i++)
        {
            MainLoopTraceLine *p = &main_loop_trace[(i + main_loop_trace_index) % MAINLOOP_NUM_ENTRIES];
            Serial.printf("%10f %10f %10f %10f %10f\r\n",
                          p->Z_a.norm(),
                          p->Z_b.norm(),
                          p->Z_c.norm(),
                          p->Z_d.norm());
        }
        Serial.println();
    }

    MainLoopTraceLine main_loop_trace[MAINLOOP_NUM_ENTRIES] = {};

    unsigned main_loop_trace_index = 0;
};

#endif // FOCSTIM_TRACE_H