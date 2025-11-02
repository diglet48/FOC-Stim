#ifndef FOCSTIM_TRACE_H
#define FOCSTIM_TRACE_H

#include <stdint.h>
#include <Arduino.h>

#include "complex.h"
#include "bsp/bsp.h"

struct MainLoopTraceLine
{
    uint32_t t_start;   // time at which the pulse computation started
    uint32_t dt_play;   // delta between t_start and end of pulse playback
    uint32_t dt_next;   // delta between t_start and (planned) t_start of next pulse
    int skipped_update_steps;
    float v_drive;
    float v_boost_min;  // minimum supply voltage observed during the pulse
    float v_boost_max;  // maximum supply voltage observed during the pulse
    float i_max_a;
    float i_max_b;
    float i_max_c;
    float i_max_d;
    float i_max_cmd;
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
        BSP_PrintDebugMsg("mainloop timings:");
        BSP_PrintDebugMsg("     start|      play|      next|     skips|");
        for (int i = 0; i < MAINLOOP_NUM_ENTRIES; i++)
        {
            MainLoopTraceLine *p = &main_loop_trace[(i + main_loop_trace_index) % MAINLOOP_NUM_ENTRIES];
            BSP_PrintDebugMsg("%10lu %10lu %10lu %10i",
                          p->t_start,
                          p->dt_play,
                          p->dt_next,
                          p->skipped_update_steps);
        }
        BSP_PrintDebugMsg("mainloop signals:");
        BSP_PrintDebugMsg("   v_drive|   max I_a|   max I_b|   max I_c|   max I_d| i_max_cmd| boost_min| boost_max|");
        for (int i = 0; i < MAINLOOP_NUM_ENTRIES; i++)
        {
            MainLoopTraceLine *p = &main_loop_trace[(i + main_loop_trace_index) % MAINLOOP_NUM_ENTRIES];
            BSP_PrintDebugMsg("%10f %10f %10f %10f %10f %10f %10f %10f",
                          p->v_drive,
                          p->i_max_a,
                          p->i_max_b,
                          p->i_max_c,
                          p->i_max_d,
                          p->i_max_cmd,
                          p->v_boost_min,
                          p->v_boost_max);
        }
        BSP_PrintDebugMsg("mainloop model:");
        BSP_PrintDebugMsg("       Z_a|       Z_b|       Z_c|       Z_d|");
        for (int i = 0; i < MAINLOOP_NUM_ENTRIES; i++)
        {
            MainLoopTraceLine *p = &main_loop_trace[(i + main_loop_trace_index) % MAINLOOP_NUM_ENTRIES];
            BSP_PrintDebugMsg("%10f %10f %10f %10f",
                          p->Z_a.norm(),
                          p->Z_b.norm(),
                          p->Z_c.norm(),
                          p->Z_d.norm());
        }
    }

    MainLoopTraceLine main_loop_trace[MAINLOOP_NUM_ENTRIES] = {};

    unsigned main_loop_trace_index = 0;
};

#endif // FOCSTIM_TRACE_H