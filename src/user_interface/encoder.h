#ifndef FOCSTIM_ENCODER_H
#define FOCSTIM_ENCODER_H

#include "bsp/bsp.h"

#ifdef BOARD_FOCSTIM_V4

class Encoder {
public:
    Encoder();

    void update();

    float percentage() const;
    float volume() const;
    void resetVolume();
    float unlockProgress() const;

    void setLocked(bool b);
    bool isLocked();
    bool isQuickTurned() const;


private:
    uint16_t last_encoder_position;
    int position;
    bool locked;

    uint32_t last_update_time;
    float negative_velocity;
};

#endif
#endif