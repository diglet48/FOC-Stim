#ifndef FOCSTIM_BUTTON_H
#define FOCSTIM_BUTTON_H

#include "bsp/bsp.h"

#ifdef BOARD_FOCSTIM_V4

enum ButtonEvent {
    None,
    Press,
    Release
};

class Button {
public:
    Button() = default;

    ButtonEvent getEvent() {
        // TODO: maybe needs debouncing?
        bool prev_state = button_down;
        button_down = BSP_ReadEncoderButton();
        if (prev_state == false && button_down == true) {
            return ButtonEvent::Press;
        }
        if (prev_state == true && button_down == false) {
            return ButtonEvent::Release;
        }
        return ButtonEvent::None;
    };

private:
    bool button_down;
};


#endif
#endif