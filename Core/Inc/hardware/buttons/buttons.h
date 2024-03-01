#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_
#include "errorstack.h"

typedef enum{
    ZERO = 0,
    NB_BUTTONS
}button_e;

typedef struct{
    volatile uint8_t debouncing_ms;
    volatile uint16_t holding_ms;
    volatile uint8_t risingEdge_ms;
    volatile uint8_t fallingEdge_ms;
}gpioTimer_t;

void buttonsUpdate();
uint8_t isButtonReleased(button_e button);
uint8_t isButtonPressed(button_e button);
uint8_t isButtonHeldDown(button_e button);
uint8_t buttonHasRisingEdge(button_e button);
uint8_t buttonHasFallingEdge(button_e button);

extern gpioTimer_t buttonsTimers[NB_BUTTONS];

#endif