#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_
#include "errorstack.h"

/**
 * @brief Enumeration of all the managed buttons
 */
typedef enum{
    ZERO = 0,
    NB_BUTTONS
}button_e;

/**
 * @brief Structure holding all the timers used by the buttons
 */
typedef struct{
    volatile uint8_t debouncing_ms;     ///< Timer used for debouncing (in ms)
    volatile uint16_t holding_ms;       ///< Timer used to detect if a button is held down (in ms)
    volatile uint8_t risingEdge_ms;     ///< Timer used to detect a rising edge (in ms)
    volatile uint8_t fallingEdge_ms;    ///< Timer used to detect a falling edge (in ms)
}gpioTimer_t;

void buttonsUpdate();
uint8_t isButtonReleased(button_e button);
uint8_t isButtonPressed(button_e button);
uint8_t isButtonHeldDown(button_e button);
uint8_t buttonHasRisingEdge(button_e button);
uint8_t buttonHasFallingEdge(button_e button);

extern gpioTimer_t buttonsTimers[NB_BUTTONS];

#endif