#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_
#include <stdint.h>

/**
 * @brief Enumeration of all the managed buttons
 */
typedef enum {
    ZERO = 0,
    HOLD,
    POWER,
    NB_BUTTONS
} button_e;

void    createButtonsTask(void);
uint8_t isButtonReleased(button_e button);
uint8_t isButtonPressed(button_e button);
uint8_t isButtonHeldDown(button_e button);
uint8_t buttonHasRisingEdge(button_e button);
uint8_t buttonHasFallingEdge(button_e button);

#endif
