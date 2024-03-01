/**
 * @brief Implement the GPIO buttons state and debouncing
 * @author Gilles Henrard
 * @date 01/03/2024
 */
#include <main.h>
#include "buttons.h"

#define DEBOUNCE_TIME_MS        50U;
#define EDGEDETECTION_TIME_MS   40U;

//machine state
static void stReleased(button_e button);
static void stPressed(button_e button);
//static void stHeldDown(button_e button);

/**
 * @brief State machine state prototype
 *
 * @return Error code of the state
 */
typedef void (*gpioState)(button_e button);

typedef struct{
    GPIO_TypeDef*   port;
    uint32_t        pin;
    gpioState       state;
}button_t;

gpioTimer_t buttonsTimers[NB_BUTTONS];

static button_t buttons[NB_BUTTONS] = {
    [ZERO] = {ZERO_BUTTON_GPIO_Port, ZERO_BUTTON_Pin, stReleased},
};


/********************************************************************************************************************************************/
/********************************************************************************************************************************************/


void buttonsUpdate(){
    for(uint8_t i = 0 ; i < NB_BUTTONS ; i++)
        (*buttons[i].state)(i);
}


/********************************************************************************************************************************************/
/********************************************************************************************************************************************/


static void stReleased(button_e button){
    //if button released, restart debouncing timer
    if(LL_GPIO_IsInputPinSet(buttons[button].port, buttons[button].pin))
        buttonsTimers[button].debouncing_ms = DEBOUNCE_TIME_MS;

    //if button not pressed for long enough, exit
    if(buttonsTimers[button].debouncing_ms)
        return;

    //set the timer during which rising edge can be read, and get to pressed state
    buttonsTimers[button].risingEdge_ms = EDGEDETECTION_TIME_MS;
    buttons[button].state = stPressed;
}

static void stPressed(button_e button){
    //if button pressed, restart debouncing timer
    if(!LL_GPIO_IsInputPinSet(buttons[button].port, buttons[button].pin))
        buttonsTimers[button].debouncing_ms = DEBOUNCE_TIME_MS;

    //if button not released for long enough, exit
    if(buttonsTimers[button].debouncing_ms)
        return;

    //set the timer during which falling edge can be read, and get to pressed state
    buttonsTimers[button].fallingEdge_ms = EDGEDETECTION_TIME_MS;
    buttons[button].state = stReleased;
}
/*
static void stHeldDown(button_e button){
    buttons[button].state = stPressed;
}
*/
