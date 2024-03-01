/**
 * @brief Implement the GPIO buttons state and debouncing
 * @author Gilles Henrard
 * @date 01/03/2024
 */
#include <main.h>
#include "buttons.h"

//machine state
static void stReleased(button_e button);
static void stPressed(button_e button);
static void stHeldDown(button_e button);

typedef struct{
    GPIO_TypeDef*   port;
    uint32_t        pin;
}button_t;

/**
 * @brief State machine state prototype
 *
 * @return Error code of the state
 */
typedef void (*gpioState)(button_e button);

static gpioState _state = stReleased;
//static button_t buttons[NB_BUTTONS];


/********************************************************************************************************************************************/
/********************************************************************************************************************************************/


void buttonsUpdate(){
    for(uint8_t i = 0 ; i < NB_BUTTONS ; i++)
        (*_state)(i);
}


/********************************************************************************************************************************************/
/********************************************************************************************************************************************/


static void stReleased(button_e button){
    (void)button;
    _state = stPressed;
}

static void stPressed(button_e button){
    (void)button;
    _state = stHeldDown;
}

static void stHeldDown(button_e button){
    (void)button;
    _state = stPressed;
}