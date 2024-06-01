/**
 * @brief Implement the GPIO buttons state and debouncing
 * @author Gilles Henrard
 * @date 08/03/2024
 */
#include "buttons.h"
#include <main.h>
#include "stdbool.h"

_Static_assert((bool)(NB_BUTTONS <= UINT8_MAX), "The application supports maximum 255 buttons");

#define DEBOUNCE_TIME_MS      50U;    ///< Number of milliseconds to wait for debouncing
#define HOLDING_TIME_MS       1000U;  ///< Number of milliseconds to wait before considering a button is held down
#define EDGEDETECTION_TIME_MS 40U;    ///< Number of milliseconds during which a falling/rising edge can be detected

//machine state
static void stReleased(button_e button);
static void stPressed(button_e button);
static void stHeldDown(button_e button);

/**
 * @brief State machine state prototype
 *
 * @return Error code of the state
 */
typedef void (*gpioState)(button_e button);

/**
 * @brief Structure defining a button GPIO
 */
typedef struct {
    GPIO_TypeDef* port;   ///< GPIO port used
    uint32_t      pin;    ///< GPIO pin used
    gpioState     state;  ///< Current button state
} button_t;

gpioTimer_t buttonsTimers[NB_BUTTONS];  ///< Array of timers used by the buttons

/**
 * @brief Buttons initialisation array
 */
static button_t buttons[NB_BUTTONS] = {
    [ZERO]  = { ZERO_BUTTON_GPIO_Port,  ZERO_BUTTON_Pin, stReleased},
    [HOLD]  = { HOLD_BUTTON_GPIO_Port,  HOLD_BUTTON_Pin, stReleased},
    [POWER] = {POWER_BUTTON_GPIO_Port, POWER_BUTTON_Pin, stReleased},
};

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Run each button's state machine
 */
void buttonsUpdate() {
    for(uint8_t i = 0; i < (uint8_t)NB_BUTTONS; i++) (*buttons[i].state)(i);
}

/**
 * @brief Check if a button is released
 * 
 * @param button    Button to check
 * @retval 0        Button is pressed
 * @retval 1        Button is released
 */
uint8_t isButtonReleased(button_e button) {
    if(button >= NB_BUTTONS)
        return 0;

    return (buttons[button].state == stReleased);
}

/**
 * @brief Check if a button is pressed or held down
 * 
 * @param button    Button to check
 * @retval 0        Button is released
 * @retval 1        Button is pressed
 */
uint8_t isButtonPressed(button_e button) {
    if(button >= NB_BUTTONS)
        return 0;

    return ((buttons[button].state == stPressed) || (buttons[button].state == stHeldDown));
}

/**
 * @brief Check if a button is held down
 * 
 * @param button    Button to check
 * @retval 0        Button is released or not yet held down
 * @retval 1        Button is held down
 */
uint8_t isButtonHeldDown(button_e button) {
    if(button >= NB_BUTTONS)
        return 0;

    return (buttons[button].state == stHeldDown);
}

/**
 * @brief Check if a button has recently had a rising edge
 * @details Rising edge occurrs when a button goes from released to pressed
 * 
 * @param button    Button to check
 * @retval 0        Button has not had a rising edge
 * @retval 1        Button has had a rising edge
 */
uint8_t buttonHasRisingEdge(button_e button) {
    if(button >= NB_BUTTONS)
        return 0;

    uint8_t tmp                         = buttonsTimers[button].risingEdge_ms;
    buttonsTimers[button].risingEdge_ms = 0;

    return (tmp > 0);
}

/**
 * @brief Check if a button has recently had a falling edge
 * @details Rising edge occurrs when a button goes from pressed to released
 * 
 * @param button    Button to check
 * @retval 0        Button has not had a falling edge
 * @retval 1        Button has had a falling edge
 */
uint8_t buttonHasFallingEdge(button_e button) {
    if(button >= NB_BUTTONS)
        return 0;

    uint8_t tmp                          = buttonsTimers[button].fallingEdge_ms;
    buttonsTimers[button].fallingEdge_ms = 0;

    return (tmp > 0);
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief State in which the button is released
 * 
 * @param button Button for which run the state
 */
static void stReleased(button_e button) {
    //if button released, restart debouncing timer
    if(LL_GPIO_IsInputPinSet(buttons[button].port, buttons[button].pin))
        buttonsTimers[button].debouncing_ms = DEBOUNCE_TIME_MS;

    //if button not pressed for long enough, exit
    if(buttonsTimers[button].debouncing_ms)
        return;

    //set the timer during which rising edge can be read, and get to pressed state
    buttonsTimers[button].risingEdge_ms = EDGEDETECTION_TIME_MS;
    buttonsTimers[button].holding_ms    = HOLDING_TIME_MS;
    buttons[button].state               = stPressed;
}

/**
 * @brief State in which the button is pressed, but not yet held
 * 
 * @param button Button for which run the state
 */
static void stPressed(button_e button) {
    //if button pressed, restart debouncing timer
    if(!LL_GPIO_IsInputPinSet(buttons[button].port, buttons[button].pin)) {
        buttonsTimers[button].debouncing_ms = DEBOUNCE_TIME_MS;

        //if button maintained for long enough, get to held down state
        if(!buttonsTimers[button].holding_ms)
            buttons[button].state = stHeldDown;
    }

    //if button not released for long enough, exit
    if(buttonsTimers[button].debouncing_ms)
        return;

    //set the timer during which falling edge can be read, and get to pressed state
    buttonsTimers[button].fallingEdge_ms = EDGEDETECTION_TIME_MS;
    buttons[button].state                = stReleased;
}

/**
 * @brief State in which the button is held down
 * 
 * @param button Button for which run the state
 */
static void stHeldDown(button_e button) {
    //if button pressed, restart debouncing timer
    if(!LL_GPIO_IsInputPinSet(buttons[button].port, buttons[button].pin))
        buttonsTimers[button].debouncing_ms = DEBOUNCE_TIME_MS;

    //if button not released for long enough, exit
    if(buttonsTimers[button].debouncing_ms)
        return;

    //set the timer during which falling edge can be read, and get to pressed state
    buttonsTimers[button].fallingEdge_ms = EDGEDETECTION_TIME_MS;
    buttons[button].state                = stReleased;
}
