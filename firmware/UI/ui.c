/**
 * @file ui.c
 * @brief Implement the display UI
 * @author Gilles Henrard
 * @date 13/06/2025
 */
#include "ui.h"
#include <errorstack.h>
#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "ST7735S.h"
#include "buttons.h"
#include "icons.h"
#include "main.h"
#include "memsBMI270.h"
#include "portmacro.h"
#include "projdefs.h"
#include "queue.h"
#include "stm32f1xx_hal_def.h"
#include "task.h"

enum {
    STACK_SIZE         = 300U,  ///< Amount of words in the task stack
    TASK_LOW_PRIORITY  = 8U,    ///< FreeRTOS number for a low priority task
    SCREENSIZE_DIVIDER = 16U,   ///< Number of times the buffer fits in the display
    FRAME_BUFFER_SIZE  = (DISPLAY_WIDTH * DISPLAY_HEIGHT) / SCREENSIZE_DIVIDER,  ///< Size of the frame buffer in bytes
};

static void        runUItask(void *argument);
static errorCode_u printMeasurements(axis_e axis);
static errorCode_u printCharacter(verdanaCharacter_e character, uint8_t xLeft, uint8_t yTop);
static errorCode_u fillBackground(void);

static volatile TaskHandle_t taskHandle = NULL;                 ///< handle of the FreeRTOS task
static pixel_t               displayBuffer[FRAME_BUFFER_SIZE];  ///< Buffer used to send data to the display

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Create the UI FreeRTOS task
 *
 * @return Success
 */
errorCode_u createUItask(void) {
    static StackType_t  taskStack[STACK_SIZE] = {0};  ///< Buffer used as the task stack
    static StaticTask_t taskState             = {0};  ///< Task state variables}

    // create the static task
    taskHandle = xTaskCreateStatic(runUItask, "UI task", STACK_SIZE, NULL, TASK_LOW_PRIORITY, taskStack, &taskState);
    if(!taskHandle) {
        Error_Handler();
    }

    return (ERR_SUCCESS);
}

/**
 * @brief Run the UI task
 *
 * @param argument Unused
 */
static void runUItask(void *argument) {
    UNUSED(argument);
    errorCode_u result;

    attachUItask(taskHandle);
    result = configureST7735S();
    if(isError(result)) {
        Error_Handler();
    }

    result = fillBackground();
    if(isError(result)) {
        Error_Handler();
    }

    //turn on backlight
    turnBacklightON();

    TickType_t previousTick = 0;
    while(1) {
        static const uint8_t REFRESH_DELAY_MS = 30U;

        vTaskDelayUntil(&previousTick, pdMS_TO_TICKS(REFRESH_DELAY_MS));

        if(anglesChanged()) {
            result = printMeasurements(X_AXIS);
            if(isError(result)) {
                result = pushErrorCode(result, 1, 1);
            }
            if(!isError(result)) {
                result = printMeasurements(Y_AXIS);
                if(isError(result)) {
                    result = pushErrorCode(result, 1, 2);
                }
            }
        }

        if(buttonHasRisingEdge(BTN_ZERO)) {
            bmi270ZeroDown();
        }

        if(isButtonHeldDown(BTN_ZERO)) {
            bmi270CancelZeroing();
        }

        if(isError(result)) {
            Error_Handler();
        }
    }
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Print angles measurements on the display
 *
 * @param axis Axis for which display the measurements
 * @retval 0 Success
 * @retval 1 Error while printing a character
 */
static errorCode_u printMeasurements(axis_e axis) {
    static const uint8_t NB_MEAS_CHARACTERS = 5U;
    static const uint8_t SECOND_LINE_Y      = 50U;
    static const uint8_t MULTIPLE_10        = 10U;
    static const uint8_t MULTIPLE_100       = 100U;
    int16_t              measurement        = getAngleDegreesTenths(axis);
    uint8_t              yTop               = (axis == X_AXIS ? 0 : SECOND_LINE_Y);
    uint8_t              toPrint[NB_MEAS_CHARACTERS];
    errorCode_u          result;

    if(measurement >= 0) {
        toPrint[0] = VERDANA_PLUS;
    } else {
        toPrint[0]  = VERDANA_MIN;
        measurement = (int16_t)-measurement;
    }
    toPrint[1] = (uint8_t)(measurement / MULTIPLE_100);
    toPrint[2] = (uint8_t)((measurement / MULTIPLE_10) % MULTIPLE_10);
    toPrint[3] = VERDANA_DOT;
    toPrint[4] = (uint8_t)(measurement % MULTIPLE_10);

    result            = ERR_SUCCESS;
    uint8_t character = 0;
    while((character < NB_MEAS_CHARACTERS) && !isError(result)) {
        result = printCharacter((verdanaCharacter_e)toPrint[character], (VERDANA_NB_COLUMNS * character), yTop);
        character++;
    }
    if(isError(result)) {
        return pushErrorCode(result, 1, 1);
    }

    return ERR_SUCCESS;
}

/**
 * @brief Print a Verdana character on screen
 *
 * @param character Character to print
 * @param xLeft Left-most coordinate of the character
 * @param yTop Top-most coordinate of the character
 * @return Success
 * @retval 1 Error while sending data to the display
 */
static errorCode_u printCharacter(verdanaCharacter_e character, uint8_t xLeft, uint8_t yTop) {
    //fill the frame buffer with background pixels
    uncompressCharacter(displayBuffer, character);

    errorCode_u result = ERR_SUCCESS;

    const size_t characterSize = VERDANA_NB_COLUMNS * VERDANA_NB_ROWS * sizeof(pixel_t);
    const area_t characterArea = {xLeft, yTop, xLeft + VERDANA_NB_COLUMNS - 1U, yTop + VERDANA_NB_ROWS - 1U};
    result = sendScreenData(displayBuffer, characterSize, FRAME_BUFFER_SIZE * sizeof(pixel_t), &characterArea);
    if(isError(result)) {
        return pushErrorCode(result, 1, 1);
    }
    return result;
}

/**
 * Fill the screen background
 *
 * @retval 0 Success
 * @retval 1 Error while sending data to the display
 */
static errorCode_u fillBackground(void) {
    errorCode_u result = ERR_SUCCESS;

    //fill the frame buffer with background pixels
    for(pixel_t pixel = 0; pixel < (pixel_t)FRAME_BUFFER_SIZE; pixel++) {
        *(&displayBuffer[pixel]) = DARK_CHARCOAL_BIGENDIAN;
    }

    //send whole chunks to the display
    uint8_t linesSent = 0;
    do {
        uint8_t chunkHeight = FRAME_BUFFER_SIZE / DISPLAY_WIDTH;
        if((linesSent + chunkHeight) > DISPLAY_HEIGHT) {
            chunkHeight = (uint8_t)(DISPLAY_HEIGHT - linesSent);
        }

        const size_t nbBytes   = chunkHeight * DISPLAY_WIDTH * sizeof(pixel_t);
        const area_t chunkArea = {0, linesSent, DISPLAY_WIDTH - 1U, (uint8_t)(linesSent + chunkHeight - 1U)};
        result = sendScreenData(displayBuffer, nbBytes, FRAME_BUFFER_SIZE * sizeof(pixel_t), &chunkArea);

        linesSent += chunkHeight;
    } while((linesSent < DISPLAY_HEIGHT) && !isError(result));

    if(isError(result)) {
        return pushErrorCode(result, 2, 1);
    }

    return ERR_SUCCESS;
}
