/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file ui.c
 * @brief Implement the display UI
 * @author Gilles Henrard
 * @date 26/07/2025
 */
#include "ui.h"

#include <FreeRTOS.h>
#include <main.h>
#include <portmacro.h>
#include <projdefs.h>
#include <stddef.h>
#include <stdint.h>
#include <stm32f1xx_hal_def.h>
#include <task.h>

#include "dispatcher.h"
#include "errorstack.h"
#include "icons.h"
#include "sensorfusion.h"
#include "st7735s.h"

enum {
    kStackSize = 300U,         ///< Amount of words in the task stack
    kTaskLowPriority = 8U,     ///< FreeRTOS number for a low priority task
    kScreenSizeDivider = 16U,  ///< Number of times the buffer fits in the display
    kFrameBufferSize = (kDisplayWidth * kDisplayHeight) / kScreenSizeDivider,  ///< Size of the frame buffer in bytes
};

static void runUItask(void *argument);
static ErrorCode printMeasurements(Axis axis, int16_t angle_degrees_tenths);
static ErrorCode printCharacter(VerdanaCharacter character, uint8_t x_left, uint8_t y_top);
static ErrorCode fillBackground(void);

static volatile TaskHandle_t task_handle = NULL;  ///< handle of the FreeRTOS task
static Pixel display_buffer[kFrameBufferSize];    ///< Buffer used to send data to the display

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Create the UI FreeRTOS task
 *
 * @return Success
 */
ErrorCode createUItask(void) {
    static StackType_t task_stack[kStackSize] = {0};  ///< Buffer used as the task stack
    static StaticTask_t task_state = {0};             ///< Task state variables}

    // create the static task
    task_handle = xTaskCreateStatic(runUItask, "UI task", kStackSize, NULL, kTaskLowPriority, task_stack, &task_state);
    if (!task_handle) {
        Error_Handler();
    }

    return (kSuccessCode);
}

/**
 * @brief Run the UI task
 *
 * @param argument Unused
 */
static void runUItask(void *argument) {
    UNUSED(argument);
    ErrorCode result;

    attachUItask(task_handle);
    result = configureST7735S();
    if (isError(result)) {
        Error_Handler();
    }

    result = fillBackground();
    if (isError(result)) {
        Error_Handler();
    }

    //turn on backlight
    turnBacklightON();

    TickType_t previous_tick = 0;  //force first update as soon as possible
    while (1) {
        static const uint8_t kRefreshDelayMS = 30U;

        //Wait until a specific amount of milliseconds passed since last update
        vTaskDelayUntil(&previous_tick, pdMS_TO_TICKS(kRefreshDelayMS));

        int16_t latest_values[kNbMessages] = {INT16_MIN, INT16_MIN, INT16_MIN, INT16_MIN};

        //Dequeue all elements to get the latest ones chronologically
        uint16_t max_attempts = UINT16_MAX;
        Message ui_message;
        while (getUImessage(&ui_message) && --max_attempts) {
            if (ui_message.type >= kNbMessages) {
                continue;
            }

            latest_values[ui_message.type] = ui_message.value;
        }

        //if an X angle value is provided, print it
        result = kSuccessCode;
        if (latest_values[kMessageXValue] > INT16_MIN) {
            result = printMeasurements(kXaxis, latest_values[kMessageXValue]);
        }

        //if an Y angle value is provided, print it
        if (latest_values[kMessageYValue] > INT16_MIN) {
            result = printMeasurements(kYaxis, latest_values[kMessageYValue]);
        }

        if (isError(result)) {
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
 * @param angle_degrees_tenths Angle to display, in [0.1°]
 * @retval 0 Success
 * @retval 1 Error while printing a character
 */
static ErrorCode printMeasurements(Axis axis, int16_t angle_degrees_tenths) {
    static const uint8_t kNbMeasureCharacters = 5U;
    static const uint8_t kSecondLineY = 50U;
    static const uint8_t kMultiple10 = 10U;
    static const uint8_t kMultiple100 = 100U;
    uint8_t y_top = (axis == kXaxis ? 0 : kSecondLineY);
    uint8_t to_print[kNbMeasureCharacters];
    ErrorCode result;

    if (angle_degrees_tenths >= 0) {
        to_print[0] = kVerdanaPlus;
    } else {
        to_print[0] = kVerdanaMinus;
        angle_degrees_tenths = (int16_t)-angle_degrees_tenths;
    }
    to_print[1] = (uint8_t)(angle_degrees_tenths / kMultiple100);
    to_print[2] = (uint8_t)((angle_degrees_tenths / kMultiple10) % kMultiple10);
    to_print[3] = kVerdanaDot;
    to_print[4] = (uint8_t)(angle_degrees_tenths % kMultiple10);

    result = kSuccessCode;
    uint8_t character = 0;
    while ((character < kNbMeasureCharacters) && !isError(result)) {
        result = printCharacter((VerdanaCharacter)to_print[character], (kVerdanaNbColumns * character), y_top);
        character++;
    }
    if (isError(result)) {
        return pushErrorCode(result, 1, 1);
    }

    return kSuccessCode;
}

/**
 * @brief Print a Verdana character on screen
 *
 * @param character Character to print
 * @param x_left Left-most coordinate of the character
 * @param y_top Top-most coordinate of the character
 * @return Success
 * @retval 1 Error while sending data to the display
 */
static ErrorCode printCharacter(VerdanaCharacter character, uint8_t x_left, uint8_t y_top) {
    //fill the frame buffer with background pixels
    uncompressCharacter(display_buffer, character);

    ErrorCode result = kSuccessCode;

    const size_t character_size = kVerdanaNbColumns * kVerdanaNbRows * sizeof(Pixel);
    const Area character_area = {x_left, y_top, x_left + kVerdanaNbColumns - 1U, y_top + kVerdanaNbRows - 1U};
    result = sendScreenData(display_buffer, character_size, kFrameBufferSize * sizeof(Pixel), &character_area);
    if (isError(result)) {
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
static ErrorCode fillBackground(void) {
    ErrorCode result = kSuccessCode;

    //fill the frame buffer with background pixels
    for (Pixel pixel = 0; pixel < (Pixel)kFrameBufferSize; pixel++) {
        *(&display_buffer[pixel]) = kDarkCharcoalBigEndian;
    }

    //send whole chunks to the display
    uint8_t lines_sent = 0;
    do {
        uint8_t chunk_height = kFrameBufferSize / kDisplayWidth;
        if ((lines_sent + chunk_height) > kDisplayHeight) {
            chunk_height = (uint8_t)(kDisplayHeight - lines_sent);
        }

        const size_t nb_bytes = chunk_height * kDisplayWidth * sizeof(Pixel);
        const Area chunk_area = {0, lines_sent, kDisplayWidth - 1U, (uint8_t)(lines_sent + chunk_height - 1U)};
        result = sendScreenData(display_buffer, nb_bytes, kFrameBufferSize * sizeof(Pixel), &chunk_area);

        lines_sent += chunk_height;
    } while ((lines_sent < kDisplayHeight) && !isError(result));

    if (isError(result)) {
        return pushErrorCode(result, 2, 1);
    }

    return kSuccessCode;
}
