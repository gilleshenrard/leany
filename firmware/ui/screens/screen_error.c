/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file screen_error.c
 * @brief Implement the error screen
 * @author Gilles Henrard
 */
#include "screen_error.h"

#include <leany_std.h>
#include <main.h>
#include <portmacro.h>
#include <stdint.h>
#include <stm32f1xx_hal.h>

#include "bitmap.h"
#include "display.h"
#include "errorstack.h"
#include "fonts.h"
#include "icons.h"
#include "label.h"
#include "orientation.inc"
#include "st7735s.h"
#include "task_imu.h"

enum {
    kErrorLabelSize = 8U,               ///< Number of characters in the error code string
    kErrorLabelWidthPx = 100U,          ///< Width of the error code label in [px]
    kErrorLabelYoffsetPx = 42U,         ///< Y offset of the error code label, relative to top bar, in [px]
    kErrorStackLabelSize = 19U,         ///< Number of characters in the error stack string
    kErrorStackLabelWidthPx = 120U,     ///< Width of the error stack label in [px]
    kErrorStackLabelYoffsetPx = 62U,    ///< Y offset of the error stack label, relative to top bar, in [px]
    kWarningIconYoffsetPx = 14U,        ///< Y offset of the warning icon, relative to top bar, in [px]
    kNbAnimatedBars = 3U,               ///< Number of bars in the animation
    kNbColourMatchings = 4U,            ///< Number of colours associated with a level
    kAnimationRefreshPeriod_ms = 200U,  ///< Number of milliseconds between animation refresh
};

typedef enum {
    kSetup = 1,                ///< setupErrorScreen() function
    kPrintErrorLabel = 2,      ///< printErrorCodeLabel() function
    kPrintErrorCodeLabel = 3,  ///< printErrorCodeLabel() function
    kUpdateAnimation = 4,      ///< updateErrorAnimation() function
} FunctionCode;

/**
 * A bar in the animation
 */
typedef struct {
    Area area;      ///< Area of the rectangle representing the bar
    uint8_t level;  ///< Opacity level (between dark and bright)
} AnimatedBar;

static ErrorCode printErrorCodeLabel(uint32_t error_code);
static ErrorCode printErrorStackLabel(const ErrorCode* error);

static ErrorCode result;                   ///< Error result buffer
static Label errorcode_label;              ///< Label displaying the error code
static Label errorstack_label;             ///< Label displaying the error stack
static int8_t animation_direction = -1;    ///< Increment of the animation index (1 or -1)
static int8_t current_bar_lit = 2;         ///< Index of the current fully lit bar
static TickType_t latest_update_tick = 0;  ///< Latest tick at which the animation has been updated
static ColourBigEndian colour_matchings[kNbColourMatchings] = {  ///< Colours assigned to each opacity level
    [0] = kColourDisabled,
    [1] = kColourCriticalOpacity28,
    [2] = kColourCriticalOpacity50,
    [3] = kColourCritical};
static AnimatedBar bars[kNbAnimatedBars] =  ///< Bars of the animation
    {

        {.area = {.x0 = 58U, .y0 = 103U, .x1 = 69U, .y1 = 105U},  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
         .level = 1},
        {.area = {.x0 = 74U, .y0 = 103U, .x1 = 85U, .y1 = 105U},  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
         .level = 2},
        {.area = {.x0 = 90U, .y0 = 103U, .x1 = 101U, .y1 = 105U},  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
         .level = 3},
};

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Print all the error screen assets on screen
 *
 * @param error Error to print 
 * @retval 0 Success
 * @retval 1 Error while filling the background
 * @retval 2 Error while printing the status bar
 * @retval 3 Error while printing the error code label
 */
ErrorCode setupErrorScreen(const ErrorCode* error) {
    //no orientation change in the IMU, as it is reused when entering the main screen
    (void)st7735sSetOrientation(kLandscape);

    result = fillBackground(display_buffer, kFrameBufferSize, kLandscape, kColourBackground);
    EXIT_ON_ERROR(result, kSetup, 1)

    result = printStatusBar(isIMUmeasurementsHolding(), isIMUzeroed(), kDisplayWidth);
    EXIT_ON_ERROR(result, kSetup, 2)

    const uint8_t warning_offset_x = (uint8_t)((kDisplayWidth - getIconWidth(kIconWarning)) / 2U);
    result = printIcon(kIconWarning, warning_offset_x, kStatusBarHeight + kWarningIconYoffsetPx, kColourCritical);
    EXIT_ON_ERROR(result, kSetup, 5)

    result = printErrorCodeLabel(error->dword);
    EXIT_ON_ERROR(result, kSetup, 3)

    result = printErrorStackLabel(error);
    EXIT_ON_ERROR(result, kSetup, 4)

    for (uint8_t bar = 0; bar < (uint8_t)kNbAnimatedBars; bar++) {
        result = printRectangle(display_buffer, kFrameBufferSize, &bars[bar].area, colour_matchings[bars[bar].level]);
        EXIT_ON_ERROR(result, kSetup, 5)
    }

    latest_update_tick = HAL_GetTick();

    return kSuccessCode;
}

/**
 * Update the Error screen line animation
 *
 * @retval 0 Success
 * @retval 1 Error while printing a bar on screen
 */
ErrorCode updateErrorAnimation(void) {
    //if refresh period not elapsed yet, exit
    if (!timeout(latest_update_tick, kAnimationRefreshPeriod_ms)) {
        return kSuccessCode;
    }
    latest_update_tick = HAL_GetTick();

    //update animation direction when the fully lit bar hit a border
    if (current_bar_lit == (kNbAnimatedBars - 1U)) {
        animation_direction = -1;
    }
    if (current_bar_lit == 0) {
        animation_direction = 1;
    }

    //update the current fully lit bar index
    current_bar_lit = (int8_t)(current_bar_lit + animation_direction);

    //update all the bars
    for (uint8_t bar = 0; bar < (uint8_t)kNbAnimatedBars; bar++) {
        //reset the current bar to fully lit, decrease opacity of the rest, ignore bars at minimum level
        if (bar == (uint8_t)current_bar_lit) {
            bars[bar].level = (kNbColourMatchings - 1);
        } else if (bars[bar].level > 0) {
            bars[bar].level--;
        } else {
            continue;
        }

        //print the bars
        result = printRectangle(display_buffer, kFrameBufferSize, &bars[bar].area, colour_matchings[bars[bar].level]);
        EXIT_ON_ERROR(result, kUpdateAnimation, 1)
    }

    return kSuccessCode;
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Print the error code label on screen
 *
 * @param error_code Error code
 * @retval 0 Success
 * @retval 1 Error while printing the label
 */
static ErrorCode printErrorCodeLabel(uint32_t error_code) {
    errorcode_label = (Label){
        .width_px = kErrorLabelWidthPx,
        .x_left = ((kDisplayWidth - kErrorLabelWidthPx) / 2U),
        .y_top = kStatusBarHeight + kErrorLabelYoffsetPx,
        .font = &kInterVSemiBold_14_num_descriptor,
    };

    char error_string[kErrorLabelSize + 1U];
    leany_snprintf(error_string, kErrorLabelSize + 1U, "%08X", error_code);

    result = printLabel(&errorcode_label, error_string, kErrorLabelSize, kColourCritical);
    EXIT_ON_ERROR(result, kPrintErrorCodeLabel, 1)

    return kSuccessCode;
}

/**
 * Print the error stack label, which disassembles the error code, on screen
 *
 * @param error Error object to disassemble
 * @retval 0 Success
 * @retval 1 Error while printing the label
 */
static ErrorCode printErrorStackLabel(const ErrorCode* error) {
    errorstack_label = (Label){
        .width_px = kErrorStackLabelWidthPx,
        .x_left = ((kDisplayWidth - kErrorStackLabelWidthPx) / 2U),
        .y_top = kStatusBarHeight + kErrorStackLabelYoffsetPx,
        .font = &kInterV_7pt_alpha_descriptor,
    };

    char error_string[kErrorStackLabelSize + 1U];
    leany_snprintf(error_string, kErrorStackLabelSize + 1U, "MOD%02u | FN%02u | L%02u", error->module_id,
                   error->function_id, error->layer0);

    result = printLabel(&errorstack_label, error_string, (kErrorStackLabelSize + 1U), kColourDisabled);
    EXIT_ON_ERROR(result, kPrintErrorCodeLabel, 1)

    return kSuccessCode;
}
