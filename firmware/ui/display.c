/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file display.c
 * @brief Implement the display control functions
 * @author Gilles Henrard
 */
#include "display.h"

#include <stddef.h>
#include <stdint.h>

#include "bitmap.h"
#include "errorstack.h"
#include "fonts.h"
#include "icons.h"
#include "label.h"
#include "leany_std.h"
#include "orientation.inc"
#include "st7735s.h"
#include "task_battery.h"

enum {
    kPercentLength = 4U,         ///< Maximum number in a string representing a percentage
    kBatteryIconXoffset = 26U,   ///< Battery icon offset from the indicator X coordinate
    kBatteryLabelWidthPX = 25U,  ///< Battery level label width
    kBatteryScreenOffset = 45U,  ///< Battery indicator offset from the right side of screen
};

/**
 * Display module function IDs
 */
typedef enum {
    kFillBackground = 1U,  ///< fillBackground() function code
    kPrintVertLine = 2U,   ///< printRectangle() function code
    kPrintLabel = 3,       ///< printLabel() function
    kPrintIcon = 4,        ///< printIcon() function
    kSetupStatus = 5,      ///< setupStatusBar() function
} FunctionCode;

static BatteryIndicator battery_indicator;  ///< Battery indicator widget

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Print a label on screen
 *
 * @param label Label to print
 * @param string String to display in the label
 * @param string_size Number of characters in the string
 * @param foreground_colour Foreground colour of the label
 * @retval 0 Success
 * @retval 1 Error while uncompressing the label
 * @retval 2 Error while sending the data to the screen
 */
ErrorCode printLabel(const Label* label, const char* string, uint8_t string_size, ColourBigEndian foreground_colour) {
    ErrorCode result = uncompressLabel(label, display_buffer, string, string_size, foreground_colour);
    EXIT_ON_ERROR(result, kPrintLabel, 1)

    //send the display buffer data to the display
    const uint8_t character_height_px = label->font->height_px;
    const Area character_area = {
        // clang-format off
        label->x_left,
        label->y_top,
        (uint8_t)(label->x_left + label->width_px - (uint8_t)1U),
        (uint8_t)(label->y_top + character_height_px - 1U),
        // clang-format on
    };

    result = sendScreenData(display_buffer, (label->width_px * character_height_px * sizeof(Pixel)),
                            kFrameBufferSize * sizeof(Pixel), &character_area);
    EXIT_ON_ERROR(result, kPrintLabel, 2)
    return kSuccessCode;
}

/**
 * Fill the screen background
 *
 * @param buffer Display buffer to use to fill the screen background
 * @param buffer_size Size of the display buffer in [byte]
 * @param orientation Current screen orientation
 * @param colour Colour with which to fill the background
 * @retval 0 Success
 * @retval 1 Error while sending data to the display
 */
ErrorCode fillBackground(Pixel* buffer, size_t buffer_size, Orientation orientation, ColourBigEndian colour) {
    ErrorCode result = kSuccessCode;

    //fill the frame buffer with background pixels
    for (Pixel pixel = 0; pixel < (Pixel)buffer_size; pixel++) {
        *(&buffer[pixel]) = (Pixel)colour;
    }

    uint8_t display_width = kDisplayWidth;
    uint8_t display_height = kDisplayHeight;
    if ((orientation == kPortrait) || (orientation == kPortrait180)) {
        display_width = kDisplayHeight;
        display_height = kDisplayWidth;
    }

    //send whole chunks to the display
    uint8_t lines_sent = 0;
    do {
        uint8_t chunk_height = (uint8_t)(buffer_size / display_width);
        if ((lines_sent + chunk_height) > display_height) {
            chunk_height = (uint8_t)(display_height - lines_sent);
        }

        const size_t nb_bytes = chunk_height * display_width * sizeof(Pixel);
        const Area chunk_area = {0, lines_sent, (uint8_t)(display_width - 1U),
                                 (uint8_t)(lines_sent + chunk_height - 1U)};
        result = sendScreenData(buffer, nb_bytes, buffer_size * sizeof(Pixel), &chunk_area);

        lines_sent += chunk_height;
    } while ((lines_sent < display_height) && !isError(result));

    EXIT_ON_ERROR(result, kFillBackground, 1)

    return kSuccessCode;
}

/**
 * Print a colour rectangle on the display
 *
 * @param buffer Buffer to use
 * @param buffer_size Maximum size of the buffer
 * @param area Area to print on the display
 * @param colour Colour of the rectangle
 * @retval 0 Success
 * @retval 1 Area is NULL
 * @retval 2 Area goes either right to left, or bottom to top
 * @retval 3 Area larger than the buffer
 * @retval 4 Error while sending the buffer to the display
 */
ErrorCode printRectangle(Pixel* buffer, size_t buffer_size, const Area* area, ColourBigEndian colour) {
    if (!area) {
        return createErrorCode(kPrintVertLine, 1, kErrorError);
    }

    //check if the area goes left to right, and top to bottom
    if ((area->x0 > area->x1) || (area->y0 > area->y1)) {
        return createErrorCode(kPrintVertLine, 2, kErrorError);
    }

    //compute the width and height, and check the area height
    const uint8_t width_px = (uint8_t)(area->x1 - area->x0) + 1U;
    const uint8_t height_px = (uint8_t)(area->y1 - area->y0) + 1U;
    const size_t size_px = (width_px * height_px);
    if (size_px > buffer_size) {
        return createErrorCode(kPrintVertLine, 3, kErrorError);
    }

    //fill the rectangle in the buffer
    for (uint8_t row = 0; row < height_px; row++) {
        Pixel* row_buffer = &buffer[row * width_px];
        for (uint8_t column = 0; column < width_px; column++) {
            *row_buffer = (Pixel)colour;
            row_buffer++;
        }
    }

    //send the buffer to the display
    ErrorCode result = sendScreenData(buffer, (size_px * sizeof(Pixel)), buffer_size, area);
    EXIT_ON_ERROR(result, kPrintVertLine, 4)

    return kSuccessCode;
}

/**
 * Setup the status bar screen section
 *
 * @param holding_status Current status of the measurements holding
 * @param zeroing_status Current zeroing status
 * @param screen_width_px Width of the screen in [px], depending on the layout
 * @retval 0 Success
 * @retval 1 Error while printing the Hold icon
 * @retval 2 Error while printing the Measurement mode icon
 * @retval 3 Error while retrieving the battery status
 * @retval 4 Error while printing the Battery icon and level
 * @retval 5 Error while printing the status bar separator
 */
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
ErrorCode printStatusBar(uint8_t holding_status, uint8_t zeroing_status, uint8_t screen_width_px) {
    ErrorCode result = kSuccessCode;

    result = printIcon(kIconHold, kHoldIconX, kStatusIconsY, (holding_status ? kColourAccent : kColourDisabled));
    EXIT_ON_ERROR(result, kSetupStatus, 1)

    const Icon type_icon = (zeroing_status ? kIconRelative : kIconAbsolute);
    result = printIcon(type_icon, kModeIconX, kStatusIconsY, kColourEnabled);
    EXIT_ON_ERROR(result, kSetupStatus, 2)

    BatteryStatus battery_status;
    result = getBatteryStatus(&battery_status);
    EXIT_ON_ERROR(result, kSetupStatus, 3)

    battery_indicator = (BatteryIndicator){.x = screen_width_px - kBatteryScreenOffset,
                                           .y = kStatusIconsY,
                                           .label = {
                                               .width_px = 25U,  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
                                               .x_left = kStatusIconsY,
                                               .y_top = kStatusIconsY,
                                               .font = &kInterV_7pt_alpha_descriptor,
                                           }};
    result = printBatteryIndicator(&battery_status, &battery_indicator);
    EXIT_ON_ERROR(result, kSetupStatus, 4)

    const Area top_line = {.x0 = 0U, .y0 = kStatusBarHeight, .x1 = screen_width_px, .y1 = kStatusBarHeight};
    result = printRectangle(display_buffer, kFrameBufferSize, &top_line, kColourDecoration);
    EXIT_ON_ERROR(result, kSetupStatus, 5)

    return kSuccessCode;
}

/**
 * Print an icon on screen
 *
 * @param icon Icon to print
 * @param x_left_px X coordinate at which print the icon
 * @param y_top_px Y coordinate at which print the icon
 * @param foreground_colour Icon foreground colour
 * @retval 0 Success
 * @retval 1 Error while uncompressing the icon in the display buffer
 * @retval 2 Error while sending the data to the screen
 */
ErrorCode printIcon(Icon icon, uint8_t x_left_px, uint8_t y_top_px, ColourBigEndian foreground_colour) {
    ErrorCode result = kSuccessCode;

    const uint8_t icon_width_px = getIconWidth(icon);
    Bitmap metadata = {
        .output_buffer = display_buffer, .x_offset_px = 0, .y_offset_px = 0, .container_width_px = icon_width_px};

    result = uncompressIcon(icon, &metadata, foreground_colour);
    EXIT_ON_ERROR(result, kPrintIcon, 1)

    const uint8_t icon_height_px = getIconHeight(icon);
    const Area character_area = {x_left_px, y_top_px, (uint8_t)(x_left_px + icon_width_px - 1U),
                                 (uint8_t)(y_top_px + icon_height_px - 1U)};
    result = sendScreenData(display_buffer, (icon_width_px * icon_height_px * sizeof(Pixel)), kFrameBufferSize,
                            &character_area);
    EXIT_ON_ERROR(result, kPrintIcon, 2)

    return result;
}

/**
 * Print the battery icon on screen
 *
 * @param status Battery status
 * @param indicator Indicator representing the status
 * @retval 0 Success
 * @retval 1 Error while uncompressing the icon in the display buffer
 * @retval 2 Error while sending the data to the screen
 */
ErrorCode printBatteryIndicator(const BatteryStatus* status, BatteryIndicator* indicator) {
    ErrorCode result = kSuccessCode;

    //compute the battery icon dimensions
    const uint8_t icon_width_px = getIconWidth(kIconBattery);
    Bitmap metadata = {
        .output_buffer = display_buffer, .x_offset_px = 0, .y_offset_px = 0, .container_width_px = icon_width_px};

    //uncompress the battery icon and bars
    result = uncompressIcon(kIconBattery, &metadata, (status->charging ? kColourAccent : kColourEnabled));
    EXIT_ON_ERROR(result, kPrintIcon, 2)
    drawBatteryLevelBars(display_buffer, status->level_percents, (bool)status->charging);

    //send the battery icon to the screen
    const uint8_t icon_height_px = getIconHeight(kIconBattery);
    const Area character_area = {
        // clang-format off
        indicator->x + kBatteryIconXoffset,
        indicator->y,
        (uint8_t)(indicator->x + kBatteryIconXoffset + icon_width_px - (uint8_t)1U),
        (uint8_t)(indicator->y + icon_height_px - 1U)
        // clang-format on
    };
    result = sendScreenData(display_buffer, (icon_width_px * icon_height_px * sizeof(Pixel)), kFrameBufferSize,
                            &character_area);
    EXIT_ON_ERROR(result, kPrintIcon, 3)

    //format the percentage string
    char percent_string[kPercentLength + 1U];
    (void)leany_snprintf(percent_string, kPercentLength + 1U, "%u%%", status->level_percents);
    const uint8_t length = (uint8_t)getStringLength(percent_string, kPercentLength);

    //uncompress the string
    const uint8_t label_height = indicator->label.font->height_px;
    indicator->label.x_left = indicator->x;
    uncompressLabel(&indicator->label, display_buffer, percent_string, length,
                    (status->charging ? kColourAccent : kColourEnabled));

    //send the percentage string to the screen
    const Area percentage_area = {
        // clang-format off
        indicator->x,
        indicator->y,
        (uint8_t)(indicator->x + kBatteryLabelWidthPX - 1U),
        (uint8_t)(indicator->y + label_height - 1U)
        // clang-format on
    };
    result = sendScreenData(display_buffer, (kBatteryLabelWidthPX * label_height * sizeof(Pixel)), kFrameBufferSize,
                            &percentage_area);

    return result;
}

/**
 * Get the battery indicator to use
 *
 * @return Indicator
 */
BatteryIndicator* getBatteryIndicator(void) { return &battery_indicator; }
