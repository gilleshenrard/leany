/*
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#include "screen_main.h"

#include <errorstack.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "battery.h"
#include "bitmap.h"
#include "display.h"
#include "fonts.h"
#include "hardware_events.h"
#include "icons.h"
#include "label.h"
#include "leany_std.h"
#include "orientation.inc"
#include "sensorfusion.h"
#include "st7735s.h"
#include "task_imu.h"

enum {
    kAngleMargin = 20U,          ///< Margin used to make sure the angle labels to not overflow the display buffer size
    kAngleStringLength = 6U,     ///< Maximum number in a string representing an angle
    kBatteryFullPercent = 100U,  ///< Value used as a 100% battery level
};

/**
 * Enumeration of all the UI function codes
 */
typedef enum {
    kSetup = 1,              ///< setupUI() function
    kPrintIcon = 2,          ///< printIcon() function
    kPrintVertLine = 3,      ///< printVerticalLine() function
    kPrintBattery = 4,       ///< printBatteryIcon() function
    kTreatMessages = 5,      ///< treatMessages() function
    kPrintMeasurements = 6,  ///< printMeasurements() function
    kFillBackground = 7,     ///< fillBackground() function
    kSetupStatus = 8,        ///< setupStatusBar() function
} FunctionCode;

/**
 * Screen orientation layout
 */
typedef struct {
    uint8_t angle_label_width;  ///< Width of the angle labels in [px]
    uint8_t pitch_label_x;      ///< X coordinate of the pitch label in [px]
    uint8_t pitch_label_y;      ///< Y coordinate of the pitch label in [px]
    uint8_t pitch_icon_x;       ///< X coordinate of the pitch icon in [px]
    uint8_t pitch_icon_y;       ///< Y coordinate of the pitch icon in [px]
    uint8_t roll_label_x;       ///< X coordinate of the roll label in [px]
    uint8_t roll_label_y;       ///< Y coordinate of the roll label in [px]
    uint8_t roll_icon_x;        ///< X coordinate of the roll icon in [px]
    uint8_t roll_icon_y;        ///< Y coordinate of the roll icon in [px]
    uint8_t display_width;      ///< Width of the display (depending on orientation) in [px]
    uint8_t display_height;     ///< Height of the display (depending on orientation) in [px]
    Area separator_area;        ///< Coordinates of the separator line
} Layout;

//private functions
static void treatHoldMessage(uint8_t message_flags[kNbEvents], uint8_t holding);
static ErrorCode printMeasurements(ColourBigEndian foreground_colour);
static void getAngleComponents(Axis axis, int16_t* angle_degrees, int16_t* angle_tenths);

static const Layout kHorizontalLayout =  ///< Horizontal layout values
    {
        .angle_label_width = ((kDisplayWidth / 2U) - kAngleMargin),
        .pitch_label_x = (kAngleMargin / 2U),
        .pitch_icon_x = (kDisplayWidth / 4U) - 1U,
        .pitch_icon_y = 48U,
        .roll_label_x = ((kDisplayWidth + kAngleMargin) / 2U),
        .roll_icon_x = ((kDisplayWidth * 3U) / 4U) - 5U,
        .roll_icon_y = 54U,
        .pitch_label_y = 75U,
        .roll_label_y = 75U,
        .display_width = kDisplayWidth,
        .display_height = kDisplayHeight,
        .separator_area = {.x0 = (kDisplayWidth / 2U), .y0 = 43U, .x1 = (kDisplayWidth / 2U), .y1 = 108U},
};

static const Layout kVerticalLayout =  ///< Vertical layout values
    {
        .angle_label_width = (kDisplayHeight / 2U),
        .pitch_label_x = (kDisplayHeight / 4U),
        .pitch_icon_x = (kDisplayHeight / 2U) - 1U,
        .pitch_icon_y = 40U,
        .roll_label_x = (kDisplayHeight / 4U),
        .roll_icon_x = (kDisplayHeight / 2U) - 5U,
        .roll_icon_y = 107,
        .pitch_label_y = 65U,
        .roll_label_y = 130U,
        .display_width = kDisplayHeight,
        .display_height = kDisplayWidth,
        .separator_area = {.x0 = 30U,
                           .y0 = 22U + ((kDisplayWidth - 22U) / 2U),
                           .x1 = 100U,
                           .y1 = 22U + ((kDisplayWidth - 22U) / 2U)},
};

static Label axis_labels[kNBaxis - 1U];                       ///< Labels representing the angles
static Layout* current_layout = (Layout*)&kHorizontalLayout;  ///< Layout currently used

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Run the UI setup steps
 *
 * @retval 0 Success
 * @retval 1 Invalid frame buffer or current layout
 * @retval 2 Error while filling the background
 * @retval 3 Error while setting up the status bar
 * @retval 4 Error while printing the Pitch icon
 * @retval 5 Error while printing the Roll icon
 * @retval 6 Error while printing the separator line
 */
ErrorCode setupMainScreen(void) {
    ErrorCode result;

    Orientation orientation = kLandscape;
    (void)getDisplayOrientation(&orientation);
    result = changeLayoutOrientation(orientation);
    EXIT_ON_ERROR(result, kSetup, 1);

    axis_labels[kXaxis] = (Label){
        .width_px = current_layout->angle_label_width,
        .x_left = current_layout->pitch_label_x,
        .y_top = current_layout->pitch_label_y,
        .font = &kInterVSemiBold_14_num_descriptor,
    };

    axis_labels[kYaxis] = (Label){
        .width_px = current_layout->angle_label_width,
        .x_left = current_layout->roll_label_x,
        .y_top = current_layout->roll_label_y,
        .font = &kInterVSemiBold_14_num_descriptor,
    };

    result = fillBackground(display_buffer, kFrameBufferSize, orientation, kColourBackground);
    EXIT_ON_ERROR(result, kSetup, 2)

    result = printStatusBar(isIMUmeasurementsHolding(), isIMUzeroed(), current_layout->display_width);
    EXIT_ON_ERROR(result, kSetup, 3)

    result = printIcon(kIconPitch, current_layout->pitch_icon_x, current_layout->pitch_icon_y, kColourAccent);
    EXIT_ON_ERROR(result, kSetup, 4)

    result = printIcon(kIconRoll, current_layout->roll_icon_x, current_layout->roll_icon_y, kColourAccent);
    EXIT_ON_ERROR(result, kSetup, 5)

    result = printRectangle(display_buffer, kFrameBufferSize, &current_layout->separator_area, kColourDecoration);
    EXIT_ON_ERROR(result, kSetup, 6)

    return result;
}

/**
 * Treat the messages pulled from the queue
 *
 * @param message_flags Array of flags indicating which new messages are to be treated
 * @return Any print function return code, or success if no failure
 */
ErrorCode treatMainScreenMessages(uint8_t message_flags[kNbEvents]) {
    uint8_t holding = isIMUmeasurementsHolding();  //holding is used throughout the whole function
    BatteryStatus battery_status;
    Orientation current_orientation = kLandscape;

    //if a measurement hold message is provided, update the icon
    treatHoldMessage(message_flags, holding);

    const ColourBigEndian foreground = (holding ? kColourAccent : kColourForeground);

    ErrorCode result = kSuccessCode;
    for (uint8_t message = 0; message < (uint8_t)kNbEvents; message++) {
        if (!message_flags[message]) {
            continue;
        }

        switch (message) {
            case kEventAngle:
                result = printMeasurements(foreground);
                EXIT_ON_ERROR(result, kTreatMessages, 1)
                break;

            case kEventZero:
                result = printIcon(kIconRelative, kModeIconX, kStatusIconsY, kColourAccent);
                break;

            case kEventCancelZero:
                result = printIcon(kIconAbsolute, kModeIconX, kStatusIconsY, kColourEnabled);
                break;

            case kEventOrientation:
                (void)getDisplayOrientation(&current_orientation);
                result = changeLayoutOrientation(current_orientation);
                EXIT_ON_ERROR(result, kTreatMessages, 2)

                //force screen and angles refresh
                result = setupMainScreen();
                EXIT_ON_ERROR(result, kTreatMessages, 3)
                result = printMeasurements(foreground);
                EXIT_ON_ERROR(result, kTreatMessages, 4)
                break;

            case kEventBatteryStatus:
                getBatteryStatus(&battery_status);
                result = printBatteryIndicator(&battery_status, getBatteryIndicator());
                break;

            case kEventHold:
            default:
                break;
        }

        EXIT_ON_ERROR(result, kTreatMessages, 5)
    }

    return result;
}

/**
 * Update the layout with a new orientation
 *
 * @param new_orientation New orientation
 * @return Success
 */
ErrorCode changeLayoutOrientation(Orientation new_orientation) {
    switch (new_orientation) {
        case kPortrait:
        case kPortrait180:
            current_layout = (Layout*)&kVerticalLayout;
            break;

        case kLandscape:
        case kLandscape180:
        case kNBorientations:
        default:
            current_layout = (Layout*)&kHorizontalLayout;
            break;
    }

    ErrorCode result = st7735sSetOrientation(new_orientation);
    EXIT_ON_ERROR(result, 1, kErrorError)

    return kSuccessCode;
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Handle a hold hardware event, if available
 *
 * @param[out] message_flags Flags array to update if necessary
 * @param holding Measurements holding status
 */
static void treatHoldMessage(uint8_t message_flags[kNbEvents], uint8_t holding) {
    if (!message_flags) {
        return;
    }

    if (!message_flags[kEventHold]) {
        return;
    }

    message_flags[kEventAngle] = 1;
    const ColourBigEndian colour = (holding ? kColourAccent : kColourDisabled);
    (void)printIcon(kIconHold, kHoldIconX, kStatusIconsY, colour);
}

/**
 * Print angles measurements on the display
 *
 * @param foreground_colour Foreground colour of the labels
 * @retval 0 Success
 * @retval 1 Invalid frame buffer or current layout
 * @retval 2 Error while sending an angle to the screen
 */
static ErrorCode printMeasurements(ColourBigEndian foreground_colour) {
    if (!current_layout) {
        return createErrorCode(kPrintMeasurements, 1, kErrorError);
    }

    ErrorCode result = kSuccessCode;
    for (Axis axis = kXaxis; axis < kZaxis; axis++) {
        int16_t angle_degrees = 0;
        int16_t angle_tenths = 0;
        getAngleComponents(axis, &angle_degrees, &angle_tenths);

        //format the angle string
        char angle_string[kAngleStringLength + 1U];
        (void)leany_snprintf(angle_string, kAngleStringLength + 1U, "%+02i.%01i*", angle_degrees, angle_tenths);
        const uint8_t length = (uint8_t)getStringLength(angle_string, kAngleStringLength);

        //print the label
        const Label* label = &axis_labels[axis];
        printLabel(label, angle_string, length, foreground_colour);
        EXIT_ON_ERROR(result, kPrintMeasurements, 2)
    }

    return result;
}

/**
 * Split an angle value into degrees and tenths of degrees components
 *
 * @param axis Axis from which to retrieve the angle
 * @param[out] angle_degrees Integer degrees component (range: -99 to +99)
 * @param[out] angle_tenths Tenths of degree component (range: 0 to 9, always positive)
 */
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
static void getAngleComponents(Axis axis, int16_t* angle_degrees, int16_t* angle_tenths) {
    const int16_t angle_degrees_tenths = getAngleDegreesTenths(axis);

    //compute clamped units and tenths values of the angle
    const int16_t max_angle_tenth = 999;
    int16_t angle_clamped = angle_degrees_tenths;
    if (angle_clamped > max_angle_tenth) {
        angle_clamped = max_angle_tenth;
    }
    if (angle_clamped < -max_angle_tenth) {
        angle_clamped = (int16_t)-max_angle_tenth;
    }

    // NOLINTBEGIN (cppcoreguidelines-avoid-magic-numbers)
    *angle_degrees = (int16_t)(angle_clamped / 10);
    *angle_tenths = (int16_t)((angle_clamped < 0 ? -angle_clamped : angle_clamped) % 10);
    // NOLINTEND (cppcoreguidelines-avoid-magic-numbers)
}
