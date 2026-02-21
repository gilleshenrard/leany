/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file screen_system.c
 * @brief Implement the system information screen
 * @author Gilles Henrard
 */
#include "screen_system.h"

#include <stddef.h>
#include <stdint.h>

#include "bitmap.h"
#include "display.h"
#include "errorstack.h"
#include "fonts.h"
#include "hal_adc.h"
#include "hardware_events.h"
#include "label.h"
#include "leany_std.h"
#include "orientation.inc"
#include "softversion.h"
#include "st7735s.h"

enum {
    kTitleSectionHeightPx = 23U,  ///< Height of the title section in [px]
    kSectionHeightPx = 34U,       ///< Height of the info sections in [px]
    kMaxTemperatureSize = 6U,     ///< Maximum number of characters to format temperature
    kInformationTitleSize = 19U,  ///< Maximum size of the information title
};

/**
 * Enumeration of all the UI function codes
 */
typedef enum {
    kSetup = 1,         ///< setupUI() function
    kInitSections = 3,  ///< initialiseSections() function
    kPrintSection = 4,  ///< printSection() function
} FunctionCode;

/**
 * Section types
 */
typedef enum {
    kFirmware = 0,  ///< Firmware version section
    kGitHash,       ///< Git commit hash section
    kTemperature,   ///< MCU internal temperature section
    kNbSections,    ///< Number of sections
} SectionCode;

/**
 * System information screen section
 */
typedef struct {
    const char* title;   ///< Section title string
    Label title_label;   ///< Section title label
    uint8_t title_size;  ///< Maximum size of the title
    const char* value;   ///< Section value string
    Label value_label;   ///< Section value label
    uint8_t value_size;  ///< Maximum size of the value
} Section;

//private functions
static ErrorCode initialiseSections(Section sections[kNbSections]);
static ErrorCode printSection(const Section* section, uint8_t index, uint8_t label_margin_px);
static ErrorCode updateTemperature(void);

//variables
static Label system_info_label;                              ///< Label representing the battery percentage
static Section sections[kNbSections];                        ///< Array of display sections
static char internal_temperature[kMaxTemperatureSize + 1U];  ///< String holding the MCU internal temperature in [°C]

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Setup and print the system screen
 * @retval 0 Success
 * @retval 1 Error while filling the background
 * @retval 2 Error while printing the main title label
 * @retval 3 Error while printing the title section separator
 * @retval 4 Error while initialising the sections
 * @retval 5 Error while updating the temperature
 */
ErrorCode setupSystemScreen(void) {
    ErrorCode result;

    const uint8_t title_label_y = (uint8_t)((kTitleSectionHeightPx - kInterV_7pt_alpha_descriptor.height_px) / 2U);
    system_info_label = (Label){
        .width_px = 110U,        // NOLINT (cppcoreguidelines-avoid-magic-numbers)
        .x_left = 9U,            // NOLINT (cppcoreguidelines-avoid-magic-numbers)
        .y_top = title_label_y,  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
        .font = &kInterV_7pt_alpha_descriptor,
    };

    //no orientation change in the IMU, as it is reused when entering the main screen
    (void)st7735sSetOrientation(kPortrait);

    result = fillBackground(display_buffer, kFrameBufferSize, kPortrait, kColourBackground);
    EXIT_ON_ERROR(result, kSetup, 1)

    const char information_title[kInformationTitleSize] = "SYSTEM INFORMATION";
    result = printLabel(&system_info_label, information_title, kInformationTitleSize, kColourEnabled);
    EXIT_ON_ERROR(result, kSetup, 2)

    const Area top_line = {
        .x0 = 0U, .y0 = (kTitleSectionHeightPx - 1U), .x1 = kDisplayHeight, .y1 = (kTitleSectionHeightPx - 1U)};
    result = printRectangle(display_buffer, kFrameBufferSize, &top_line, kColourDecoration);
    EXIT_ON_ERROR(result, kSetup, 3)

    result = initialiseSections(sections);
    EXIT_ON_ERROR(result, kSetup, 4)

    result = updateTemperature();
    EXIT_ON_ERROR(result, kSetup, 5)

    return kSuccessCode;
}

/**
 * Treat the messages pulled from the queue
 *
 * @param message_flags Array of flags indicating which new messages are to be treated
 * @return Any print function return code, or success if no failure
 */
ErrorCode treatSystemScreenMessages(const uint8_t message_flags[kNbEvents]) {
    if (!message_flags[kEventTemperature]) {
        return kSuccessCode;
    }

    (void)updateTemperature();

    return kSuccessCode;
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Inialise and print all system info sections
 *
 * @param sections_array Array of sections to populate and print
 * @retval 0 Success
 * @retval 1 Error while printing a section
 */
static ErrorCode initialiseSections(Section sections_array[kNbSections]) {
    //initialise the labels
    // NOLINTBEGIN (cppcoreguidelines-avoid-magic-numbers)
    sections_array[kFirmware] =
        (Section){.title = "VERSION", .title_size = 8U, .value = kFirmwareVersion, .value_size = kFirmwareVersionSize};
    sections_array[kGitHash] =
        (Section){.title = "HASH", .title_size = 5U, .value = kGitCommitHash, .value_size = kCommitHashSize};
    sections_array[kTemperature] = (Section){
        .title = "TEMP.", .title_size = 6U, .value = internal_temperature, .value_size = (kMaxTemperatureSize + 1U)};
    // NOLINTEND (cppcoreguidelines-avoid-magic-numbers)

    //precompute coordinates
    const uint8_t label_margin_px = 7U;
    const uint8_t label_width_px = (kDisplayHeight / 2U);
    const uint8_t label_y = (uint8_t)((kSectionHeightPx - kInterV_7pt_alpha_descriptor.height_px) / 2U);
    const uint8_t label_top = (kTitleSectionHeightPx + label_y);

    for (uint8_t index = 0; index < (uint8_t)kNbSections; index++) {
        Section* section = &sections_array[index];

        //initialise the title label
        section->title_label = (Label){.font = &kInterV_7pt_alpha_descriptor,
                                       .width_px = label_width_px,
                                       .x_left = label_margin_px,
                                       .y_top = label_top + (uint8_t)(index * kSectionHeightPx),
                                       .alignment = kAlignmentLeft};

        //initialise the value label
        section->value_label = (Label){.font = &kInterV_7pt_alpha_descriptor,
                                       .width_px = label_width_px,
                                       .x_left = (kDisplayHeight - label_width_px - label_margin_px),
                                       .y_top = label_top + (uint8_t)(index * kSectionHeightPx),
                                       .alignment = kAlignmentRight};

        ErrorCode result = printSection(section, index, label_margin_px);
        EXIT_ON_ERROR(result, kInitSections, index)
    }

    return kSuccessCode;
}

/**
 * Print a system information section
 *
 * @param section Section to print
 * @param index Index of the section on screen
 * @param label_margin_px Margin of each label in [px]
 * @retval 0 Success
 * @retval 1 Error while printing the title label
 * @retval 2 Error while printing the value label
 * @retval 3 Error while printing the separator line
 */
static ErrorCode printSection(const Section* section, uint8_t index, uint8_t label_margin_px) {
    ErrorCode result = kSuccessCode;

    //print the labels and separator
    result = printLabel(&section->title_label, section->title,
                        (uint8_t)getStringLength(section->title, section->title_size), kColourEnabled);
    EXIT_ON_ERROR(result, kPrintSection, 1)

    result = printLabel(&section->value_label, section->value,
                        (uint8_t)getStringLength(section->value, section->value_size), kColourEnabled);
    EXIT_ON_ERROR(result, kPrintSection, 2)

    //do not print the separator after the last section
    if (index >= ((uint8_t)kNbSections - 1U)) {
        return kSuccessCode;
    }

    //initialise the separator
    const Area separator = {.x0 = label_margin_px,
                            .y0 = (uint8_t)(kTitleSectionHeightPx + ((index + 1U) * kSectionHeightPx) - (uint8_t)1U),
                            .x1 = kDisplayHeight - label_margin_px,
                            .y1 = (uint8_t)(kTitleSectionHeightPx + ((index + 1U) * kSectionHeightPx) - (uint8_t)1U)};

    result = printRectangle(display_buffer, kFrameBufferSize, &separator, kColourDecoration);
    EXIT_ON_ERROR(result, kPrintSection, 3)

    return kSuccessCode;
}

/**
 * Update the MCU internal temperature label
 *
 * @return printLabel return code
 */
static ErrorCode updateTemperature(void) {
    int32_t temperature = 0;
    (void)getInternalTemperatureCelsius(&temperature);

    int32_t length = leany_snprintf(internal_temperature, (kMaxTemperatureSize + 1U), "%3u*C", (uint8_t)temperature);
    return printLabel(&sections[kTemperature].value_label, internal_temperature, (uint8_t)length, kColourEnabled);
}
