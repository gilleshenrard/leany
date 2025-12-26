/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file icons.c
 * @author Gilles Henrard
 * @brief Implement icons bitmaps where each bit represent either a background or a foreground pixel
 */

#include "icons.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bitmap.h"
#include "errorstack.h"

enum {
    kPoolSize = 47U,       ///< Number of rows (uint16_t) comprising the bitmaps pool
    kLargePoolSize = 20U,  ///< Number of rows (uint32_t) comprising the large bitmaps pool
};

static const BitmapRow kIconsPool[kPoolSize];           ///< Bitmaps pool containing the small icons
static const uint32_t kLargeIconsPool[kLargePoolSize];  ///< Bitmaps pool containing the large icons
static const IconMetadata kDescriptors[kNbIcons] =      ///< Descriptors of all the icons
    {
        [kIconHold] = {.width_px = 8U, .height_px = 8U, .pool_offset = 0},
        [kIconAbsolute] = {.width_px = 8U, .height_px = 8U, .pool_offset = 8U},
        [kIconRelative] = {.width_px = 8U, .height_px = 8U, .pool_offset = 16U},
        [kIconPitch] = {.width_px = 3U, .height_px = 12U, .pool_offset = 24U},
        [kIconRoll] = {.width_px = 12U, .height_px = 3U, .pool_offset = 36U},
        [kIconBattery] = {.width_px = 15U, .height_px = 8U, .pool_offset = 39U},
        [kIconWarning] = {.width_px = 22U, .height_px = 20U, .pool_offset = 0U},
};

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Uncompress an icon to the pixel buffer
 *
 * @param icon Icon to uncompress
 * @param bitmap_metadata Metadata of the icon bitmap
 * @param foreground_colour Icon foreground colour
 * @retval 0 Success
 * @retval 1 No metadata provided
 * @retval 2 Icon out of range
 */
ErrorCode uncompressIcon(Icon icon, Bitmap* bitmap_metadata, ColourBigEndian foreground_colour) {
    if (!bitmap_metadata) {
        return createErrorCode(1, 1, kErrorError);
    }

    if (icon >= kNbIcons) {
        return createErrorCode(1, 2, kErrorError);
    }

    const IconMetadata* icon_metadata = (IconMetadata*)&kDescriptors[icon];
    const void* icon_bitmap = NULL;
    if (icon == kIconWarning) {
        icon_bitmap = (BitmapRow*)&kLargeIconsPool[icon_metadata->pool_offset];
    } else {
        icon_bitmap = (uint32_t*)&kIconsPool[icon_metadata->pool_offset];
    }

    bitmap_metadata->width_px = icon_metadata->width_px;
    bitmap_metadata->container_width_px = icon_metadata->width_px;
    bitmap_metadata->height_px = icon_metadata->height_px;
    return uncompressBitmap(bitmap_metadata, icon_bitmap, foreground_colour);
}

/**
 * Draw the battery level bars on top of the battery icon
 *
 * @param buffer Buffer in which draw the bars
 * @param battery_percent Battery level percentage
 * @param charging Flag indicating whether the battery is charging or not
 */
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void drawBatteryLevelBars(Pixel buffer[], uint8_t battery_percent, bool charging) {
    const uint8_t x_margin = 2U;
    const uint8_t y_margin = 2U;
    const uint8_t bar_width = 3U;

    ColourBigEndian bars_colour = kColourOK;

    //set the number of bars to display
    uint8_t nb_bars = 3U;
    if (battery_percent <= 50U) {  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
        bars_colour = kColourWarning;
        nb_bars = 2U;
    }
    if (battery_percent <= 25U) {  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
        bars_colour = kColourCritical;
        nb_bars = 1U;
    }

    //change the foreground colour if the battery is charging
    if (charging) {
        bars_colour = kColourAccent;
    }

    //add bars to the icon buffer
    const uint8_t battery_width_px = kDescriptors[kIconBattery].width_px;
    for (uint8_t level = 0; level < nb_bars; level++) {
        for (uint8_t row = 0; row < 4U; row++) {
            uint16_t buffer_offset = (uint16_t)((row + y_margin) * battery_width_px) + x_margin;
            buffer_offset += (uint16_t)(level * bar_width);

            Pixel* bar_row = &buffer[buffer_offset];
            bar_row[0] = (Pixel)bars_colour;
            bar_row[1] = (Pixel)bars_colour;
            bar_row[2] = kColourBackground;
        }
    }
}

/**
 * Get the width of an icon in [px]
 * @param icon Icon of which get the width
 * @return Width in [px]
 */
uint8_t getIconWidth(Icon icon) {
    if (icon >= kNbIcons) {
        return 0;
    }

    return kDescriptors[icon].width_px;
}

/**
 * Get the height of an icon in [px]
 * @param icon Icon of which get the height
 * @return Height in [px]
 */
uint8_t getIconHeight(Icon icon) {
    if (icon >= kNbIcons) {
        return 0;
    }

    return kDescriptors[icon].height_px;
}

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

static const BitmapRow kIconsPool[kPoolSize] = {
    // clang-format off

    // pause icon (8 pixels wide)
    0xE700U,
    0xE700U,
    0xE700U,
    0xE700U,
    0xE700U,
    0xE700U,
    0xE700U,
    0xE700U,

    // absolute measurements icon (8 pixels wide)
    0x1000U,
    0x1000U,
    0x1000U,
    0x1000U,
    0x9200U,
    0x5400U,
    0x3800U,
    0x1000U,

    // relative measurements icon (8 pixels wide)
    0xFF00U,
    0x9000U,
    0x9000U,
    0xF000U,
    0x8000U,
    0x8000U,
    0x8000U,
    0x8000U,

    // pitch icon (3 pixels wide)
    0x4000U,
    0xE000U,
    0x4000U,
    0x4000U,
    0x4000U,
    0x0000U,
    0x0000U,
    0x4000U,
    0x4000U,
    0x4000U,
    0xE000U,
    0x4000U,

    // roll icon (12 pixels wide)
    0x4020U,
    0xF9F0U,
    0x4020U,

    // battery icon (15 pixels wide)
    0xFFF0U,
    0x8010U,
    0x8016U,
    0x8016U,
    0x8016U,
    0x8016U,
    0x8010U,
    0xFFF0U,

    // clang-format on
};

static const uint32_t kLargeIconsPool[kLargePoolSize] = {
    // clang-format off
    0x00300000U, //           ##          
    0x00780000U, //          ####         
    0x00780000U, //          ####         
    0x00FC0000U, //         ######        
    0x00CC0000U, //         ##  ##        
    0x01CE0000U, //        ###  ###       
    0x01860000U, //        ##    ##       
    0x03870000U, //       ###    ###
    0x03330000U, //       ##  ##  ##      
    0x07338000U, //      ###  ##  ###     
    0x06318000U, //      ##   ##   ##     
    0x0E31C000U, //     ###   ##   ###    
    0x0C30C000U, //     ##    ##    ##    
    0x1C00E000U, //    ###          ###   
    0x18006000U, //    ##            ##   
    0x38307000U, //   ###     ##     ###  
    0x30303000U, //   ##      ##      ##  
    0x70006800U, //  ###              ### 
    0x7FFFF800U, //  #################### 
    0x7FFFF800U, //  ####################

    // clang-format on
};
