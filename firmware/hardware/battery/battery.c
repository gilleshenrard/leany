/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file battery.c
 * @author Gilles Henrard
 */
#include "battery.h"

#include <stdint.h>

#include "errorstack.h"

enum {
    kBatteryFullPercent = 100U,  ///< Value used as a 100% battery level
};

static uint8_t battery_percentage = kBatteryFullPercent;  ///< Current battery percentage (for simulation)
static uint8_t battery_charging = 0U;                     ///< Current battery charge status (for simulation)

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Get the battery status
 *
 * @param[out] status Battery current status
 * @return Success 
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
ErrorCode getBatteryStatus(BatteryStatus* status) {
    if (!status) {
        return createErrorCode(1, 1, kErrorError);
    }

    //
    //TODO: THIS MUST BE THREAD-SAFE
    //

    *status = (BatteryStatus){.level_percents = battery_percentage, .charging = battery_charging};
    return kSuccessCode;
}

/**
 * Set the battery percentage level
 * @param percentage New percentage in [%]
 * @retval 1 Percentage updated
 * @retval 0 Could not update percentage
 */
uint8_t setBatteryPercentage(uint8_t percentage) {
    if (percentage > kBatteryFullPercent) {
        return 0;
    }

    battery_percentage = percentage;
    return 1;
}

/**
 * Set the battery charge status
 * @param status 1 if charging, 0 otherwise
 */
void setBatteryChargeStatus(uint8_t status) { battery_charging = status; }
