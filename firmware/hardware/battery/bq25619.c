/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 *
 * @file bq25619.c
 * @details Implements BQ25619 charger IC configuration and control functions
 * @author Gilles Henrard
 *
 * @note Datasheet : https://www.ti.com/lit/gpn/bq25619
 */
#include "bq25619.h"

#include <stdint.h>
#include <stm32f103xb.h>

#include "bq25619_registers.inc"
#include "errorstack.h"
#include "hal_i2c.h"
#include "systick.h"

enum {
    kI2Ctimeout_ms = 10U,    ///< Maximum number of milliseconds an I²C transfer can last
    kNbChipIDtests = 5U,     ///< Number of times chip ID reading must be tested
    kChipIDtimeout = 1000U,  ///< Maximum number of milliseconds to attempt reading the chip ID
};

/**
 * @brief Enumeration of all the function ID used in errors
 */
typedef enum {
    kTestID = 1,        ///< testBQ25619identifier() function
    kReset = 2,         ///< resetBQ25619() function
    kConfigure = 3,     ///< configureBQ25619() function
    kUpdateStatus = 4,  ///< updateBQ25619status() function
} FunctionCode;

static ErrorCode result = {0};          ///< Buffer used to store the latest error code
static I2C_TypeDef* i2c_handle = I2C1;  ///< I²C handle to use with all transmissons

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Check if the charger's device ID can be retrieved
 *
 * @retval 0 Success
 * @retval 1 Error while reading the register
 * @retval 2 No ID could be retrieved
 */
ErrorCode testBQ25619identifier(void) {
    //attempt reading the chip ID a few times to confirm communication works properly
    uint32_t start_tick = getCurrentTick();
    uint8_t valid_tests_remaining = kNbChipIDtests;
    do {
        uint8_t part_number = 0;
        result = readI2Cregisters(i2c_handle, kDEFAULT_SLAVEADDR, kPART_INFO, &part_number, 1);

        const uint8_t pn_valid = ((part_number & (uint8_t)kPN_MASK) == kPN_VALUE);
        if (pn_valid) {
            valid_tests_remaining--;
        }
    } while (valid_tests_remaining && !systickTimeout(start_tick, kChipIDtimeout));

    if (isError(result)) {
        return pushErrorCode(result, kTestID, 1);
    }

    if (valid_tests_remaining) {
        return createErrorCode(kTestID, 2, kErrorCritical);
    }

    return kSuccessCode;
}

/**
 * Send a hard reset request to the charger
 * 
 * @retval 0 Success
 * @retval 1 Error while sending the request
 */
ErrorCode resetBQ25619(void) {
    //reset the registers to their default value
    uint8_t reset = kREG_RESET;
    result = writeI2CRegisters(i2c_handle, kDEFAULT_SLAVEADDR, kPART_INFO, &reset, 1);
    EXIT_ON_ERROR(result, kReset, 1)

    return kSuccessCode;
}

/**
 * Configure the charger
 * 
 * @retval 0 Success
 * @retval 1 Error while writing a register
 */
ErrorCode configureBQ25619(void) {
    const uint8_t config_values[][2] = {
        // NOLINTBEGIN(misc-redundant-expression,hicpp-signed-bitwise)
        {kINPUT_CUR_LIMIT, kDISABLE_HIZ | kTEMPERATURE_IGNORE | kENABLE_BATSNS | kINPUT_LIMIT_1_2A},
        {kCHG_CONTROL0, kDISABLE_WATCHDOG | kDISABLE_PMID | kENABLE_CHARGE | kVSYS_MIN_3_5V},
        {kCHG_CUR_LIMIT, kREGULATE_LOW_CURRENT | kFASTCHARGE_1180MA},
        {kPCHG_TERM_CUR_LIMIT, kPRECHARGE_260MA | kTERMINATION_260MA},
        {kBATT_VOLT_LIMIT, kBATTERY_LIMIT_4_2V | kNO_TOPOFF_TIMER | kBATT_RECHG_THRESHOLD_120MV},
        {kCHG_CONTROL1, kENABLE_CHG_TERMINATION | kDISABLE_WATCHDOG_TIMER | kENABLE_CHG_SAFETY_TIMER | kSAFETY_10H},
        {kCHG_CONTROL2, kOVERVOLTAGE_THRESHOLD_6_4V | kINPUT_VOLTAGE_DPM_4_5V},
        // NOLINTEND(misc-redundant-expression,hicpp-signed-bitwise)
    };

    // write the configuration registers
    const uint8_t nb_registers = sizeof(config_values) / 2U;
    for (uint8_t value = 0; value < nb_registers; value++) {
        result =
            writeI2CRegisters(i2c_handle, kDEFAULT_SLAVEADDR, config_values[value][0], &config_values[value][1], 1U);
        EXIT_ON_ERROR(result, kConfigure, 1)
    }

    return kSuccessCode;
}

/**
 * Retireve the latest charger status
 *
 * @param interrupt_received Flags indicating the charger sent a GPIO interrupt
 * @retval 0 Success
 * @retval 1 Error while reading the status registers
 * @retval 2 Error while updating the registers
 */
ErrorCode updateBQ25619status(ChargerStatus* new_status, ChargerStatus* changes) {
    if (!new_status || !changes) {
        return createErrorCode(kUpdateStatus, 1, kErrorError);
    }

    //getting the actual new status requires a second reading
    ChargerStatus previous_status = *new_status;
    result = readI2Cregisters(i2c_handle, kDEFAULT_SLAVEADDR, kCHG_STATUS0, new_status->bytes, kNB_STATUS_BYTES);
    EXIT_ON_ERROR(result, kUpdateStatus, 2)

    //compute the changes between the old and new status
    changes->bytes[0] = previous_status.bytes[0] ^ new_status->bytes[0];
    changes->bytes[1] = previous_status.bytes[1] ^ new_status->bytes[1];
    changes->bytes[2] = previous_status.bytes[2] ^ new_status->bytes[2];

    return kSuccessCode;
}
