/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
/**
 * @file battery_management.c
 * @brief Implement the battery voltage and BQ25619 charger status monitoring
 * @author Gilles Henrard
 * @date 21/09/2025
 *
 * @details
 * BQ25619 Datasheet : https://www.ti.com/lit/ds/symlink/bq25618.pdf?ts=1729197098952&ref_url=https%253A%252F%252Fwww.ti.com%252Fsitesearch%252Fen-us%252Fdocs%252Funiversalsearch.tsp%253FlangPref%253Den-US%2526nr%253D186%2526searchTerm%253Dbq25618
 */
#include "battery_management.h"

#include <FreeRTOS.h>
#include <errorstack.h>
#include <main.h>
#include <portmacro.h>
#include <projdefs.h>
#include <stdint.h>
#include <stm32f103xb.h>
#include <stm32f1xx_ll_i2c.h>
#include <task.h>

#include "bq25619_registers.inc"

enum {
    kStackSize = 250U,        ///< Amount of words in the task stack
    kTaskLowPriority = 8U,    ///< FreeRTOS number for a low priority task
    kI2Ctimeout_ms = 10U,     ///< Maximum number of milliseconds an I²C transfer can last
    kChipIDtimeout = 1000U,   ///< Maximum number of milliseconds to attempt reading the chip ID
    kNbChipIDtests = 5U,      ///< Number of times chip ID reading must be tested
    kUpdatePeriodMS = 1000U,  ///< Period between two status updates in [ms]
};

/**
 * @brief Enumeration of all the function ID used in errors
 */
typedef enum {
    kTaskLoop = 1,           ///< taskBatteryManagement() function
    kReadRegisters = 2,      ///< readRegisters() function
    kStateStartup = 3,       ///< stateStartup() state function
    kStateIdle = 4,          ///< stateIdle() state function
    kRequestRead = 5,        ///< requestRead() function
    kInitiateReception = 6,  ///< initiateReception() function
    kWriteRegisters = 7,     ///< writeRegisters() function
    kStateConfiguring = 8,   ///< stateConfiguring() state function
} FunctionCode;

//state machine functions
static void taskBatteryManagement(void* argument);
static ErrorCode stateStartup(void);
static ErrorCode stateConfiguring(void);
static ErrorCode stateIdle(void);

//utilitary functions
static ErrorCode initiateTransaction(I2C_TypeDef* descriptor, BQ25619register first_register, TickType_t start_tick);
static ErrorCode readRegisters(I2C_TypeDef* descriptor, BQ25619register first_register, uint8_t data[],
                               uint8_t nb_bytes);
static ErrorCode writeRegisters(I2C_TypeDef* descriptor, BQ25619register first_register, const uint8_t data[],
                                uint8_t nb_bytes);

static volatile TaskHandle_t task_handle = NULL;     ///< handle of the FreeRTOS task
static volatile FunctionCode state = kStateStartup;  ///< Current state machine state
static volatile uint8_t task_notifiable = 0;      ///< Flag indicating whether the task can safely receive notifications
static StackType_t task_stack[kStackSize] = {0};  ///< Buffer used as the task stack
static StaticTask_t task_state = {0};             ///< Task state variables
static ErrorCode result = {0};                    ///< Buffer used to store the latest error code
static ChargerStatus latest_status = {0};         ///< Latest value of the charger status bytes
static ChargerStatus changes_mask = {0};          ///< Changes in the status bytes
static I2C_TypeDef* i2c_handle = I2C1;            ///< I²C handle to use with all transmissons

/****************************************************************************************************************/
/****************************************************************************************************************/

/**
 * Handle a GPIO interrupt received from the charger
 */
void chargerInterruptTriggered(void) {
    if (!task_notifiable) {
        return;
    }

    BaseType_t has_woken = 0;
    vTaskNotifyGiveFromISR(task_handle, &has_woken);
    portYIELD_FROM_ISR(has_woken);
}

/**
 * Create the FreeRTOS task which takes care of the battery and charger management
 *
 * @return Success
 */
ErrorCode createBatteryTask(void) {
    //create the static task
    task_handle = xTaskCreateStatic(taskBatteryManagement, "Battery management task", kStackSize, NULL,
                                    kTaskLowPriority, task_stack, &task_state);
    if (!task_handle) {
        Error_Handler();
    }

    return kSuccessCode;
}

/****************************************************************************************************************/
/****************************************************************************************************************/

/**
 * Run the battery management FreeRTOS task
 *
 * @param argument Unused
 */
static void taskBatteryManagement(void* argument) {
    (void)argument;

    LL_I2C_Enable(i2c_handle);

    while (1) {
        switch (state) {
            case kStateStartup:
                result = stateStartup();
                state = kStateConfiguring;
                break;

            case kStateConfiguring:
                result = stateConfiguring();
                state = kStateIdle;
                break;

            case kStateIdle:
                result = stateIdle();
                break;

            case kTaskLoop:
            case kReadRegisters:
            case kRequestRead:
            case kInitiateReception:
            case kWriteRegisters:
            default:
                result = createErrorCode(kTaskLoop, 1, kErrorCritical);
                break;
        }

        if (isError(result)) {
            Error_Handler();
        }
    }
}

/**
 * State during which the charger startup procedure is ran
 *
 * @retval 0 Success
 * @retval 1 Error while reading the chip ID
 * @retval 2 Invalid chip ID
 * @retval 3 Error while resetting the registers to default
 */
static ErrorCode stateStartup(void) {
    //attempt reading the chip ID a few times to confirm communication works properly
    uint8_t part_number = 0;
    TickType_t start_tick = xTaskGetTickCount();
    uint8_t tests_remaining = kNbChipIDtests;
    do {
        result = readRegisters(i2c_handle, kPART_INFO, &part_number, 1);
        EXIT_ON_ERROR(result, kStateStartup, 1)

        if ((part_number & (uint8_t)kPN_MASK) == kPN_VALUE) {
            tests_remaining--;
        }
    } while (tests_remaining && !timeout(start_tick, kChipIDtimeout));

    if (tests_remaining) {
        return createErrorCode(kStateStartup, 2, kErrorCritical);
    }

    //reset the registers to their default value
    uint8_t reset = kREG_RESET;
    result = writeRegisters(i2c_handle, kPART_INFO, &reset, 1);
    EXIT_ON_ERROR(result, kStateStartup, 3)

    return kSuccessCode;
}

/**
 * State during which the charger's registers are configured
 *
 * @retval 0 Success
 * @retval 1 Error while writing a configuration register
 */
static ErrorCode stateConfiguring(void) {
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
        result = writeRegisters(i2c_handle, config_values[value][0], &config_values[value][1], 1U);
        EXIT_ON_ERROR(result, kStateConfiguring, 1)
    }

    task_notifiable = 1;
    return kSuccessCode;
}

/**
 * State during which the state machine is idle
 *
 * @retval 1 Error while reading the status
 * @retval 2 Error while re-reading the status
 */
static ErrorCode stateIdle(void) {
    //wait for a while unless a GPIO interrupt was caught
    const uint32_t interrupt_received = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kUpdatePeriodMS));

    //read the current charger status
    result = readRegisters(i2c_handle, kCHG_STATUS0, latest_status.bytes, kNB_STATUS_BYTES);
    EXIT_ON_ERROR(result, kStateIdle, 1)

    //if no interrupt was caught, stop there
    if (interrupt_received == pdFALSE) {
        return kSuccessCode;
    }

    //getting the actual new status requires a second reading
    ChargerStatus previous_status = latest_status;
    result = readRegisters(i2c_handle, kCHG_STATUS0, latest_status.bytes, kNB_STATUS_BYTES);
    EXIT_ON_ERROR(result, kStateIdle, 2)

    //compute the changes between the old and new status
    changes_mask.bytes[0] = previous_status.bytes[0] ^ latest_status.bytes[0];
    changes_mask.bytes[1] = previous_status.bytes[1] ^ latest_status.bytes[1];
    changes_mask.bytes[2] = previous_status.bytes[2] ^ latest_status.bytes[2];

    return kSuccessCode;
}

/****************************************************************************************************************/
/****************************************************************************************************************/

/**
 * Initiate an I²C reception sequence
 *
 * @param descriptor I²C descriptor to use
 * @param first_register Number of the first register to read
 * @param start_tick FreeRTOS tick at the start of the sequence
 * @retval 0 Success
 * @retval 1 Timeout while waiting for start signal to be done
 * @retval 2 Timeout while waiting for ADDR byte to be sent
 * @retval 3 Timeout while waiting for the register address to be sent
 */
static ErrorCode initiateTransaction(I2C_TypeDef* descriptor, BQ25619register first_register, TickType_t start_tick) {
    uint8_t timeout_value = 0;

    //send a start signal
    LL_I2C_GenerateStartCondition(descriptor);
    EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_SB(descriptor), kInitiateReception, 1)

    //send the slave address
    LL_I2C_TransmitData8(descriptor, (uint8_t)((uint8_t)kDEFAULT_SLAVEADDR << 1U) | (uint8_t)kCHG_WRITE);
    EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_ADDR(descriptor), kInitiateReception, 2)
    LL_I2C_AcknowledgeNextData(descriptor, LL_I2C_NACK);
    LL_I2C_ClearFlag_ADDR(descriptor);

    //send the register address
    EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_TXE(descriptor), kInitiateReception, 3)
    LL_I2C_TransmitData8(descriptor, (uint8_t)first_register);

    return kSuccessCode;
}

/**
 * Burst read data via I²C
 *
 * @param descriptor SPI descriptor to use
 * @param first_register First register to read
 * @param[out] data Data buffer to fill
 * @param nb_bytes Number of bytes to read
 * @retval 0 Success
 * @retval 1 Invalid descriptor or data buffer provided
 * @retval 2 Error while initiating the read sequence
 * @retval 3 Timeout while waiting for the data byte
 * @retval 4 Timeout while waiting for slave address to be sent
 * @retval 5 Timeout while waiting for a data byte to be received
 * @retval 6 Timeout while waiting for the last data byte to be received
 */
static ErrorCode readRegisters(I2C_TypeDef* descriptor, BQ25619register first_register, uint8_t data[],
                               uint8_t nb_bytes) {
    (void)nb_bytes;

    if (!descriptor || (!data && nb_bytes)) {
        return createErrorCode(kReadRegisters, 1, kErrorError);
    }

    if (!nb_bytes) {
        return kSuccessCode;
    }

    uint8_t timeout_value = 0;
    TickType_t start_tick = xTaskGetTickCount();

    //initiate the I²C reception
    result = initiateTransaction(descriptor, first_register, start_tick);
    EXIT_ON_ERROR(result, kReadRegisters, 2)

    //resend the slave address in read mode
    LL_I2C_GenerateStartCondition(descriptor);
    EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_SB(descriptor), kInitiateReception, 3)
    LL_I2C_TransmitData8(descriptor, (uint8_t)((uint8_t)kDEFAULT_SLAVEADDR << 1U) | (uint8_t)kCHG_READ);
    EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_ADDR(descriptor), kInitiateReception, 4)
    LL_I2C_ClearFlag_ADDR(descriptor);

    //read all bytes except for the last one
    uint8_t current_byte = 0;
    LL_I2C_AcknowledgeNextData(descriptor, LL_I2C_ACK);
    while (nb_bytes - 1U) {
        EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_RXNE(descriptor), kReadRegisters, 5)
        data[current_byte] = LL_I2C_ReceiveData8(descriptor);
        current_byte++;
        nb_bytes--;
    }

    //read the last byte, with a NACK and a STOP signal
    LL_I2C_AcknowledgeNextData(descriptor, LL_I2C_NACK);
    LL_I2C_GenerateStopCondition(descriptor);
    EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_RXNE(descriptor), kReadRegisters, 6)
    data[current_byte] = LL_I2C_ReceiveData8(descriptor);
    return kSuccessCode;
}

/**
 * Burst-write data to registers
 *
 * @param descriptor I²C descriptor to use
 * @param first_register Number of the first register to which write data
 * @param data Data to write
 * @param nb_bytes Number of bytes of data to write
 * @retval 0 Success
 * @retval 1 Invalid descriptor or data buffer provided
 * @retval 2 Error while initiating the write sequence
 * @retval 3 Timeout while waiting for TXE to be set
 * @retval 4 Timeout while waiting for TXE to be set before the last data byte
 * @retval 5 Timeout while waiting for Byte Transfer Finish to be set
 */
static ErrorCode writeRegisters(I2C_TypeDef* descriptor, BQ25619register first_register, const uint8_t data[],
                                uint8_t nb_bytes) {
    if (!descriptor || (!data && nb_bytes)) {
        return createErrorCode(kReadRegisters, 1, kErrorError);
    }

    if (!nb_bytes) {
        return kSuccessCode;
    }

    uint8_t timeout_value = 0;
    TickType_t start_tick = xTaskGetTickCount();

    //initiate the I²C transmission
    result = initiateTransaction(descriptor, first_register, start_tick);
    EXIT_ON_ERROR(result, kReadRegisters, 2)

    //send the data bytes all until the one before last
    uint8_t current_byte = 0;
    while (nb_bytes - 1U) {
        EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_TXE(descriptor), kWriteRegisters, 3)
        LL_I2C_TransmitData8(descriptor, data[current_byte]);
        current_byte++;
        nb_bytes--;
    }

    //send the last byte and stop the transmission
    EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_TXE(descriptor), kWriteRegisters, 4)
    LL_I2C_TransmitData8(descriptor, data[current_byte]);
    EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_BTF(descriptor), kWriteRegisters, 5)
    LL_I2C_GenerateStopCondition(descriptor);

    return kSuccessCode;
}
