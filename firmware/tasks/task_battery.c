/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 *
 * @file task_battery.c
 * @brief Implement the battery voltage and charger status monitoring
 * @author Gilles Henrard
 */
#include "task_battery.h"

#include <FreeRTOS.h>
#include <errorstack.h>
#include <main.h>
#include <portmacro.h>
#include <projdefs.h>
#include <semphr.h>
#include <stdint.h>
#include <stm32f103xb.h>
#include <stm32f1xx_ll_i2c.h>
#include <task.h>

#include "bq25619.h"

enum {
    kStackSize = 250U,           ///< Amount of words in the task stack
    kTaskLowPriority = 8U,       ///< FreeRTOS number for a low priority task
    kChipIDtimeout = 1000U,      ///< Maximum number of milliseconds to attempt reading the chip ID
    kNbChipIDtests = 5U,         ///< Number of times chip ID reading must be tested
    kUpdatePeriodMS = 1000U,     ///< Period between two status updates in [ms]
    kBatteryFullPercent = 100U,  ///< Value used as a 100% battery level
    kMutexTimeoutMs = 10U,       ///< Maximum number of milliseconds before considering a mutex timeout
};

/**
 * @brief Enumeration of all the function ID used in errors
 */
typedef enum {
    kTaskLoop = 1,          ///< taskBatteryManagement() function
    kStateStartup = 3,      ///< stateStartup() state function
    kStateIdle = 4,         ///< stateIdle() state function
    kRequestRead = 5,       ///< requestRead() function
    kStateConfiguring = 8,  ///< stateConfiguring() state function
} FunctionCode;

//state machine functions
static void taskBatteryManagement(void* argument);
static ErrorCode stateStartup(void);
static ErrorCode stateConfiguring(void);
static ErrorCode stateIdle(void);

static volatile TaskHandle_t task_handle = NULL;          ///< handle of the FreeRTOS task
static volatile FunctionCode state = kStateStartup;       ///< Current state machine state
static StackType_t task_stack[kStackSize] = {0};          ///< Buffer used as the task stack
static StaticTask_t task_state = {0};                     ///< Task state variables
static SemaphoreHandle_t battery_mutex = NULL;            ///< Mutex used to protect the battery status
static ErrorCode result = {0};                            ///< Buffer used to store the latest error code
static I2C_TypeDef* i2c_handle = I2C1;                    ///< I²C handle to use with all transmissons
static uint8_t battery_percentage = kBatteryFullPercent;  ///< Current battery percentage (for simulation)
static uint8_t battery_charging = 0U;                     ///< Current battery charge status (for simulation)

/****************************************************************************************************************/
/****************************************************************************************************************/

/**
 * Handle a GPIO interrupt received from the charger
 */
void chargerInterruptTriggered(void) {
    if (state != kStateIdle) {
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
    static StaticSemaphore_t battery_mutex_state = {0};  ///< battery mutex state variables

    //create a semaphore to protect battery status
    battery_mutex = xSemaphoreCreateMutexStatic(&battery_mutex_state);
    configASSERT(battery_mutex);

    //create the static task
    task_handle = xTaskCreateStatic(taskBatteryManagement, "Battery management task", kStackSize, NULL,
                                    kTaskLowPriority, task_stack, &task_state);
    if (!task_handle) {
        Error_Handler();
    }

    return kSuccessCode;
}

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

    if (xSemaphoreTake(battery_mutex, pdMS_TO_TICKS(kMutexTimeoutMs)) == pdTRUE) {
        *status = (BatteryStatus){.level_percents = battery_percentage, .charging = battery_charging};
    }

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
        percentage = kBatteryFullPercent;
    }

    if (xSemaphoreTake(battery_mutex, pdMS_TO_TICKS(kMutexTimeoutMs)) == pdTRUE) {
        battery_percentage = percentage;
    }
    return 1;
}

/**
 * Set the battery charge status
 * @param status 1 if charging, 0 otherwise
 */
void setBatteryChargeStatus(uint8_t status) {
    if (xSemaphoreTake(battery_mutex, pdMS_TO_TICKS(kMutexTimeoutMs)) == pdTRUE) {
        battery_charging = status;
    }
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
            case kRequestRead:
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
    result = testBQ25619identifier();
    EXIT_ON_ERROR(result, kStateStartup, 1)

    result = resetBQ25619();
    EXIT_ON_ERROR(result, kStateStartup, 2)

    return kSuccessCode;
}

/**
 * State during which the charger's registers are configured
 *
 * @retval 0 Success
 * @retval 1 Error while writing a configuration register
 */
static ErrorCode stateConfiguring(void) {
    result = configureBQ25619();
    EXIT_ON_ERROR(result, kStateConfiguring, 1)

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

    result = updateBQ25619status(interrupt_received);
    EXIT_ON_ERROR(result, kStateIdle, 1);

    return kSuccessCode;
}
