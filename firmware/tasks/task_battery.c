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
#include <FreeRTOSConfig.h>
#include <errorstack.h>
#include <main.h>
#include <portmacro.h>
#include <projdefs.h>
#include <semphr.h>
#include <stdint.h>
#include <stm32f103xb.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_i2c.h>
#include <task.h>

#include "bq25619.h"
#include "bq25619_registers.inc"
#include "hal_adc.h"
#include "hal_i2c.h"
#include "hardware_events.h"
#include "systick.h"

enum {
    kStackSize = 250U,                  ///< Amount of words in the task stack
    kTaskLowPriority = 8U,              ///< FreeRTOS number for a low priority task
    kChipIDtimeout = 1000U,             ///< Maximum number of milliseconds to attempt reading the chip ID
    kNbChipIDtests = 5U,                ///< Number of times chip ID reading must be tested
    kUpdatePeriodMS = 200U,             ///< Period between two status updates in [ms]
    kBatteryFullPercent = 100U,         ///< Value used as a 100% battery level
    kMutexTimeoutMs = 10U,              ///< Maximum number of milliseconds before considering a mutex timeout
    kNbRetries = 5U,                    ///< Maximum number of retries upon I²C lack of ACK
    kBatteryLvlUpdatePeriodMs = 1000U,  ///< Period in [ms] between two battery level updates
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
static ErrorCode updateBatteryLevel(void);
static uint8_t adcToVoltageTenths(uint16_t adc_raw);

static volatile TaskHandle_t task_handle = NULL;          ///< handle of the FreeRTOS task
static volatile FunctionCode state = kStateStartup;       ///< Current state machine state
static StackType_t task_stack[kStackSize] = {0};          ///< Buffer used as the task stack
static StaticTask_t task_state = {0};                     ///< Task state variables
static SemaphoreHandle_t battery_mutex = NULL;            ///< Mutex used to protect the battery status
static ErrorCode result = {0};                            ///< Buffer used to store the latest error code
static I2C_TypeDef* i2c_handle = I2C1;                    ///< I²C handle to use with all transmissons
static uint8_t battery_percentage = kBatteryFullPercent;  ///< Current battery percentage (for simulation)
static uint8_t battery_charging = 0U;                     ///< Current battery charge status (for simulation)
static TickType_t previous_tick = 0;                      ///< Tick at the last status update
static ChargerStatus current_battery_status;              ///< Current battery status flags
static uint32_t last_battery_lvl_update_tick = 0;         ///< Last tick at which battery lvl was updated
static uint8_t battery_voltage_tenths = 0;                ///< Current battery voltage in [0.1V]

/****************************************************************************************************************/
/****************************************************************************************************************/

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
        *status = (BatteryStatus){.level_percents = battery_percentage,
                                  .charging = (battery_charging | isBQ25619charging(&current_battery_status))};
        xSemaphoreGive(battery_mutex);
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
        xSemaphoreGive(battery_mutex);
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
        xSemaphoreGive(battery_mutex);
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
                previous_tick = getCurrentTick();
                last_battery_lvl_update_tick = getCurrentTick();
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

    for (uint8_t attempt = 0; attempt < (uint8_t)kNbRetries; attempt++) {
        result = resetBQ25619();
        if (getDeepestError(result) != kErrorAcknowledgeFailure) {
            break;
        }
    }
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
    for (uint8_t attempt = 0; attempt < (uint8_t)kNbRetries; attempt++) {
        result = configureBQ25619();
        if (getDeepestError(result) != kErrorAcknowledgeFailure) {
            break;
        }
    }
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
    vTaskDelayUntil(&previous_tick, pdMS_TO_TICKS(kUpdatePeriodMS));

    ChargerStatus changes;

    //retrieve the latest charger status
    for (uint8_t attempt = 0; attempt < (uint8_t)kNbRetries; attempt++) {
        result = updateBQ25619status(&current_battery_status, &changes);
        if (getDeepestError(result) != kErrorAcknowledgeFailure) {
            break;
        }
    }
    EXIT_ON_ERROR(result, kStateIdle, 1);

    //if status changed, trigger event
    if (BQ25619statusChanged(&changes)) {
        if (BQ25619Error(&current_battery_status)) {
            return createErrorCode(kStateIdle, 2, kErrorError);
        }
        triggerHardwareEvent(kEventBatteryStatus);
    }

    return updateBatteryLevel();
}

/**
 * Measure the battery voltage and estimate its level
 *
 * @return ErrorCode 
 */
static ErrorCode updateBatteryLevel(void) {
    //check if it is time to update the battery percentage
    if (!systickTimeout(last_battery_lvl_update_tick, kBatteryLvlUpdatePeriodMs)) {
        return kSuccessCode;
    }
    last_battery_lvl_update_tick = getCurrentTick();

    //open the battery measurement path
    LL_GPIO_SetOutputPin(BATT_EN_GPIO_Port, BATT_EN_Pin);
    vTaskDelay(pdMS_TO_TICKS(1U));

    //request ADC measurements
    if (requestADCmeasurement(kADCchannelBattery)) {
        LL_GPIO_ResetOutputPin(BATT_EN_GPIO_Port, BATT_EN_Pin);
        return kSuccessCode;
    }

    //get the latest battery value
    ADCresult adc_result;
    if (!getADCvalue(kADCchannelBattery, &adc_result)) {
        LL_GPIO_ResetOutputPin(BATT_EN_GPIO_Port, BATT_EN_Pin);
        return kSuccessCode;
    }

    //close the battery measurement path (saves energy)
    LL_GPIO_ResetOutputPin(BATT_EN_GPIO_Port, BATT_EN_Pin);

    //transform the ADC value to [0.1V]
    battery_voltage_tenths = adcToVoltageTenths(adc_result.value);

    return kSuccessCode;
}

/**
 * Transform an ADC value to battery voltage in [0.1V]
 *
 * @param adc_raw Value to transform
 * @return Battery voltage
 */
static uint8_t adcToVoltageTenths(uint16_t adc_raw) {
    /**
     * Battery voltage goes through a voltage divider to a 12-bit 3.3V ADC
     * Voltage divider : 50k high / 50k low
     * ADC : 12-bits -> 4096 steps
     *
     * -> voltage in [0.1V] = adc_raw * 10 * (3.3) * (50 + 50)
     *                                  ------------------------
     *                                          4095 * 50
     */
    static const uint32_t kConversionNumerator = 3300U;
    static const uint32_t kConversionDenominator = 204750U;

    return (uint8_t)((adc_raw * kConversionNumerator) / kConversionDenominator);
}
