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
#include <assert.h>
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
#include "bq25619_registers.inc"
#include "hal_adc.h"
#include "hal_i2c.h"
#include "hardware_events.h"
#include "systick.h"
#include "task_gpio.h"

enum {
    kStackSize = 250U,                  ///< Amount of words in the task stack
    kTaskLowPriority = 8U,              ///< FreeRTOS number for a low priority task
    kChipIDtimeout = 1000U,             ///< Maximum number of milliseconds to attempt reading the chip ID
    kNbChipIDtests = 5U,                ///< Number of times chip ID reading must be tested
    kUpdatePeriodMS = 200U,             ///< Period between two status updates in [ms]
    kBatteryFullPercent = 100U,         ///< Value used as a 100% battery level
    kMutexTimeoutMs = 10U,              ///< Maximum number of milliseconds before considering a mutex timeout
    kNbRetries = 5U,                    ///< Maximum number of retries upon I²C lack of ACK
    kNbAverageSamples = 8U,             ///< Maximum number of elements in the average buffer
    kBatteryLvlUpdatePeriodMs = 1000U,  ///< Period in [ms] between two battery level updates
    kMutexMS = 5U,                      ///< Max number of milliseconds to wait for a mutex
    kAdcMaxValue = 4095,                ///< Maximum ADC LSB value (12-bits -> [0 ... 4095])
};

static_assert(((uint8_t)kNbAverageSamples & ((uint8_t)kNbAverageSamples - 1U)) == 0U,
              "kNbAverageSamples must be a power of 2");

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
static void updateBatteryVoltage(void);
static uint16_t adcToBatteryVoltage_mV(uint16_t adc_raw, uint32_t adc_vref_mv);
static void averageBatteryVoltageMv(uint32_t new_voltage_mv);

static volatile TaskHandle_t task_handle = NULL;                 ///< handle of the FreeRTOS task
static volatile FunctionCode state = kStateStartup;              ///< Current state machine state
static StackType_t task_stack[kStackSize] = {0};                 ///< Buffer used as the task stack
static StaticTask_t task_state = {0};                            ///< Task state variables
static SemaphoreHandle_t battery_mutex = NULL;                   ///< Mutex used to protect the battery status
static ErrorCode result = {0};                                   ///< Buffer used to store the latest error code
static I2C_TypeDef* i2c_handle = I2C1;                           ///< I²C handle to use with all transmissons
static uint8_t battery_percentage = kBatteryFullPercent;         ///< Current battery percentage (for simulation)
static uint8_t battery_charging = 0U;                            ///< Current battery charge status (for simulation)
static TickType_t previous_tick = 0;                             ///< Tick at the last status update
static ChargerStatus current_battery_status;                     ///< Current battery status flags
static uint32_t last_battery_lvl_update_tick = 0;                ///< Last tick at which battery lvl was updated
static uint32_t battery_average_queue[kNbAverageSamples] = {0};  ///< Buffer used for averaging battery
static uint8_t battery_average_nbsamples = 0;                    ///< Number of samples saved in the buffer
static uint8_t battery_average_index = 0;                        ///< Index of the current buffer head
static uint32_t battery_average_total = 0;                       ///< Total value used to calculate battery average
static uint16_t battery_voltage_mv = 0;                          ///< Current battery voltage in [mV]

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

/**
 * Turn the system off
 *
 * @return disconnectBattery() code
 */
ErrorCode turnSystemOff(void) { return disconnectBattery(); }

/**
 * Get the latest measured battery voltage
 *
 * @param[out] voltage_mv Battery voltage in [mV]
 * @retval 1 Successfully retrieved
 * @retval 0 Could not retrieve voltage
 */
uint8_t getBatteryVoltageMv(uint16_t* voltage_mv) {
    uint8_t success = 0;
    if (xSemaphoreTake(battery_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        *voltage_mv = battery_voltage_mv;
        success = 1;
        xSemaphoreGive(battery_mutex);
    }

    return success;
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

    last_battery_lvl_update_tick = getCurrentTick();
    while (1) {
        switch (state) {
            case kStateStartup:
                result = stateStartup();
                state = kStateConfiguring;
                break;

            case kStateConfiguring:
                result = stateConfiguring();
                previous_tick = getCurrentTick();
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

        updateBatteryVoltage();
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

    vTaskDelay(pdMS_TO_TICKS(5));

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

    return kSuccessCode;
}

/**
 * Request a battery read on ADC
 */
static void updateBatteryVoltage(void) {
    static uint8_t updating = 0;

    //check if it is time to update the battery percentage
    if (systickTimeout(last_battery_lvl_update_tick, kBatteryLvlUpdatePeriodMs)) {
        last_battery_lvl_update_tick = getCurrentTick();
        updating = 1;
    }

    //if not supposed to be updating, exit
    if (!updating) {
        return;
    }

    //get the latest battery value
    uint16_t battery_adc_raw = 0;
    if (!getADCvalue(kADC1, kADCchannelBattery, &battery_adc_raw)) {
        return;
    }

    uint32_t adc_reference_mv = 0;
    if (!getADCreference_mV(&adc_reference_mv)) {
        return;
    }

    //transform the ADC value to [mV]
    const uint16_t new_battery_voltage_mv = adcToBatteryVoltage_mV(battery_adc_raw, adc_reference_mv);
    averageBatteryVoltageMv(new_battery_voltage_mv);
    updating = 0;
}

/**
 * Transform an ADC value to battery voltage in [mV]
 *
 * @param adc_raw Value to transform
 * @param adc_vref_mv ADC voltage reference in [mV]
 * @return Battery voltage in [mV]
 */
static uint16_t adcToBatteryVoltage_mV(uint16_t adc_raw, uint32_t adc_vref_mv) {
    static const uint32_t kVoltageDividerHighKohms = 56UL;
    static const uint32_t kVoltageDividerLowKohms = 56UL;

    if (!adc_vref_mv) {
        return 0;
    }

    const uint32_t conversion_numerator = (adc_vref_mv * (kVoltageDividerHighKohms + kVoltageDividerLowKohms));
    static const uint32_t kConversionDenominator = (kAdcMaxValue * kVoltageDividerLowKohms);

    return (uint16_t)((adc_raw * conversion_numerator) / kConversionDenominator);
}

/**
 * Blend a new battery voltage into the running average measurements
 *
 * @param new_voltage_mv New voltage to add in [mV]
 */
static void averageBatteryVoltageMv(uint32_t new_voltage_mv) {
    //add elements to the buffer until it's full
    if (battery_average_nbsamples < (kNbAverageSamples - 1)) {
        battery_average_queue[battery_average_index] = new_voltage_mv;
        battery_average_index++;
        battery_average_total += new_voltage_mv;
        battery_average_nbsamples++;
        if (xSemaphoreTake(battery_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
            battery_voltage_mv = (uint16_t)(battery_average_total / battery_average_nbsamples);
            xSemaphoreGive(battery_mutex);
        }
        return;
    }

    //buffer is full, use a cyclic average
    battery_average_index = (battery_average_index + 1U) % kNbAverageSamples;
    battery_average_total -= battery_average_queue[battery_average_index];
    battery_average_queue[battery_average_index] = new_voltage_mv;
    battery_average_total += new_voltage_mv;
    if (xSemaphoreTake(battery_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        battery_voltage_mv = (uint16_t)(battery_average_total / kNbAverageSamples);
        xSemaphoreGive(battery_mutex);
    }
}
