/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file task_gpio.c
 * @brief Implement the FreeRTOS task taking care of the GPIOs
 * @author Gilles Henrard
 */
#include "task_gpio.h"

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <main.h>
#include <portmacro.h>
#include <projdefs.h>
#include <semphr.h>
#include <stddef.h>
#include <stdint.h>
#include <stm32f1xx_hal_def.h>
#include <task.h>

#include "buttons.h"
#include "errorstack.h"
#include "hal_adc.h"
#include "hardware_events.h"
#include "led.h"
#include "systick.h"

enum {
    kStackSize = 150U,              ///< Amount of words in the task stack
    kTaskLowPriority = 8U,          ///< FreeRTOS number for a low priority task
    kTemperatureRefreshMs = 2000U,  ///< Timespan between two readings in [ms]
    kMutexMS = 5U,                  ///< Max number of milliseconds to wait for a mutex

    // STM32F103 temperature sensor calibration parameters (from datasheet)
    kTempSensorAvgSlope_uV_C = 4300,  ///< Avg slope: 4.3 mV/°C (scaled to uV/°C)
    kTempSensorV25_mV = 1430,         ///< Voltage at 25°C: 1.43V (in mV)
    kTempSensorCalibTemp_C = 25,      ///< Calibration temperature: 25°C
};

//task functions
static void taskGPIO(void* argument);
static void updateInternalTemperature(void);
static int32_t adcToInternalTemperature(uint16_t adc_raw);

//state variables
static volatile TaskHandle_t task_handle = NULL;    ///< handle of the FreeRTOS task
static SemaphoreHandle_t temperature_mutex = NULL;  ///< Mutex which protects the temperature readings
static ErrorCode result;                            ///< Current functions errorstack result
static uint32_t current_tick = 0;                   ///< Current OS tick
static int32_t latest_temperature_celsius = 0;      ///< Latest MCU internal temperature in [°C]

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Create FreeRTOS static task for the buttons
 */
void createGPIOtask(void) {
    static StackType_t task_stack[kStackSize] = {0};         ///< Buffer used as the task stack
    static StaticTask_t task_state = {0};                    ///< Task state variables
    static StaticSemaphore_t temperature_mutex_state = {0};  ///< ADC value mutex state variables

    //create a semaphore to protect measurements
    temperature_mutex = xSemaphoreCreateMutexStatic(&temperature_mutex_state);
    if (!temperature_mutex) {
        Error_Handler();
    }

    //create the static task
    task_handle = xTaskCreateStatic(taskGPIO, "GPIO task", kStackSize, NULL, kTaskLowPriority, task_stack, &task_state);
    configASSERT(task_handle)
}

/**
 * Get the latest MCU internal temperature in [°C]
 * @param[out] temperature_celsius Pointer to store temperature value
 * @retval 1 Temperature retrieved successfully
 * @retval 0 Mutex timeout occurred
 */
uint8_t getInternalTemperatureCelsius(int32_t* temperature_celsius) {
    if (temperature_celsius == NULL) {
        return 0;
    }

    if (xSemaphoreTake(temperature_mutex, pdMS_TO_TICKS(kMutexMS)) == pdFALSE) {
        return 0;
    }

    *temperature_celsius = latest_temperature_celsius;
    (void)xSemaphoreGive(temperature_mutex);

    return 1;
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Run the GPIO state machine
 *
 * @param argument Unused
 */
static void taskGPIO(void* argument) {
    UNUSED(argument);

    initialiseLED();
    initialiseHALadc();
    current_tick = getCurrentTick();

    while (1) {
        result = runButtonsStateMachine();
        if (isError(result)) {
            Error_Handler();
        }

        runLEDstateMachine();
        runADCstateMachine();
        updateInternalTemperature();

        vTaskDelay(pdMS_TO_TICKS(5U));
    }
}

/**
 * Update the MCU internal temperature
 */
static void updateInternalTemperature(void) {
    // State 1: Check if it's time to start a new conversion
    if (systickTimeout(current_tick, kTemperatureRefreshMs)) {
        current_tick = getCurrentTick();
        (void)requestADCmeasurement(kADCchannelTemperature);
    }

    // State 2: Check if any conversion is ready to read (independent of timeout)
    ADCresult adc_result;
    const uint8_t updated = getADCvalue(kADCchannelTemperature, &adc_result);
    if (!updated) {
        return;
    }

    uint8_t value_changed = 0;
    if (xSemaphoreTake(temperature_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        int32_t new_temperature = adcToInternalTemperature(adc_result.value);

        if (latest_temperature_celsius != new_temperature) {
            latest_temperature_celsius = new_temperature;
            value_changed = 1;
        }
        (void)xSemaphoreGive(temperature_mutex);
    }

    if (value_changed) {
        triggerHardwareEvent(kEventTemperature);
    }
}

/**
 * Compute the MCU internal temperature from a raw ADC value
 *
 * @param adc_raw ADC raw value
 * @return MCU internal temperature
 */
static int32_t adcToInternalTemperature(const uint16_t adc_raw) {
    return __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(kTempSensorAvgSlope_uV_C, kTempSensorV25_mV, kTempSensorCalibTemp_C,
                                                kAdcVref_mV, adc_raw, LL_ADC_RESOLUTION_12B);
}
