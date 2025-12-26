/*
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file hal_adc.c
 * @brief Implement the MCU internal temperature ADC reading
 * @author Gilles Henrard
 */
#include "hal_adc.h"

#include <FreeRTOS.h>
#include <main.h>
#include <portmacro.h>
#include <projdefs.h>
#include <semphr.h>
#include <stddef.h>
#include <stdint.h>
#include <stm32f103xb.h>
#include <stm32f1xx_hal.h>
#include <stm32f1xx_ll_adc.h>

#include "hardware_events.h"

enum {
    kMutexMS = 5U,                  ///< Max number of milliseconds to wait for a mutex
    kInitTimeoutMs = 100U,          ///< Maximum number of milliseconds to wait for first reading
    kTemperatureRefreshMs = 2000U,  ///< Timespan between two readings in [ms]

    // STM32F103 temperature sensor calibration parameters (from datasheet)
    kTempSensorAvgSlope_mV_C = 4300,  ///< Avg slope: 4.3 mV/°C (scaled by 1000)
    kTempSensorV25_mV = 1430,         ///< Voltage at 25°C: 1.43V (in mV)
    kTempSensorCalibTemp_C = 25,      ///< Calibration temperature: 25°C
    kAdcVref_Mv = 3300,               ///< ADC reference voltage: 3.3V (in mV)
};

//private functions
static void readTemperatureData(void);

//variables
static int32_t latest_temperature_celsius = 0;           ///< Latest MCU internal temperature in [°C]
static uint32_t current_tick = 0;                        ///< Current OS tick
static SemaphoreHandle_t temperature_mutex = NULL;       ///< Mutex which protects the temperature readings
static volatile SemaphoreHandle_t updated_mutex = NULL;  ///< Binary semaphore indicating a new reading is available

/****************************************************************************************************************/
/****************************************************************************************************************/

/**
 * Handle an ADC1 interrupt
 */
void ADCinterruptTriggered(void) {
    LL_ADC_ClearFlag_EOS(ADC1);

    if (updated_mutex == NULL) {
        return;
    }

    BaseType_t has_woken = 0;
    (void)xSemaphoreGiveFromISR(updated_mutex, &has_woken);
    portYIELD_FROM_ISR(has_woken);
}

/**
 * Initialise the ADC1
 */
void initialiseUserADC(void) {
    static StaticSemaphore_t temperature_mutex_state = {0};  ///< ADC value mutex state variables
    static StaticSemaphore_t updated_mutex_state = {0};      ///< ADC value mutex state variables

    //create a semaphore to protect measurements
    temperature_mutex = xSemaphoreCreateMutexStatic(&temperature_mutex_state);
    if (!temperature_mutex) {
        Error_Handler();
    }

    //create a semaphore to use as an "updated" binary flag
    updated_mutex = xSemaphoreCreateBinaryStatic(&updated_mutex_state);
    if (!updated_mutex) {
        Error_Handler();
    }

    LL_ADC_EnableIT_EOS(ADC1);  // enable ADC EOC interrupt
    LL_ADC_Enable(ADC1);        // enable ADC

    //request a first conversion
    current_tick = HAL_GetTick();
    LL_ADC_REG_StartConversionSWStart(ADC1);
    while (xSemaphoreTake(updated_mutex, 0) == pdFALSE) {
        if (timeout(current_tick, kInitTimeoutMs)) {
            Error_Handler();
        }
    }

    //read the temperature value
    readTemperatureData();
    current_tick = HAL_GetTick();
}

/**
 * Run the ADC1 reading state machine
 */
void runUserADCstateMachine(void) {
    // State 1: Check if it's time to start a new conversion
    if (timeout(current_tick, kTemperatureRefreshMs)) {
        current_tick = HAL_GetTick();
        LL_ADC_REG_StartConversionSWStart(ADC1);
    }

    // State 2: Check if any conversion is ready to read (independent of timeout)
    if (xSemaphoreTake(updated_mutex, 0) == pdTRUE) {
        readTemperatureData();
    }
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

/****************************************************************************************************************/
/****************************************************************************************************************/

/**
 * Read the raw data converted by the internal MCU temperature ADC
 */
static void readTemperatureData(void) {
    uint8_t value_changed = 0;

    if (xSemaphoreTake(temperature_mutex, pdMS_TO_TICKS(kMutexMS)) == pdFALSE) {
        return;
    }

    const uint16_t temperature_raw_adc = LL_ADC_REG_ReadConversionData12(ADC1);
    int32_t new_temperature =
        __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(kTempSensorAvgSlope_mV_C, kTempSensorV25_mV, kTempSensorCalibTemp_C,
                                             kAdcVref_Mv, temperature_raw_adc, LL_ADC_RESOLUTION_12B);

    if (latest_temperature_celsius != new_temperature) {
        latest_temperature_celsius = new_temperature;
        value_changed = 1;
    }
    (void)xSemaphoreGive(temperature_mutex);

    if (value_changed) {
        triggerHardwareEvent(kEventTemperature);
    }
}
