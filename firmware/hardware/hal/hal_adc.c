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
#include <projdefs.h>
#include <semphr.h>
#include <stddef.h>
#include <stdint.h>
#include <stm32f103xb.h>
#include <stm32f1xx_ll_adc.h>

#include "hardware_events.h"
#include "systick.h"

enum {
    kMutexMS = 5U,          ///< Max number of milliseconds to wait for a mutex
    kInitTimeoutMs = 100U,  ///< Maximum number of milliseconds to wait for first reading

    // STM32F103 temperature sensor calibration parameters (from datasheet)
    kTempSensorAvgSlope_mV_C = 4300,  ///< Avg slope: 4.3 mV/°C (scaled by 1000)
    kTempSensorV25_mV = 1430,         ///< Voltage at 25°C: 1.43V (in mV)
    kTempSensorCalibTemp_C = 25,      ///< Calibration temperature: 25°C
    kAdcVref_Mv = 3300,               ///< ADC reference voltage: 3.3V (in mV)
};

//variables
static int32_t latest_temperature_celsius = 0;      ///< Latest MCU internal temperature in [°C]
static SemaphoreHandle_t temperature_mutex = NULL;  ///< Mutex which protects the temperature readings
static volatile uint8_t adc_updated = 0;            ///< Flag indicating whether the ADC finished updating

/****************************************************************************************************************/
/****************************************************************************************************************/

/**
 * Handle an ADC1 interrupt
 */
void ADCinterruptTriggered(void) {
    LL_ADC_ClearFlag_EOS(ADC1);
    adc_updated = 1;
}

/**
 * Initialise the ADC1
 */
void initialiseUserADC(void) {
    static StaticSemaphore_t temperature_mutex_state = {0};  ///< ADC value mutex state variables

    //create a semaphore to protect measurements
    temperature_mutex = xSemaphoreCreateMutexStatic(&temperature_mutex_state);
    if (!temperature_mutex) {
        Error_Handler();
    }

    LL_ADC_EnableIT_EOS(ADC1);  // enable ADC EOC interrupt
    LL_ADC_Enable(ADC1);        // enable ADC

    //request a first conversion
    uint32_t current_tick = getCurrentTick();
    LL_ADC_REG_StartConversionSWStart(ADC1);

    //read the temperature value
    while (!adc_updated) {
        if (systickTimeout(current_tick, kInitTimeoutMs)) {
            Error_Handler();
        }
    }
    adc_updated = 0;
    readTemperatureData();
}

/**
 * Check if the ADC finished updating
 * @note This will reset the internal flag
 *
 * @retval 1 ADC updated
 * @retval 0 Not updated yet
 */
uint8_t consume_adc_updated(void) {
    uint8_t is_updated = adc_updated;
    adc_updated = 0;

    return is_updated;
}

/**
 * Request an ADC read
 *
 * @retval 1 Request success
 * @retval 0 Could not request ADC read
 */
uint8_t requestADCread(void) {
    if (adc_updated) {
        return 0;
    }

    LL_ADC_REG_StartConversionSWStart(ADC1);
    return 1;
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

/**
 * Read the raw data converted by the internal MCU temperature ADC
 */
void readTemperatureData(void) {
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
