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
#include <assert.h>
#include <main.h>
#include <portmacro.h>
#include <projdefs.h>
#include <semphr.h>
#include <stddef.h>
#include <stdint.h>
#include <stm32f1xx_hal_def.h>
#include <stm32f1xx_ll_adc.h>
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
    kADCrefreshMs = 1000U,          ///< Timespan between two ADC readings in [ms]
    kMutexMS = 5U,                  ///< Max number of milliseconds to wait for a mutex
    kVrefUpdatePeriodMs = 1000U,    ///< Period in [ms] between two Vref updates

    // STM32F103 temperature sensor calibration parameters (from datasheet)
    kTempSensorAvgSlope_uV_C = 4300,  ///< Avg slope: 4.3 mV/°C (scaled to uV/°C)
    kTempSensorV25_mV = 1430,         ///< Voltage at 25°C: 1.43V (in mV)
    kTempSensorCalibTemp_C = 25,      ///< Calibration temperature: 25°C
    kAdcMaxValue = 4095,              ///< Maximum ADC LSB value (12-bits -> [0 ... 4095])
    kMCUvrefInt_mV = 1200,            ///< MCU VrefInt voltage in [mV]
};

//task functions
static void taskGPIO(void* argument);
static void updateADCvalues(void);
static void updateInternalTemperature(void);
static int32_t adcToInternalTemperature(uint16_t adc_raw);

//state variables
static volatile TaskHandle_t task_handle = NULL;    ///< handle of the FreeRTOS task
static SemaphoreHandle_t temperature_mutex = NULL;  ///< Mutex which protects the temperature readings
static SemaphoreHandle_t reference_mutex = NULL;    ///< Mutex which protects the ADC reference voltage readings
static int32_t internal_temperature_celsius = 0;    ///< Latest MCU internal temperature in [°C]
static uint32_t adc_vref_mv;                        ///< Current ADC Vref voltage in [mV]
static uint32_t last_temperature_update_tick = 0;   ///< Last tick at which temperature was updated
static uint32_t last_adc_update_tick = 0;           ///< Last tick at which ADC was updated

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Create FreeRTOS static task for the GPIO modules
 */
void createGPIOtask(void) {
    static StackType_t task_stack[kStackSize] = {0};         ///< Buffer used as the task stack
    static StaticTask_t task_state = {0};                    ///< Task state variables
    static StaticSemaphore_t temperature_mutex_state = {0};  ///< internal temperature mutex state variables
    static StaticSemaphore_t reference_mutex_state = {0};    ///< ADC reference voltage mutex state variables

    //create a mutex to protect temperature measurements
    temperature_mutex = xSemaphoreCreateMutexStatic(&temperature_mutex_state);
    configASSERT(temperature_mutex);

    //create a mutex to protect ADC reference voltage measurements
    reference_mutex = xSemaphoreCreateMutexStatic(&reference_mutex_state);
    configASSERT(reference_mutex);

    //create the static task
    task_handle = xTaskCreateStatic(taskGPIO, "GPIO task", kStackSize, NULL, kTaskLowPriority, task_stack, &task_state);
    configASSERT(task_handle);
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

    *temperature_celsius = internal_temperature_celsius;
    (void)xSemaphoreGive(temperature_mutex);

    return 1;
}

/**
 * Get the current ADC reference voltage
 *
 * @param[out] voltage_mv Reference voltage in [mV]
 * @retval 1 Success
 * @retval 0 Could not retrieve voltage reference
 */
uint8_t getADCreference_mV(uint32_t* voltage_mv) {
    if (!voltage_mv) {
        return 0;
    }

    uint8_t success = 0;

    if (xSemaphoreTake(reference_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        *voltage_mv = adc_vref_mv;
        success = 1;
        (void)xSemaphoreGive(reference_mutex);
    }

    return success;
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

    ErrorCode result;

    initialiseLED();
    initialiseHALadc();

    (void)requestADCmeasurement(kADC1);

    last_adc_update_tick = getCurrentTick();
    last_temperature_update_tick = getCurrentTick();
    while (1) {
        result = runButtonsStateMachine();
        if (isError(result)) {
            Error_Handler();
        }

        runLEDstateMachine();
        runADCstateMachine();
        updateADCvalues();
        updateInternalTemperature();

        vTaskDelay(pdMS_TO_TICKS(5U));
    }
}

/**
 * Read the ADC values and update the ADC reference voltage
 */
static void updateADCvalues(void) {
    if (systickTimeout(last_adc_update_tick, kADCrefreshMs)) {
        last_adc_update_tick = getCurrentTick();
        (void)requestADCmeasurement(kADC1);
    }

    uint16_t vref_adc_raw = 0;
    if (!getADCvalue(kADC1, kADCchannelVrefInt, &vref_adc_raw)) {
        return;
    }

    /*
     * Compute the actual VDDA (ADC reference voltage) by using the MCU internal bandgap reference (VrefInt).
     *
     * VDDA cannot be assumed to equal 3.3V: it is the LDO output, which droops
     * as the battery discharges. Instead, VrefInt (1.2V, stable regardless of VDDA)
     * is used as a known anchor.
     *
     * The ADC measures ratios against VDDA:
     *   vref_adc_raw = (VrefInt / VDDA) * 4095
     *
     * Rearranging to isolate VDDA:
     *   VDDA = (VrefInt * 4095) / vref_adc_raw
     */
    if (xSemaphoreTake(reference_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        adc_vref_mv = (kMCUvrefInt_mV * kAdcMaxValue) / vref_adc_raw;
        (void)xSemaphoreGive(reference_mutex);
    }
}

/**
 * Update the MCU internal temperature
 */
static void updateInternalTemperature(void) {
    // If not time to update the temperature, exit
    if (!systickTimeout(last_temperature_update_tick, kTemperatureRefreshMs)) {
        return;
    }
    last_temperature_update_tick = getCurrentTick();

    // Check if any conversion is ready to read (independent of timeout)
    uint16_t adc_temp_raw = 0;
    if (!getADCvalue(kADC1, kADCchannelTemperature, &adc_temp_raw)) {
        return;
    }

    // check if temperature changed
    uint8_t value_changed = 0;
    if (xSemaphoreTake(temperature_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        int32_t new_temperature = adcToInternalTemperature(adc_temp_raw);

        if (internal_temperature_celsius != new_temperature) {
            internal_temperature_celsius = new_temperature;
            value_changed = 1;
        }
        (void)xSemaphoreGive(temperature_mutex);
    }

    // trigger hardware event on temp. changed
    if (value_changed) {
        triggerHardwareEvent(kEventTemperature);
    }
}

/**
 * Compute the MCU internal temperature from a raw ADC value
 *
 * @param adc_raw ADC raw value
 * @return MCU internal temperature in [°C]
 */
static int32_t adcToInternalTemperature(const uint16_t adc_raw) {
    if (!adc_vref_mv) {
        return 0;
    }

    return __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(kTempSensorAvgSlope_uV_C, kTempSensorV25_mV, kTempSensorCalibTemp_C,
                                                adc_vref_mv, adc_raw, LL_ADC_RESOLUTION_12B);
}
