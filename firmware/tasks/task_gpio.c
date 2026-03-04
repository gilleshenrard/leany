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
    kStackSize = 150U,                  ///< Amount of words in the task stack
    kTaskLowPriority = 8U,              ///< FreeRTOS number for a low priority task
    kTemperatureRefreshMs = 2000U,      ///< Timespan between two readings in [ms]
    kADCrefreshMs = 1000U,              ///< Timespan between two ADC readings in [ms]
    kMutexMS = 5U,                      ///< Max number of milliseconds to wait for a mutex
    kBatteryLvlUpdatePeriodMs = 1000U,  ///< Period in [ms] between two battery level updates
    kVrefUpdatePeriodMs = 1000U,        ///< Period in [ms] between two Vref updates
    kNbAverageSamples = 8U,             ///< Maximum number of elements in the average buffer

    // STM32F103 temperature sensor calibration parameters (from datasheet)
    kTempSensorAvgSlope_uV_C = 4300,  ///< Avg slope: 4.3 mV/°C (scaled to uV/°C)
    kTempSensorV25_mV = 1430,         ///< Voltage at 25°C: 1.43V (in mV)
    kTempSensorCalibTemp_C = 25,      ///< Calibration temperature: 25°C
    kAdcMaxValue = 4095,              ///< Maximum ADC LSB value (12-bits -> [0 ... 4095])
    kMCUvrefInt_mV = 1200,            ///< MCU VrefInt voltage in [mV]
};

static_assert(((uint8_t)kNbAverageSamples & ((uint8_t)kNbAverageSamples - 1U)) == 0U,
              "kNbAverageSamples must be a power of 2");

//task functions
static void taskGPIO(void* argument);
static void updateADCvalues(void);
static void updateInternalTemperature(void);
static int32_t adcToInternalTemperature(uint16_t adc_raw);
static void updateBatteryVoltage(void);
static uint16_t adcToBatteryVoltage_mV(uint16_t adc_raw);
static void averageBatteryVoltageMv(uint32_t new_voltage_mv);

//state variables
static volatile TaskHandle_t task_handle = NULL;                 ///< handle of the FreeRTOS task
static SemaphoreHandle_t temperature_mutex = NULL;               ///< Mutex which protects the temperature readings
static SemaphoreHandle_t battery_mutex = NULL;                   ///< Mutex which protects the battery readings
static ErrorCode result;                                         ///< Current functions errorstack result
static int32_t internal_temperature_celsius = 0;                 ///< Latest MCU internal temperature in [°C]
static uint32_t adc_vref_mv;                                     ///< Current ADC Vref voltage in [mV]
static uint16_t battery_voltage_mv = 0;                          ///< Current battery voltage in [mV]
static uint32_t last_temperature_update_tick = 0;                ///< Last tick at which temperature was updated
static uint32_t last_adc_update_tick = 0;                        ///< Last tick at which ADC was updated
static uint32_t last_battery_lvl_update_tick = 0;                ///< Last tick at which battery lvl was updated
static uint32_t battery_average_queue[kNbAverageSamples] = {0};  ///< Buffer used for averaging battery
static uint8_t battery_average_nbsamples = 0;                    ///< Number of samples saved in the buffer
static uint8_t battery_average_index = 0;                        ///< Index of the current buffer head
static uint32_t battery_average_total = 0;                       ///< Total value used to calculate battery average

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Create FreeRTOS static task for the buttons
 */
void createGPIOtask(void) {
    static StackType_t task_stack[kStackSize] = {0};         ///< Buffer used as the task stack
    static StaticTask_t task_state = {0};                    ///< Task state variables
    static StaticSemaphore_t temperature_mutex_state = {0};  ///< ADC value mutex state variables
    static StaticSemaphore_t battery_mutex_state = {0};      ///< battery value mutex state variables

    //create a semaphore to protect measurements
    temperature_mutex = xSemaphoreCreateMutexStatic(&temperature_mutex_state);
    configASSERT(temperature_mutex);

    //create a semaphore to protect battery measurements
    battery_mutex = xSemaphoreCreateMutexStatic(&battery_mutex_state);
    configASSERT(battery_mutex);

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
 * Get the latest measured battery voltage
 *
 * @param[out] voltage_mv Battery voltage in [mV]
 * @retval 1 Successfully retrieved
 * @retval 0 Could not retrieve voltage
 */
uint8_t getBatteryVoltageMv(uint16_t* voltage_mv) {
    uint8_t success = 0;
    if (xSemaphoreTake(temperature_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        *voltage_mv = battery_voltage_mv;
        success = 1;
        xSemaphoreGive(temperature_mutex);
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

    initialiseLED();
    initialiseHALadc();

    (void)requestADCmeasurement(kADC1);

    last_battery_lvl_update_tick = getCurrentTick();
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
        updateBatteryVoltage();

        vTaskDelay(pdMS_TO_TICKS(5U));
    }
}

/**
 * Update the MCU internal temperature
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
    adc_vref_mv = (kMCUvrefInt_mV * kAdcMaxValue) / vref_adc_raw;
}

/**
 * Update the MCU internal temperature
 */
static void updateInternalTemperature(void) {
    if (!systickTimeout(last_temperature_update_tick, kTemperatureRefreshMs)) {
        return;
    }
    last_temperature_update_tick = getCurrentTick();

    // State 2: Check if any conversion is ready to read (independent of timeout)
    uint16_t adc_temp_raw = 0;
    if (!getADCvalue(kADC1, kADCchannelTemperature, &adc_temp_raw)) {
        return;
    }

    uint8_t value_changed = 0;
    if (xSemaphoreTake(temperature_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        int32_t new_temperature = adcToInternalTemperature(adc_temp_raw);

        if (internal_temperature_celsius != new_temperature) {
            internal_temperature_celsius = new_temperature;
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
 * @return MCU internal temperature in [°C]
 */
static int32_t adcToInternalTemperature(const uint16_t adc_raw) {
    if (!adc_vref_mv) {
        return 0;
    }

    return __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(kTempSensorAvgSlope_uV_C, kTempSensorV25_mV, kTempSensorCalibTemp_C,
                                                adc_vref_mv, adc_raw, LL_ADC_RESOLUTION_12B);
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

    //wait for 2ms after GPIO being set high
    if (!updating || !systickTimeout(last_battery_lvl_update_tick, 2U)) {
        return;
    }

    //get the latest battery value
    uint16_t battery_adc_raw = 0;
    if (!getADCvalue(kADC1, kADCchannelBattery, &battery_adc_raw)) {
        return;
    }

    //transform the ADC value to [mV]
    const uint16_t new_battery_voltage_mv = adcToBatteryVoltage_mV(battery_adc_raw);
    averageBatteryVoltageMv(new_battery_voltage_mv);
    updating = 0;
}

/**
 * Transform an ADC value to battery voltage in [mV]
 *
 * @param adc_raw Value to transform
 * @return Battery voltage in [mV]
 */
static uint16_t adcToBatteryVoltage_mV(uint16_t adc_raw) {
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
        if (xSemaphoreTake(temperature_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
            battery_voltage_mv = (uint16_t)(battery_average_total / battery_average_nbsamples);
            xSemaphoreGive(temperature_mutex);
        }
        return;
    }

    //buffer is full, use a cyclic average
    battery_average_index = (battery_average_index + 1U) % kNbAverageSamples;
    battery_average_total -= battery_average_queue[battery_average_index];
    battery_average_queue[battery_average_index] = new_voltage_mv;
    battery_average_total += new_voltage_mv;
    if (xSemaphoreTake(temperature_mutex, pdMS_TO_TICKS(kMutexMS)) == pdTRUE) {
        battery_voltage_mv = (uint16_t)(battery_average_total / kNbAverageSamples);
        xSemaphoreGive(temperature_mutex);
    }
}
