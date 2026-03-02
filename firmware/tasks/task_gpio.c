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
#include <stm32f1xx_ll_adc.h>
#include <stm32f1xx_ll_gpio.h>
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
    kMutexMS = 5U,                      ///< Max number of milliseconds to wait for a mutex
    kBatteryLvlUpdatePeriodMs = 1000U,  ///< Period in [ms] between two battery level updates
    kVrefUpdatePeriodMs = 1000U,        ///< Period in [ms] between two Vref updates

    // STM32F103 temperature sensor calibration parameters (from datasheet)
    kTempSensorAvgSlope_uV_C = 4300,  ///< Avg slope: 4.3 mV/°C (scaled to uV/°C)
    kTempSensorV25_mV = 1430,         ///< Voltage at 25°C: 1.43V (in mV)
    kTempSensorCalibTemp_C = 25,      ///< Calibration temperature: 25°C
    kAdcMaxValue = 4095,              ///< Maximum ADC LSB value (12-bits -> [0 ... 4095])
    kMCUvrefInt_mV = 1200,            ///< MCU VrefInt voltage in [mV]
};

//task functions
static void taskGPIO(void* argument);
static void updateInternalTemperature(void);
static int32_t adcToInternalTemperature(uint16_t adc_raw);
static ErrorCode requestBatteryRead(void);
static uint16_t adcToVoltage_mV(uint16_t adc_raw);
static ErrorCode updateBatteryLevel(void);
static void updateVref(void);

//state variables
static volatile TaskHandle_t task_handle = NULL;    ///< handle of the FreeRTOS task
static SemaphoreHandle_t temperature_mutex = NULL;  ///< Mutex which protects the temperature readings
static ErrorCode result;                            ///< Current functions errorstack result
static int32_t internal_temperature_celsius = 0;    ///< Latest MCU internal temperature in [°C]
static uint32_t adc_vref_mv;                        ///< Current ADC Vref voltage in [mV]
static uint16_t battery_voltage_mv = 0;             ///< Current battery voltage in [mV]
static uint32_t current_tick = 0;                   ///< Current OS tick
static uint32_t last_battery_lvl_update_tick = 0;   ///< Last tick at which battery lvl was updated
static uint32_t last_vref_update_tick = 0;          ///< Last tick at which battery lvl was updated

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

    LL_GPIO_SetOutputPin(BATT_EN_GPIO_Port, BATT_EN_Pin);
    (void)requestBatteryRead();

    last_battery_lvl_update_tick = getCurrentTick();
    last_vref_update_tick = getCurrentTick();
    current_tick = getCurrentTick();
    while (1) {
        result = runButtonsStateMachine();
        if (isError(result)) {
            Error_Handler();
        }

        runLEDstateMachine();
        runADCstateMachine();
        updateVref();
        updateInternalTemperature();
        (void)requestBatteryRead();
        (void)updateBatteryLevel();

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
 * @return MCU internal temperature
 */
static int32_t adcToInternalTemperature(const uint16_t adc_raw) {
    return __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(kTempSensorAvgSlope_uV_C, kTempSensorV25_mV, kTempSensorCalibTemp_C,
                                                adc_vref_mv, adc_raw, LL_ADC_RESOLUTION_12B);
}

/**
 * Request a battery read on ADC
 *
 * @return ErrorCode 
 */
static ErrorCode requestBatteryRead(void) {
    //check if it is time to update the battery percentage
    if (!systickTimeout(last_battery_lvl_update_tick, kBatteryLvlUpdatePeriodMs)) {
        return kSuccessCode;
    }
    last_battery_lvl_update_tick = getCurrentTick();

    // //open the battery measurement path
    // LL_GPIO_SetOutputPin(BATT_EN_GPIO_Port, BATT_EN_Pin);

    //request ADC measurements
    if (!requestADCmeasurement(kADCchannelBattery)) {
        // LL_GPIO_ResetOutputPin(BATT_EN_GPIO_Port, BATT_EN_Pin);
        return kSuccessCode;
    }

    return kSuccessCode;
}

/**
 * Update battery level upon ADC completion
 *
 * @return ErrorCode 
 */
static ErrorCode updateBatteryLevel(void) {
    //get the latest battery value
    ADCresult adc_result;
    if (!getADCvalue(kADCchannelBattery, &adc_result)) {
        return kSuccessCode;
    }

    // //close the battery measurement path (saves energy)
    // LL_GPIO_ResetOutputPin(BATT_EN_GPIO_Port, BATT_EN_Pin);

    //transform the ADC value to [mV]
    battery_voltage_mv = adcToVoltage_mV(adc_result.value);

    return kSuccessCode;
}

/**
 * Transform an ADC value to battery voltage in [0.01V]
 *
 * @param adc_raw Value to transform
 * @return Battery voltage in [0.01V]
 */
static uint16_t adcToVoltage_mV(uint16_t adc_raw) {
    static const uint32_t kVoltageDividerHighKohms = 56UL;
    static const uint32_t kVoltageDividerLowKohms = 56UL;

    const uint32_t conversion_numerator = (adc_vref_mv * (kVoltageDividerHighKohms + kVoltageDividerLowKohms));
    static const uint32_t kConversionDenominator = (kAdcMaxValue * kVoltageDividerLowKohms);

    return (uint16_t)((adc_raw * conversion_numerator) / kConversionDenominator);
}

/**
 * Update the ADC voltage reference
 *
 * @return Success
 */
static void updateVref(void) {
    // State 1: Check if it's time to start a new conversion
    if (systickTimeout(last_vref_update_tick, kVrefUpdatePeriodMs)) {
        last_vref_update_tick = getCurrentTick();
        (void)requestADCmeasurement(kADCchannelVrefInt);
    }

    // State 2: Check if any conversion is ready to read (independent of timeout)
    ADCresult adc_result;
    const uint8_t updated = getADCvalue(kADCchannelVrefInt, &adc_result);
    if (!updated) {
        return;
    }

    adc_vref_mv = (kMCUvrefInt_mV * kAdcMaxValue) / adc_result.value;
}
