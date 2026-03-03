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
    kADCrefreshMs = 1000U,              ///< Timespan between two ADC readings in [ms]
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
static void updateADCvalues(void);
static void updateInternalTemperature(void);
static int32_t adcToInternalTemperature(uint16_t adc_raw);
static ErrorCode updateBatteryVoltage(void);
static uint16_t adcToBatteryVoltage_mV(uint16_t adc_raw);

//state variables
static volatile TaskHandle_t task_handle = NULL;    ///< handle of the FreeRTOS task
static SemaphoreHandle_t temperature_mutex = NULL;  ///< Mutex which protects the temperature readings
static ErrorCode result;                            ///< Current functions errorstack result
static int32_t internal_temperature_celsius = 0;    ///< Latest MCU internal temperature in [°C]
static uint32_t adc_vref_mv;                        ///< Current ADC Vref voltage in [mV]
static uint16_t battery_voltage_mv = 0;             ///< Current battery voltage in [mV]
static uint32_t current_tick = 0;                   ///< Current OS tick
static uint32_t last_adc_update_tick = 0;           ///< Last tick at which ADC was updated
static uint32_t last_battery_lvl_update_tick = 0;   ///< Last tick at which battery lvl was updated

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
    (void)requestADCmeasurement(kADC1);

    last_battery_lvl_update_tick = getCurrentTick();
    last_adc_update_tick = getCurrentTick();
    current_tick = getCurrentTick();
    while (1) {
        result = runButtonsStateMachine();
        if (isError(result)) {
            Error_Handler();
        }

        runLEDstateMachine();
        runADCstateMachine();
        updateADCvalues();
        updateInternalTemperature();
        (void)updateBatteryVoltage();

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

    adc_vref_mv = (kMCUvrefInt_mV * kAdcMaxValue) / vref_adc_raw;
}

/**
 * Update the MCU internal temperature
 */
static void updateInternalTemperature(void) {
    if (!systickTimeout(current_tick, kTemperatureRefreshMs)) {
        return;
    }
    current_tick = getCurrentTick();

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
static ErrorCode updateBatteryVoltage(void) {
    //check if it is time to update the battery percentage
    if (systickTimeout(last_battery_lvl_update_tick, kBatteryLvlUpdatePeriodMs)) {
        last_battery_lvl_update_tick = getCurrentTick();

        // //open the battery measurement path
        // LL_GPIO_SetOutputPin(BATT_EN_GPIO_Port, BATT_EN_Pin);
    }

    //get the latest battery value
    uint16_t battery_adc_raw = 0;
    if (!getADCvalue(kADC1, kADCchannelBattery, &battery_adc_raw)) {
        return kSuccessCode;
    }

    // //close the battery measurement path (saves energy)
    // LL_GPIO_ResetOutputPin(BATT_EN_GPIO_Port, BATT_EN_Pin);

    //transform the ADC value to [mV]
    battery_voltage_mv = adcToBatteryVoltage_mV(battery_adc_raw);

    return kSuccessCode;
}

/**
 * Transform an ADC value to battery voltage in [0.01V]
 *
 * @param adc_raw Value to transform
 * @return Battery voltage in [0.01V]
 */
static uint16_t adcToBatteryVoltage_mV(uint16_t adc_raw) {
    static const uint32_t kVoltageDividerHighKohms = 56UL;
    static const uint32_t kVoltageDividerLowKohms = 56UL;

    const uint32_t conversion_numerator = (adc_vref_mv * (kVoltageDividerHighKohms + kVoltageDividerLowKohms));
    static const uint32_t kConversionDenominator = (kAdcMaxValue * kVoltageDividerLowKohms);

    return (uint16_t)((adc_raw * conversion_numerator) / kConversionDenominator);
}
