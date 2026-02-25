/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file hal_adc.c
 * @author Gilles Henrard
 */
#include "hal_adc.h"

#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <portmacro.h>
#include <projdefs.h>
#include <queue.h>
#include <semphr.h>
#include <stdint.h>
#include <stm32f103xb.h>
#include <stm32f1xx_ll_adc.h>

#include "systick.h"

enum {
    kADCtimeoutMs = 100U,  ///< Maximum number of milliseconds to wait for ADC reading
    kRequestsLength = 5U,
    kNoWait = 0,

    // STM32F103 temperature sensor calibration parameters (from datasheet)
    kTempSensorAvgSlope_mV_C = 4300,  ///< Avg slope: 4.3 mV/°C (scaled by 1000)
    kTempSensorV25_mV = 1430,         ///< Voltage at 25°C: 1.43V (in mV)
    kTempSensorCalibTemp_C = 25,      ///< Calibration temperature: 25°C
    kAdcVref_Mv = 3300,               ///< ADC reference voltage: 3.3V (in mV)
};

typedef struct {
    ADCchannel channel;
} ADCrequest;

typedef enum {
    kStateIdle = 0,
    kStateAcquiring = 1,
} ADCstate;

static void stateIdle(void);
static void stateAcquiring(void);

static volatile SemaphoreHandle_t adcdone_binsemaphore = NULL;
static StaticSemaphore_t adcdone_binsemaphore_state;
static QueueHandle_t adc_queue = NULL;
static StaticQueue_t adc_queue_state;
static ADC_TypeDef* adc_handle = ADC1;
static ADCrequest requests[kRequestsLength];
static ADCresult adc_values[kADCnbChannels];
static ADCstate state = kStateIdle;
static ADCrequest latest_request;
static uint32_t last_tick = 0;

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

void ADCinterruptTriggered(void) {
    LL_ADC_ClearFlag_EOS(adc_handle);

    if (!adcdone_binsemaphore) {
        return;
    }

    BaseType_t has_woken = pdFALSE;
    xSemaphoreGiveFromISR(adcdone_binsemaphore, &has_woken);
    portYIELD_FROM_ISR(has_woken);
}

void initialiseHALadc(void) {
    //create the ADC-done binary semaphore
    adcdone_binsemaphore = xSemaphoreCreateBinaryStatic(&adcdone_binsemaphore_state);
    configASSERT(adcdone_binsemaphore);

    //create the ADC requests queue
    adc_queue = xQueueCreateStatic(kRequestsLength, sizeof(ADCrequest), (uint8_t*)requests, &adc_queue_state);
    configASSERT(adc_queue);

    //Enable the ADC
    LL_ADC_EnableIT_EOS(adc_handle);  // enable ADC EOC interrupt
    LL_ADC_Enable(adc_handle);        // enable ADC
}

uint8_t requestADCmeasurement(ADCchannel channel) {
    const ADCrequest request = {.channel = channel};
    return (xQueueSend(adc_queue, &request, kNoWait) == pdTRUE);
}

uint8_t getADCvalue(ADCchannel channel, ADCresult* value) {
    uint8_t updated = 0;

    taskENTER_CRITICAL();
    value->value = adc_values[channel].value;
    updated = adc_values[channel].updated;
    adc_values[channel].updated = 0;
    taskEXIT_CRITICAL();

    return updated;
}

void runADCstateMachine(void) {
    switch (state) {
        case kStateIdle:
            stateIdle();
            break;

        case kStateAcquiring:
            stateAcquiring();
            break;

        default:
            break;
    }
}

int32_t adcToInternalTemperature(const uint16_t adc_raw) {
    return __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(kTempSensorAvgSlope_mV_C, kTempSensorV25_mV, kTempSensorCalibTemp_C,
                                                kAdcVref_Mv, adc_raw, LL_ADC_RESOLUTION_12B);
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

static void stateIdle(void) {
    if (xQueueReceive(adc_queue, &latest_request, kNoWait) == pdFALSE) {
        return;
    }

    //drain the ADC-done semaphore and start acquisition
    xSemaphoreTake(adcdone_binsemaphore, 0);
    LL_ADC_REG_StartConversionSWStart(adc_handle);

    last_tick = getCurrentTick();
    state = kStateAcquiring;
}

static void stateAcquiring(void) {
    if (systickTimeout(last_tick, kADCtimeoutMs)) {
        state = kStateIdle;
        return;
    }

    if (xSemaphoreTake(adcdone_binsemaphore, kNoWait) == pdFALSE) {
        return;
    }

    taskENTER_CRITICAL();
    adc_values[latest_request.channel].value = LL_ADC_REG_ReadConversionData12(adc_handle);
    adc_values[latest_request.channel].updated = 1;
    taskEXIT_CRITICAL();

    state = kStateIdle;
}
