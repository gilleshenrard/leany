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
    kRequestsLength = 5U,  ///< Maximum number of ADC requests in the queue
    kNoWait = 0,           ///< Used instead of 0 for readability

    // STM32F103 temperature sensor calibration parameters (from datasheet)
    kTempSensorAvgSlope_mV_C = 4300,  ///< Avg slope: 4.3 mV/°C (scaled by 1000)
    kTempSensorV25_mV = 1430,         ///< Voltage at 25°C: 1.43V (in mV)
    kTempSensorCalibTemp_C = 25,      ///< Calibration temperature: 25°C
    kAdcVref_Mv = 3300,               ///< ADC reference voltage: 3.3V (in mV)
};

/**
 * ADC request
 */
typedef struct {
    ADCchannel channel;  ///< ADC channel for which to request an update
} ADCrequest;

/**
 * ADC states
 */
typedef enum {
    kStateIdle = 0,       ///< No update ongoing
    kStateAcquiring = 1,  ///< ADC is updating
} ADCstate;

//private functions
static void stateIdle(void);
static void stateAcquiring(void);

//state variables
static volatile SemaphoreHandle_t adcdone_binsemaphore = NULL;  ///< Binary semaphore indicating whether done updating
static StaticSemaphore_t adcdone_binsemaphore_state;            ///< ADC done semaphore state
static QueueHandle_t adc_queue = NULL;                          ///< ADC requests queue
static StaticQueue_t adc_queue_state;                           ///< ADC requests queue state
static ADC_TypeDef* adc_handle = ADC1;                          ///< ADC handle to use
static ADCrequest requests[kRequestsLength];                    ///< Requests queue buffer
static ADCresult adc_values[kADCnbChannels];                    ///< Latest ADC values for all channels
static ADCstate state = kStateIdle;                             ///< Current state machine state
static ADCrequest latest_request;                               ///< Latest request treated
static uint32_t last_tick = 0;                                  ///< System tick at the start of a request

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * React to an ADC interrupt
 */
void ADCinterruptTriggered(void) {
    LL_ADC_ClearFlag_EOS(adc_handle);

    //semaphore not created yet
    if (!adcdone_binsemaphore) {
        return;
    }

    BaseType_t has_woken = pdFALSE;
    xSemaphoreGiveFromISR(adcdone_binsemaphore, &has_woken);
    portYIELD_FROM_ISR(has_woken);
}

/**
 * Initialise the HAL ADC elements, and enable ADC and its interrupt
 */
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

/**
 * Request an update on an ADC channel
 *
 * @param channel Channel to update
 * @retval 1 Request sent
 * @retval 0 Queue could not accept the request
 */
uint8_t requestADCmeasurement(ADCchannel channel) {
    const ADCrequest request = {.channel = channel};
    return (xQueueSend(adc_queue, &request, kNoWait) == pdTRUE);
}

/**
 * Get the current value of an ADC channel
 *
 * @param channel Channel of which get the value
 * @param[out] value ADC value
 * @return Whether the ADC channel has an update since last read
 */
uint8_t getADCvalue(ADCchannel channel, ADCresult* value) {
    uint8_t updated = 0;

    //critical section used for non-blocking section that cannot fail
    taskENTER_CRITICAL();
    value->value = adc_values[channel].value;
    updated = adc_values[channel].updated;
    adc_values[channel].updated = 0;
    taskEXIT_CRITICAL();

    return updated;
}

/**
 * Run the current ADC state
 */
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

/**
 * Compute the MCU internal temperature from a raw ADC value
 *
 * @param adc_raw ADC raw value
 * @return MCU internal temperature
 */
int32_t adcToInternalTemperature(const uint16_t adc_raw) {
    return __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(kTempSensorAvgSlope_mV_C, kTempSensorV25_mV, kTempSensorCalibTemp_C,
                                                kAdcVref_Mv, adc_raw, LL_ADC_RESOLUTION_12B);
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * State in which a request is pulled from the queue
 */
static void stateIdle(void) {
    //pull a request from the request queue
    if (xQueueReceive(adc_queue, &latest_request, kNoWait) == pdFALSE) {
        return;
    }

    //drain the ADC-done semaphore and start acquisition
    xSemaphoreTake(adcdone_binsemaphore, 0);
    LL_ADC_REG_StartConversionSWStart(adc_handle);

    //get to next state
    last_tick = getCurrentTick();
    state = kStateAcquiring;
}

/**
 * State in which the current request is treated
 */
static void stateAcquiring(void) {
    //if too long, abort
    if (systickTimeout(last_tick, kADCtimeoutMs)) {
        state = kStateIdle;
        return;
    }

    //if not done yet, exit
    if (xSemaphoreTake(adcdone_binsemaphore, kNoWait) == pdFALSE) {
        return;
    }

    //critical section used for non-blocking section that cannot fail
    taskENTER_CRITICAL();
    adc_values[latest_request.channel].value = LL_ADC_REG_ReadConversionData12(adc_handle);
    adc_values[latest_request.channel].updated = 1;
    taskEXIT_CRITICAL();

    state = kStateIdle;
}
