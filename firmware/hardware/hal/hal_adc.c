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
#include <main.h>
#include <portmacro.h>
#include <projdefs.h>
#include <semphr.h>
#include <stdint.h>
#include <stm32f103xb.h>
#include <stm32f1xx_ll_adc.h>
#include <stm32f1xx_ll_dma.h>
#include <stm32f1xx_ll_gpio.h>

#include "systick.h"

enum {
    kADCtimeoutMs = 100U,   ///< Maximum number of milliseconds to wait for ADC reading
    kADCcalibWaitMs = 10U,  ///< Number of milliseconds to wait between calibration stages
    kRequestsLength = 5U,   ///< Maximum number of ADC requests in the queue
};

/**
 * ADC states
 */
typedef enum {
    kStateIdle = 0,       ///< No update ongoing
    kStateAcquiring = 1,  ///< ADC is updating
} ADCstate;

/**
 * ADC to DMA mapping
 */
typedef struct {
    DMA_TypeDef* dma_handle;  ///< DMA handle used
    ADC_TypeDef* adc_handle;  ///< ADC handle used
    uint32_t dma_channel;     ///< DMA channel
    uint8_t nb_channels;      ///< Number of ADC channels read in sequence
    uint16_t* update_values;  ///< Values updated by the ADC
    uint16_t* latest_values;  ///< Copy of values, available to tasks
} ADCmapping;

typedef uint8_t RequestIDType;  ///< Type of a request ID

//state functions
static void stateIdle(void);
static void stateAcquiring(void);

//state variables
static volatile SemaphoreHandle_t adcdone_binsemaphore = NULL;  ///< Binary semaphore indicating whether done updating
static StaticSemaphore_t adcdone_binsemaphore_state;            ///< ADC done semaphore state
static QueueHandle_t adc_queue = NULL;                          ///< ADC requests queue
static StaticQueue_t adc_queue_state;                           ///< ADC requests queue state
static TickType_t last_tick = 0;                                ///< System tick at the start of a request
static const TickType_t kNoWait = 0;                            ///< Used instead of 0 for readability
static ADCstate state = kStateIdle;                             ///< Current state machine state
static RequestIDType latest_request = 0;                        ///< Latest request ID received
static RequestIDType requests[kRequestsLength] = {0};           ///< Buffer for the requests queue
static uint16_t dma1_latest_values[kADC1nbChannels] = {0};      ///< Buffer of the latest values read by DMA1
static uint16_t dma1_update_values[kADC1nbChannels] = {0};      ///< Buffer of the copy of values read by DMA1
static ADCmapping devices[kADCnbDevices] =                      ///< ADC device mappings
    {
        [kADC1] = {.dma_handle = DMA1,
                   .adc_handle = ADC1,
                   .dma_channel = LL_DMA_CHANNEL_1,
                   .nb_channels = kADC1nbChannels,
                   .update_values = dma1_update_values,
                   .latest_values = dma1_latest_values},
};

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * React to an ADC interrupt
 */
void ADCinterruptTriggered(void) {
    //semaphore not created yet
    if (!adcdone_binsemaphore) {
        return;
    }

    LL_DMA_ClearFlag_TC1(devices[latest_request].dma_handle);

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
    adc_queue = xQueueCreateStatic(kRequestsLength, sizeof(RequestIDType), (uint8_t*)requests, &adc_queue_state);
    configASSERT(adc_queue);

    for (uint8_t device = 0; device < (uint8_t)kADCnbDevices; device++) {
        const ADCmapping* mapping = &devices[device];

        //ADC needs to be disabled for calibration
        LL_ADC_Disable(mapping->adc_handle);
        last_tick = getCurrentTick();
        while (LL_ADC_IsEnabled(mapping->adc_handle) && !systickTimeout(last_tick, kADCcalibWaitMs)) {
        }

        //Wait for at least 2 ADC clock cycles before calibration
        last_tick = getCurrentTick();
        while (!systickTimeout(last_tick, kADCcalibWaitMs)) {
        }

        //Start calibration
        last_tick = getCurrentTick();
        LL_ADC_StartCalibration(mapping->adc_handle);
        while (LL_ADC_IsCalibrationOnGoing(mapping->adc_handle) && !systickTimeout(last_tick, kADCcalibWaitMs)) {
        }

        //Re-enable the ADC
        LL_ADC_Enable(mapping->adc_handle);
    }
}

/**
 * Request an update on an ADC device
 *
 * @param device Device to update
 * @retval 1 Request sent
 * @retval 0 Queue could not accept the request
 */
uint8_t requestADCmeasurement(ADCdevice device) {
    if (device >= kADCnbDevices) {
        return 0;
    }

    const RequestIDType device_id = (RequestIDType)device;
    return (xQueueSend(adc_queue, &device_id, kNoWait) == pdTRUE);
}

/**
 * Get the current value of an ADC channel
 *
 * @param device Device from which to retrieve the value
 * @param channel Channel of which get the value
 * @param[out] value ADC value
 * @return Whether retrieval succeeded
 */
uint8_t getADCvalue(ADCdevice device, uint8_t channel, uint16_t* value) {
    if (device >= kADCnbDevices) {
        return 0;
    }

    if (channel >= devices[device].nb_channels) {
        return 0;
    }

    if (!value) {
        return 0;
    }

    //critical section used for non-blocking section that cannot fail
    taskENTER_CRITICAL();
    *value = devices[device].latest_values[channel];
    taskEXIT_CRITICAL();

    return 1;
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

    //drain the ADC-done semaphore
    xSemaphoreTake(adcdone_binsemaphore, 0);

    const ADCmapping* mapping = &devices[latest_request];

    LL_GPIO_SetOutputPin(BATT_EN_GPIO_Port, BATT_EN_Pin);

    //start DMA acquisition
    LL_DMA_DisableChannel(mapping->dma_handle, mapping->dma_channel);
    LL_DMA_ClearFlag_GI1(mapping->dma_handle);
    LL_DMA_EnableIT_TC(mapping->dma_handle, mapping->dma_channel);
    LL_DMA_ConfigAddresses(mapping->dma_handle, mapping->dma_channel,
                           LL_ADC_DMA_GetRegAddr(mapping->adc_handle, LL_ADC_DMA_REG_REGULAR_DATA),
                           (uint32_t)mapping->update_values, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetDataLength(mapping->dma_handle, mapping->dma_channel, mapping->nb_channels);  //must be reset every time
    LL_DMA_EnableChannel(mapping->dma_handle, mapping->dma_channel);
    LL_ADC_REG_StartConversionSWStart(mapping->adc_handle);

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
        LL_DMA_DisableChannel(devices[latest_request].dma_handle, devices[latest_request].dma_channel);
        state = kStateIdle;
        return;
    }

    //if not done yet, exit
    if (xSemaphoreTake(adcdone_binsemaphore, kNoWait) == pdFALSE) {
        return;
    }

    //critical section used for non-blocking section that cannot fail
    taskENTER_CRITICAL();
    for (uint8_t channel = 0; channel < devices[latest_request].nb_channels; channel++) {
        devices[latest_request].latest_values[channel] = devices[latest_request].update_values[channel];
    }
    taskEXIT_CRITICAL();

    LL_GPIO_ResetOutputPin(BATT_EN_GPIO_Port, BATT_EN_Pin);

    state = kStateIdle;
}
