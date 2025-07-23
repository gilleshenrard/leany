/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file halspi.c
 * @brief Implement a generic HAL for SPI communication
 * @author Gilles Henrard
 * @date 26/07/2025
 */
#include "halspi.h"

#include <main.h>
#include <portmacro.h>
#include <projdefs.h>
#include <stddef.h>
#include <stdint.h>
#include <stm32f1xx_hal.h>
#include <stm32f1xx_ll_dma.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_spi.h>

#include "errorstack.h"

/**
 * @brief SPI Data/command pin status enumeration
 */
typedef enum {
    kCommand = 0,  ///< Command is to be sent
    kData,         ///< Data is to be sent
} DCgpio;

/**
 * Enumeration of the function IDs
 */
typedef enum {
    kSPIreadRegisters = 1U,           ///< Function ID for the readRegisters()
    kSPIwriteRegistersContinue = 2U,  ///< Function ID for the writeRegistersAndContinue()
    kSPIdmaSend = 3U,                 ///< Function ID for the sendDMA()
} FunctionCode;

//state variables

static inline void setDataCommandGPIO(const SPI* descriptor, DCgpio function);
static void sendSPIbyte(SPI* descriptor, uint8_t byte_to_transmit, uint32_t tx_start_tick);
static volatile TaskHandle_t latest_task = NULL;  ///< Handle used by the FreeRTOS task

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Handler called upon DMA transfer complete
 */
void st7735sDMAinterruptHandler(void) {
    BaseType_t has_woken = 0;
    vTaskNotifyGiveFromISR(latest_task, &has_woken);
    portYIELD_FROM_ISR(has_woken);
}

/**
 * brief Set the Data/Command pin
 * 
 * @param descriptor SPI descriptor to use
 * @param function Value of the data/command pin
 */
static inline void setDataCommandGPIO(const SPI* descriptor, DCgpio function) {
    if (!descriptor->data_command_port) {
        return;
    }

    if (function == kCommand) {
        LL_GPIO_ResetOutputPin(descriptor->data_command_port, descriptor->data_command_pin);
    } else {
        LL_GPIO_SetOutputPin(descriptor->data_command_port, descriptor->data_command_pin);
    }
}

/**
 * Terminate a SPI transmission
 *
 * @param descriptor SPI descriptor to use
 */
void closeTransmission(const SPI* descriptor) { LL_GPIO_SetOutputPin(descriptor->cs_port, descriptor->pin); }

/**
 * Receive a byte from the SPI bus
 * 
 * @param descriptor SPI descriptor to use
 * @param byte_to_transmit Byte to transmit either as a command or as a filler to keep the SPI clock running
 * @param tx_start_tick Tick at which the transmission started
 * @return uint8_t Received byte
 */
uint8_t receiveSPIbyte(SPI* descriptor, uint8_t byte_to_transmit, uint32_t tx_start_tick) {
    //send the byte to transmit and wait for the reception to be complete
    LL_SPI_TransmitData8(descriptor->handle, descriptor->read_mask | byte_to_transmit);
    while ((!LL_SPI_IsActiveFlag_RXNE(descriptor->handle)) && !timeout(tx_start_tick, kSPItimeoutMS)) {
    };

    //read the received byte (clears the rx buffer) and return it
    return LL_SPI_ReceiveData8(descriptor->handle);
}

/**
 * Send a byte via SPI
 *
 * @param descriptor        SPI descriptor to use
 * @param byte_to_transmit  Byte to transmit
 * @param tx_start_tick     Tick to compare to know if a timeout occurred
 */
static void sendSPIbyte(SPI* descriptor, uint8_t byte_to_transmit, uint32_t tx_start_tick) {
    LL_SPI_TransmitData8(descriptor->handle, byte_to_transmit);
    while (!LL_SPI_IsActiveFlag_TXE(descriptor->handle) && !timeout(tx_start_tick, kSPItimeoutMS)) {
    };
}

/**
 * Burst read registers via SPI
 *
 * @param descriptor SPI descriptor to use
 * @param first_register Number of the first register to read
 * @param[out] value Registers value array
 * @param size Number of registers to read
 * @return   Success
 * @retval 1 SPI handle or value buffer NULL
 * @retval 2 Timeout
 */
ErrorCode readRegisters(SPI* descriptor, SPIregister first_register, SPIregister value[], size_t size) {
    const uint8_t spi_rx_filler = 0xFFU;  ///< Value to send as a filler while receiving multiple bytes

    //if no bytes to read, success
    if (!size) {
        return kSuccessCode;
    }

    //make sure neither the handle nor the buffer are NULL
    if (!descriptor->handle || !value) {
        return (createErrorCode(kSPIreadRegisters, 1, kErrorCritical));
    }

    //set timeout timer and enable CS
    uint32_t spi_start_tick = HAL_GetTick();
    LL_GPIO_ResetOutputPin(descriptor->cs_port, descriptor->pin);

    //send the read request and a dummy byte to synchronize the SPI clock
    (void)receiveSPIbyte(descriptor, first_register, spi_start_tick);
    (void)receiveSPIbyte(descriptor, spi_rx_filler, spi_start_tick);

    //receive the bytes to read
    do {
        *value = receiveSPIbyte(descriptor, spi_rx_filler, spi_start_tick);
        value++;
        size--;
    } while (size && !timeout(spi_start_tick, kSPItimeoutMS));

    //wait for transaction to be finished and clear Overrun flag
    while (LL_SPI_IsActiveFlag_BSY(descriptor->handle) && !timeout(spi_start_tick, kSPItimeoutMS)) {
    };
    LL_SPI_ClearFlag_OVR(descriptor->handle);

    //disable CS
    LL_GPIO_SetOutputPin(descriptor->cs_port, descriptor->pin);

    //if timeout, error
    if (timeout(spi_start_tick, kSPItimeoutMS)) {
        return (createErrorCode(kSPIreadRegisters, 2, kErrorWarning));
    }

    return (kSuccessCode);
}

/**
 * Burst write registers on the BMI270 and do not terminate the transmission.
 * @details This function is blocking
 *
 * @param descriptor SPI descriptor to use
 * @param register_number Register to which write the values
 * @param[in] parameters Parameters to send
 * @param size Number of parameter bytes to write
 * @return	 Success
 * @retval 1 Invalid parameters provided
 * @retval 2 Timeout
 */
ErrorCode writeRegistersAndContinue(SPI* descriptor, SPIregister register_number, const SPIregister parameters[],
                                    size_t size) {
    //if invalid parameters, error
    if ((!descriptor->handle) || (!parameters && size) || (register_number > descriptor->highest_register_number)) {
        return (createErrorCode(kSPIwriteRegistersContinue, 1, kErrorWarning));
    }

    //set timeout timer and enable CS
    uint32_t spi_start_tick = HAL_GetTick();
    setDataCommandGPIO(descriptor, kCommand);
    LL_GPIO_ResetOutputPin(descriptor->cs_port, descriptor->pin);

    //send the write instruction
    sendSPIbyte(descriptor, descriptor->write_mask | register_number, spi_start_tick);

    //write the value data
    setDataCommandGPIO(descriptor, kData);
    while (size && !timeout(spi_start_tick, kSPItimeoutMS)) {
        sendSPIbyte(descriptor, *parameters, spi_start_tick);
        parameters++;
        size--;
    }

    //wait for transaction to be finished and clear Overrun flag
    while (LL_SPI_IsActiveFlag_BSY(descriptor->handle) && !timeout(spi_start_tick, kSPItimeoutMS)) {
    };
    LL_SPI_ClearFlag_OVR(descriptor->handle);

    //if timeout, error
    if (timeout(spi_start_tick, kSPItimeoutMS)) {
        return (createErrorCode(kSPIwriteRegistersContinue, 2, kErrorWarning));
    }

    return (kSuccessCode);
}

/**
 * Burst write registers on the BMI270 and terminate the transmission.
 * @details This function is blocking
 *
 * @param descriptor SPI descriptor to use
 * @param register_number Register to which write the values
 * @param[in] parameters Parameters to send
 * @param size Number of parameter bytes to write
 * @return Value returned by writeRegistersAndContinue()
 */
ErrorCode writeRegisters(SPI* descriptor, SPIregister register_number, const SPIregister parameters[], size_t size) {
    ErrorCode result = writeRegistersAndContinue(descriptor, register_number, parameters, size);
    closeTransmission(descriptor);
    return result;
}

/**
 * Send data to SPI via DMA
 *
 * @param dma       DMA descriptor to use
 * @param buffer    Data to send
 * @param nb_bytes   Number of bytes to send
 * @param max_bytes  Maximum number of bytes which can be send (e.g. buffer size)
 * @retval 0 Success
 * @retval 1 Number of bytes exceeds the maximum
 * @retval 2 Timeout while waiting for the DMA transmit complete interrupt
 * @retval 3 DMA error
 */
ErrorCode sendDMA(DMA* dma, uint8_t* buffer, size_t nb_bytes, size_t max_bytes) {
    ErrorCode result;

    if (nb_bytes > max_bytes) {
        return createErrorCode(kSPIdmaSend, 1, kErrorCritical);
    }

    if (!nb_bytes) {
        return kSuccessCode;
    }

    latest_task = dma->task;

    //configure the DMA transaction
    LL_DMA_DisableChannel(dma->dma, dma->channel);
    LL_DMA_ClearFlag_GI5(dma->dma);
    LL_DMA_EnableIT_TC(dma->dma, dma->channel);
    LL_DMA_ConfigAddresses(dma->dma, dma->channel, (uint32_t)buffer, LL_SPI_DMA_GetRegAddr(dma->spi.handle),
                           LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetDataLength(dma->dma, dma->channel, nb_bytes);  //must be reset every time
    LL_DMA_EnableChannel(dma->dma, dma->channel);
    LL_SPI_EnableDMAReq_TX(dma->spi.handle);

    result = kSuccessCode;

    //wait for measurements to be ready
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kSPItimeoutMS)) == pdFALSE) {
        result = createErrorCode(kSPIdmaSend, 2, kErrorError);
        goto finaliseDMA;
    }

    if (LL_DMA_IsActiveFlag_TE5(dma->dma)) {
        result = createErrorCode(kSPIdmaSend, 3, kErrorError);
        goto finaliseDMA;
    }

finaliseDMA:
    LL_DMA_DisableChannel(dma->dma, dma->channel);
    closeTransmission(&dma->spi);
    return result;
}
