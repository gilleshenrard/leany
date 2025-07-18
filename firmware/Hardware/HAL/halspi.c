/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file halspi.c
 * @brief Implement a generic HAL for SPI communication
 * @author Gilles Henrard
 * @date 13/06/2025
 */
#include "halspi.h"
#include <stddef.h>
#include <stdint.h>
#include "errorstack.h"
#include "main.h"
#include "portmacro.h"
#include "projdefs.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_spi.h"

/**
 * @brief SPI Data/command pin status enumeration
 */
typedef enum {
    COMMAND = 0,  ///< Command is to be sent
    DATA,         ///< Data is to be sent
} DCgpio_e;

/**
 * Enumeration of the function IDs
 */
typedef enum {
    SPI_READREGISTERS      = 1U,  ///< Function ID for the readRegisters()
    SPI_WRITEREGISTERSCONT = 2U,  ///< Function ID for the writeRegistersAndContinue()
    SPI_DMASEND            = 3U,  ///< Function ID for the sendDMA()
} function_e;

//state variables

static inline void           setDataCommandGPIO(const spi_t* descriptor, DCgpio_e function);
static volatile TaskHandle_t latestTask = NULL;  ///< Handle used by the FreeRTOS task

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Handler called upon DMA transfer complete
 */
void st7735sDMAinterruptHandler(void) {
    BaseType_t hasWoken = 0;
    vTaskNotifyGiveFromISR(latestTask, &hasWoken);
    portYIELD_FROM_ISR(hasWoken);
}

/**
 * brief Set the Data/Command pin
 * 
 * @param descriptor SPI descriptor to use
 * @param function Value of the data/command pin
 */
static inline void setDataCommandGPIO(const spi_t* descriptor, DCgpio_e function) {
    if(!descriptor->dataCommandPort) {
        return;
    }

    if(function == COMMAND) {
        LL_GPIO_ResetOutputPin(descriptor->dataCommandPort, descriptor->dataCommandPin);
    } else {
        LL_GPIO_SetOutputPin(descriptor->dataCommandPort, descriptor->dataCommandPin);
    }
}

/**
 * Terminate a SPI transmission
 *
 * @param descriptor SPI descriptor to use
 */
void closeTransmission(const spi_t* descriptor) {
    LL_GPIO_SetOutputPin(descriptor->CSport, descriptor->pin);
}

/**
 * Receive a byte from the SPI bus
 * 
 * @param descriptor SPI descriptor to use
 * @param byteToTransmit Byte to transmit either as a command or as a filler to keep the SPI clock running
 * @param txStartTick Tick at which the transmission started
 * @return uint8_t Received byte
 */
uint8_t receiveSPIbyte(spi_t* descriptor, uint8_t byteToTransmit, uint32_t txStartTick) {
    //send the byte to transmit and wait for the reception to be complete
    LL_SPI_TransmitData8(descriptor->handle, descriptor->readMask | byteToTransmit);
    while((!LL_SPI_IsActiveFlag_RXNE(descriptor->handle)) && !timeout(txStartTick, SPI_TIMEOUT_MS)) {};

    //read the received byte (clears the rx buffer) and return it
    return LL_SPI_ReceiveData8(descriptor->handle);
}

/**
 * Burst read registers via SPI
 *
 * @param descriptor SPI descriptor to use
 * @param firstRegister Number of the first register to read
 * @param[out] value Registers value array
 * @param size Number of registers to read
 * @return   Success
 * @retval 1 SPI handle or value buffer NULL
 * @retval 2 Timeout
 */
errorCode_u readRegisters(spi_t* descriptor, spiregister_t firstRegister, spiregister_t value[], size_t size) {
    const uint8_t SPI_RX_FILLER = 0xFFU;  ///< Value to send as a filler while receiving multiple bytes

    //if no bytes to read, success
    if(!size) {
        return ERR_SUCCESS;
    }

    //make sure neither the handle nor the buffer are NULL
    if(!descriptor->handle || !value) {
        return (createErrorCode(SPI_READREGISTERS, 1, ERR_CRITICAL));
    }

    //set timeout timer and enable CS
    uint32_t SPIstartTick = HAL_GetTick();
    LL_GPIO_ResetOutputPin(descriptor->CSport, descriptor->pin);

    //send the read request and a dummy byte to synchronize the SPI clock
    (void)receiveSPIbyte(descriptor, firstRegister, SPIstartTick);
    (void)receiveSPIbyte(descriptor, SPI_RX_FILLER, SPIstartTick);

    //receive the bytes to read
    do {
        *value = receiveSPIbyte(descriptor, SPI_RX_FILLER, SPIstartTick);
        value++;
        size--;
    } while(size && !timeout(SPIstartTick, SPI_TIMEOUT_MS));

    //wait for transaction to be finished and clear Overrun flag
    while(LL_SPI_IsActiveFlag_BSY(descriptor->handle) && !timeout(SPIstartTick, SPI_TIMEOUT_MS)) {};
    LL_SPI_ClearFlag_OVR(descriptor->handle);

    //disable CS
    LL_GPIO_SetOutputPin(descriptor->CSport, descriptor->pin);

    //if timeout, error
    if(timeout(SPIstartTick, SPI_TIMEOUT_MS)) {
        return (createErrorCode(SPI_READREGISTERS, 2, ERR_WARNING));
    }

    return (ERR_SUCCESS);
}

/**
 * Burst write registers on the BMI270 and do not terminate the transmission.
 * @details This function is blocking
 *
 * @param descriptor SPI descriptor to use
 * @param registerNumber Register to which write the values
 * @param[in] parameters Parameters to send
 * @param size Number of parameter bytes to write
 * @return	 Success
 * @retval 1 No SPI handle specified
 * @retval 2 No parameters given or no size despite parameters
 * @retval 3 Register number out of range
 * @retval 4 Timeout
 */
errorCode_u writeRegistersAndContinue(spi_t* descriptor, spiregister_t registerNumber, const spiregister_t parameters[],
                                      size_t size) {
    //if handle not specified, error
    if(!descriptor->handle) {
        return (createErrorCode(SPI_WRITEREGISTERSCONT, 1, ERR_WARNING));
    }

    //if no parameters provided and size not zero, error
    if(!parameters && size) {
        return (createErrorCode(SPI_WRITEREGISTERSCONT, 2, ERR_WARNING));
    }

    //if register number above known or within the reserved range, error
    if(registerNumber > descriptor->highestRegisterNumber) {
        return (createErrorCode(SPI_WRITEREGISTERSCONT, 3, ERR_WARNING));
    }

    //set timeout timer and enable CS
    uint32_t SPIstartTick = HAL_GetTick();
    setDataCommandGPIO(descriptor, COMMAND);
    LL_GPIO_ResetOutputPin(descriptor->CSport, descriptor->pin);

    //send the write instruction
    LL_SPI_TransmitData8(descriptor->handle,
                         descriptor->writeMask | registerNumber);  // cppcheck-suppress badBitmaskCheck
    while(!LL_SPI_IsActiveFlag_TXE(descriptor->handle) && !timeout(SPIstartTick, SPI_TIMEOUT_MS)) {};

    //write the value data
    setDataCommandGPIO(descriptor, DATA);
    while(size && !timeout(SPIstartTick, SPI_TIMEOUT_MS)) {
        while(!LL_SPI_IsActiveFlag_TXE(descriptor->handle) && !timeout(SPIstartTick, SPI_TIMEOUT_MS)) {};
        if(LL_SPI_IsActiveFlag_TXE(descriptor->handle)) {
            LL_SPI_TransmitData8(descriptor->handle, *parameters);
        }
        parameters++;
        size--;
    }

    //wait for transaction to be finished and clear Overrun flag
    while(LL_SPI_IsActiveFlag_BSY(descriptor->handle) && !timeout(SPIstartTick, SPI_TIMEOUT_MS)) {};
    LL_SPI_ClearFlag_OVR(descriptor->handle);

    //if timeout, error
    if(timeout(SPIstartTick, SPI_TIMEOUT_MS)) {
        return (createErrorCode(SPI_WRITEREGISTERSCONT, 4, ERR_WARNING));
    }

    return (ERR_SUCCESS);
}

/**
 * Burst write registers on the BMI270 and terminate the transmission.
 * @details This function is blocking
 *
 * @param descriptor SPI descriptor to use
 * @param registerNumber Register to which write the values
 * @param[in] parameters Parameters to send
 * @param size Number of parameter bytes to write
 * @return Value returned by writeRegistersAndContinue()
 */
errorCode_u writeRegisters(spi_t* descriptor, spiregister_t registerNumber, const spiregister_t parameters[],
                           size_t size) {
    errorCode_u result = writeRegistersAndContinue(descriptor, registerNumber, parameters, size);
    closeTransmission(descriptor);
    return result;
}

/**
 * Send data to SPI via DMA
 *
 * @param dma       DMA descriptor to use
 * @param buffer    Data to send
 * @param nbBytes   Number of bytes to send
 * @param maxBytes  Maximum number of bytes which can be send (e.g. buffer size)
 * @retval 0 Success
 * @retval 1 Number of bytes exceeds the maximum
 * @retval 2 Timeout while waiting for the DMA transmit complete interrupt
 * @retval 3 DMA error
 */
errorCode_u sendDMA(dma_t* dma, uint8_t* buffer, size_t nbBytes, size_t maxBytes) {
    errorCode_u result;

    if(nbBytes > maxBytes) {
        return createErrorCode(SPI_DMASEND, 1, ERR_CRITICAL);
    }

    if(!nbBytes) {
        return ERR_SUCCESS;
    }

    latestTask = dma->task;

    //configure the DMA transaction
    LL_DMA_DisableChannel(dma->dma, dma->Channel);
    LL_DMA_ClearFlag_GI5(dma->dma);
    LL_DMA_EnableIT_TC(dma->dma, dma->Channel);
    LL_DMA_ConfigAddresses(dma->dma, dma->Channel, (uint32_t)buffer, LL_SPI_DMA_GetRegAddr(dma->spi.handle),
                           LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetDataLength(dma->dma, dma->Channel, nbBytes);  //must be reset every time
    LL_DMA_EnableChannel(dma->dma, dma->Channel);
    LL_SPI_EnableDMAReq_TX(dma->spi.handle);

    result = ERR_SUCCESS;

    //wait for measurements to be ready
    if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SPI_TIMEOUT_MS)) == pdFALSE) {
        result = createErrorCode(SPI_DMASEND, 2, ERR_ERROR);
        goto finaliseDMA;
    }

    if(LL_DMA_IsActiveFlag_TE5(dma->dma)) {
        result = createErrorCode(SPI_DMASEND, 3, ERR_ERROR);
        goto finaliseDMA;
    }

finaliseDMA:
    LL_DMA_DisableChannel(dma->dma, dma->Channel);
    closeTransmission(&dma->spi);
    return result;
}
