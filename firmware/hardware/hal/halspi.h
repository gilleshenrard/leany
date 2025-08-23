/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef HAL_SPI_H
#define HAL_SPI_H
#include <FreeRTOS.h>
#include <stddef.h>
#include <stdint.h>
#include <stm32f103xb.h>
#include <task.h>

#include "errorstack.h"

enum {
    kSPItimeoutMS = 100U,       ///< Number of milliseconds beyond which SPI is in timeout
    kSPIstructAlignment = 32U,  ///< Optimised memory alignment for the SPI structure
    kDMAstructAlignment = 64U,  ///< Optimised memory alignment for the DMA structure
};

typedef uint8_t SPIregister;

typedef struct {
    SPI_TypeDef* handle;                  ///< SPI handle defined in HAL library
    GPIO_TypeDef* cs_port;                ///< GPIO port used by SPI CS pin
    uint32_t pin;                         ///< GPIO mask used by SPI CS pin
    SPIregister read_mask;                ///< Bit mask used to code in a read request
    SPIregister write_mask;               ///< Bit mask used to code in a write request
    SPIregister highest_register_number;  ///< Highest register number a request can be + 1
    GPIO_TypeDef* data_command_port;      ///< GPIO port used by the DC pin
    uint32_t data_command_pin;            ///< GPIO mask used by the DC pin
    uint8_t read_dummy;                   ///< Flag indicating whether a dummy byte read must be done
} __attribute__((aligned(kSPIstructAlignment))) SPI;

/**
 * Structure defining a SPI DMA channel descriptor
 */
typedef struct {
    SPI spi;            ///< SPI handle used
    TaskHandle_t task;  ///< Task to which the channel is linked
    DMA_TypeDef* dma;   ///< DMA handle used
    uint32_t channel;   ///< DMA channel used
} __attribute__((aligned(kDMAstructAlignment))) DMA;

uint8_t receiveSPIbyte(SPI* descriptor, uint8_t byte_to_transmit, uint32_t tx_start_tick);
ErrorCode readRegisters(SPI* descriptor, SPIregister first_register, SPIregister value[], size_t size);
ErrorCode writeRegisters(SPI* descriptor, SPIregister register_number, const SPIregister parameters[], size_t size);
ErrorCode writeRegistersAndContinue(SPI* descriptor, SPIregister register_number, const SPIregister parameters[],
                                    size_t size);
ErrorCode writeRegistersBunch(SPI* descriptor, const uint8_t registers[][2], uint8_t nb_registers);
void closeTransmission(const SPI* descriptor);
ErrorCode sendDMA(DMA* dma, uint8_t* buffer, size_t nb_bytes, size_t max_bytes);
void st7735sDMAinterruptHandler(void);

#endif
