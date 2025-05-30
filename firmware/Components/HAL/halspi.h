#ifndef HAL_SPI_H
#define HAL_SPI_H
#include <stddef.h>
#include "FreeRTOS.h"
#include "errorstack.h"
#include "stm32f103xb.h"
#include "task.h"

enum {
    SPI_TIMEOUT_MS      = 100U,  ///< Number of milliseconds beyond which SPI is in timeout
    SPISTRUCT_ALIGNMENT = 32U,   ///< Optimised memory alignment for the SPI structure
    DMASTRUCT_ALIGNMENT = 64U,   ///< Optimised memory alignment for the DMA structure
    SPI_READREGISTERS   = 1U,    ///< Function ID for the readRegisters()
    SPI_WRITEREGISTERS  = 2U,    ///< Function ID for the writeRegisters()
};

typedef uint8_t spiregister_t;

typedef struct {
    SPI_TypeDef*  handle;                 ///< SPI handle defined in HAL library
    GPIO_TypeDef* CSport;                 ///< GPIO port used by SPI CS pin
    uint32_t      pin;                    ///< GPIO mask used by SPI CS pin
    spiregister_t readMask;               ///< Bit mask used to code in a read request
    spiregister_t writeMask;              ///< Bit mask used to code in a write request
    spiregister_t highestRegisterNumber;  ///< Highest register number a request can be
    GPIO_TypeDef* dataCommandPort;
    uint32_t      dataCommandPin;
} __attribute__((aligned(SPISTRUCT_ALIGNMENT))) spi_t;

typedef struct {
    TaskHandle_t task;
    spi_t        spi;
    DMA_TypeDef* dma;
    uint32_t     Channel;
} __attribute__((aligned(DMASTRUCT_ALIGNMENT))) dma_t;

uint8_t     receiveSPIbyte(spi_t* descriptor, uint8_t byteToTransmit, uint32_t txStartTick);
errorCode_u readRegisters(spi_t* descriptor, spiregister_t firstRegister, spiregister_t value[], size_t size);
errorCode_u writeRegisters(spi_t* descriptor, spiregister_t registerNumber, const spiregister_t parameters[],
                           size_t size);
errorCode_u writeRegistersAndContinue(spi_t* descriptor, spiregister_t registerNumber, const spiregister_t parameters[],
                                      size_t size);
void        closeTransmission(const spi_t* descriptor);
errorCode_u sendDMA(dma_t* dma, uint8_t* buffer, size_t nbBytes, size_t maxBytes);
void        st7735sDMAinterruptHandler(void);
#endif
