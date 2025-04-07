#ifndef HAL_SPI_H
#define HAL_SPI_H
#include <stddef.h>
#include "errorstack.h"
#include "stm32f103xb.h"

enum {
    SPI_TIMEOUT_MS     = 100U,  ///< Number of milliseconds beyond which SPI is in timeout
    STRUCT_ALIGNMENT   = 16U,
    SPI_READREGISTERS  = 1U,
    SPI_WRITEREGISTERS = 2U,
};

typedef uint8_t spiregister_t;

typedef struct {
    SPI_TypeDef*  handle;
    GPIO_TypeDef* CSport;
    uint32_t      pin;
    spiregister_t readMask;
    spiregister_t writeMask;
    spiregister_t highestRegisterNumber;
} __attribute__((aligned(STRUCT_ALIGNMENT))) spi_t;

uint8_t     receiveSPIbyte(spi_t* descriptor, uint8_t byteToTransmit, uint32_t txStartTick);
errorCode_u readRegisters(spi_t* descriptor, spiregister_t firstRegister, spiregister_t value[], size_t size);
errorCode_u writeRegisters(spi_t* descriptor, spiregister_t registerNumber, const spiregister_t value[], size_t size);
#endif
