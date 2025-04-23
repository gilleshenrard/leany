#ifndef HAL_SPI_H
#define HAL_SPI_H
#include <stddef.h>
#include "errorstack.h"
#include "stm32f103xb.h"

enum {
    SPI_TIMEOUT_MS     = 100U,  ///< Number of milliseconds beyond which SPI is in timeout
    STRUCT_ALIGNMENT   = 16U,   ///< Optimised bytes alignment for the SPI structure
    SPI_READREGISTERS  = 1U,    ///< Function ID for the readRegisters()
    SPI_WRITEREGISTERS = 2U,    ///< Function ID for the writeRegisters()
};

typedef uint8_t spiregister_t;

typedef struct {
    SPI_TypeDef*  handle;                 ///< SPI handle defined in HAL library
    GPIO_TypeDef* CSport;                 ///< GPIO port used by SPI CS pin
    uint32_t      pin;                    ///< GPIO mask used by SPI CS pin
    spiregister_t readMask;               ///< Bit mask used to code in a read request
    spiregister_t writeMask;              ///< Bit mask used to code in a write request
    spiregister_t highestRegisterNumber;  ///< Highest register number a request can be
} __attribute__((aligned(STRUCT_ALIGNMENT))) spi_t;

uint8_t     receiveSPIbyte(spi_t* descriptor, uint8_t byteToTransmit, uint32_t txStartTick);
errorCode_u readRegisters(spi_t* descriptor, spiregister_t firstRegister, spiregister_t value[], size_t size);
errorCode_u writeRegisters(spi_t* descriptor, spiregister_t registerNumber, const spiregister_t value[], size_t size);
#endif
