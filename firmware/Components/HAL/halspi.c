#include "halspi.h"
#include <stddef.h>
#include <stdint.h>
#include "errorstack.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_spi.h"

/**
 * Receive a byte from the SPI bus
 * 
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
 * Burst read registers on the BMI270
 *
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
    LL_GPIO_ResetOutputPin(BMI270_CS_GPIO_Port, BMI270_CS_Pin);

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
    LL_GPIO_SetOutputPin(BMI270_CS_GPIO_Port, BMI270_CS_Pin);

    //if timeout, error
    if(timeout(SPIstartTick, SPI_TIMEOUT_MS)) {
        return (createErrorCode(SPI_READREGISTERS, 2, ERR_WARNING));
    }

    return (ERR_SUCCESS);
}

/**
 * Burst write a single register on the BMI270
 *
 * @param registerNumber Register number
 * @param[in] value Value to assign to the register
 * @param size Number of bytes (i.e. register values or data bytes to a single register) to write
 * @return	 Success
 * @retval 1 No SPI handle specified
 * @retval 2 Register number out of range
 * @retval 3 Timeout
 */
errorCode_u writeRegisters(spi_t* descriptor, spiregister_t registerNumber, const spiregister_t value[], size_t size) {
    //if no bytes to write, success
    if(!size) {
        return ERR_SUCCESS;
    }

    //if handle not specified, error
    if(!descriptor->handle) {
        return (createErrorCode(SPI_WRITEREGISTERS, 1, ERR_WARNING));
    }

    //if register number above known or within the reserved range, error
    if(registerNumber > descriptor->highestRegisterNumber) {
        return (createErrorCode(SPI_WRITEREGISTERS, 2, ERR_WARNING));
    }

    //set timeout timer and enable CS
    uint32_t SPIstartTick = HAL_GetTick();
    LL_GPIO_ResetOutputPin(BMI270_CS_GPIO_Port, BMI270_CS_Pin);

    //send the write instruction
    LL_SPI_TransmitData8(descriptor->handle,
                         descriptor->writeMask | registerNumber);  // cppcheck-suppress badBitmaskCheck

    //write the value data
    do {
        while(!LL_SPI_IsActiveFlag_TXE(descriptor->handle) && !timeout(SPIstartTick, SPI_TIMEOUT_MS)) {};
        LL_SPI_TransmitData8(descriptor->handle, *value);
        value++;
        size--;
    } while(size && !timeout(SPIstartTick, SPI_TIMEOUT_MS));

    //wait for transaction to be finished and clear Overrun flag
    while(LL_SPI_IsActiveFlag_BSY(descriptor->handle) && !timeout(SPIstartTick, SPI_TIMEOUT_MS)) {};
    LL_SPI_ClearFlag_OVR(descriptor->handle);

    //disable CS
    LL_GPIO_SetOutputPin(BMI270_CS_GPIO_Port, BMI270_CS_Pin);

    //if timeout, error
    if(timeout(SPIstartTick, SPI_TIMEOUT_MS)) {
        return (createErrorCode(SPI_WRITEREGISTERS, 3, ERR_WARNING));
    }

    return (ERR_SUCCESS);
}
