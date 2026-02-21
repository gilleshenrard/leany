/*
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include "hal_i2c.h"

#include <stdint.h>
#include <stm32f103xb.h>
#include <stm32f1xx_hal.h>
#include <stm32f1xx_ll_i2c.h>

#include "errorstack.h"

enum {
    kI2Ctimeout_ms = 10U,  ///< Maximum number of milliseconds an I²C transfer can last
    kREAD = 0x01U,         ///< Bit value to apply to read data from the charger
    kWRITE = 0x00U,        ///< Bit value to apply to write data to the charger
};

/**
 * @brief Enumeration of all the function ID used in errors
 */
typedef enum {
    kReadRegisters = 1,      ///< readRegisters() function
    kInitiateReception = 2,  ///< initiateReception() function
    kWriteRegisters = 3,     ///< writeRegisters() function
} FunctionCode;

static ErrorCode initiateTransaction(I2C_TypeDef* descriptor, uint8_t slave_address, uint8_t first_register,
                                     uint32_t start_tick);

static ErrorCode result = {0};  ///< Buffer used to store the latest error code

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Burst read data via I²C
 *
 * @param descriptor SPI descriptor to use
 * @param slave_address I²C slave address
 * @param first_register First register to read
 * @param[out] data Data buffer to fill
 * @param nb_bytes Number of bytes to read
 * @retval 0 Success
 * @retval 1 Invalid descriptor or data buffer provided
 * @retval 2 Error while initiating the read sequence
 * @retval 3 Timeout while waiting for the data byte
 * @retval 4 Timeout while waiting for slave address to be sent
 * @retval 5 Timeout while waiting for a data byte to be received
 * @retval 6 Timeout while waiting for the last data byte to be received
 */
ErrorCode readI2Cregisters(I2C_TypeDef* descriptor, uint8_t slave_address, uint8_t first_register, uint8_t data[],
                           uint8_t nb_bytes) {
    (void)nb_bytes;

    if (!descriptor || (!data && nb_bytes)) {
        return createErrorCode(kReadRegisters, 1, kErrorError);
    }

    if (!nb_bytes) {
        return kSuccessCode;
    }

    uint8_t timeout_value = 0;
    uint32_t start_tick = HAL_GetTick();

    //initiate the I²C reception
    result = initiateTransaction(descriptor, slave_address, first_register, start_tick);
    EXIT_ON_ERROR(result, kReadRegisters, 2)

    //resend the slave address in read mode
    LL_I2C_GenerateStartCondition(descriptor);
    EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_SB(descriptor), kI2Ctimeout_ms, kInitiateReception, 3)
    LL_I2C_TransmitData8(descriptor, (uint8_t)(slave_address << 1U) | (uint8_t)kREAD);
    EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_ADDR(descriptor), kI2Ctimeout_ms, kInitiateReception, 4)
    LL_I2C_ClearFlag_ADDR(descriptor);

    //read all bytes except for the last one
    uint8_t current_byte = 0;
    LL_I2C_AcknowledgeNextData(descriptor, LL_I2C_ACK);
    while (nb_bytes - 1U) {
        EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_RXNE(descriptor), kI2Ctimeout_ms, kReadRegisters, 5)
        data[current_byte] = LL_I2C_ReceiveData8(descriptor);
        current_byte++;
        nb_bytes--;
    }

    //read the last byte, with a NACK and a STOP signal
    LL_I2C_AcknowledgeNextData(descriptor, LL_I2C_NACK);
    LL_I2C_GenerateStopCondition(descriptor);
    EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_RXNE(descriptor), kI2Ctimeout_ms, kReadRegisters, 6)
    data[current_byte] = LL_I2C_ReceiveData8(descriptor);
    return kSuccessCode;
}

/**
 * Burst-write data to registers
 *
 * @param descriptor I²C descriptor to use
 * @param slave_address I²C slave address
 * @param first_register Number of the first register to which write data
 * @param data Data to write
 * @param nb_bytes Number of bytes of data to write
 * @retval 0 Success
 * @retval 1 Invalid descriptor or data buffer provided
 * @retval 2 Error while initiating the write sequence
 * @retval 3 Timeout while waiting for TXE to be set
 * @retval 4 Timeout while waiting for TXE to be set before the last data byte
 * @retval 5 Timeout while waiting for Byte Transfer Finish to be set
 */
ErrorCode writeI2CRegisters(I2C_TypeDef* descriptor, uint8_t slave_address, uint8_t first_register,
                            const uint8_t data[], uint8_t nb_bytes) {
    if (!descriptor || (!data && nb_bytes)) {
        return createErrorCode(kReadRegisters, 1, kErrorError);
    }

    if (!nb_bytes) {
        return kSuccessCode;
    }

    uint8_t timeout_value = 0;
    uint32_t start_tick = HAL_GetTick();

    //initiate the I²C transmission
    result = initiateTransaction(descriptor, slave_address, first_register, start_tick);
    EXIT_ON_ERROR(result, kReadRegisters, 2)

    //send the data bytes all until the one before last
    uint8_t current_byte = 0;
    while (nb_bytes - 1U) {
        EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_TXE(descriptor), kI2Ctimeout_ms, kWriteRegisters, 3)
        LL_I2C_TransmitData8(descriptor, data[current_byte]);
        current_byte++;
        nb_bytes--;
    }

    //send the last byte and stop the transmission
    EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_TXE(descriptor), kI2Ctimeout_ms, kWriteRegisters, 4)
    LL_I2C_TransmitData8(descriptor, data[current_byte]);
    EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_BTF(descriptor), kI2Ctimeout_ms, kWriteRegisters, 5)
    LL_I2C_GenerateStopCondition(descriptor);

    return kSuccessCode;
}

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Initiate an I²C reception sequence
 *
 * @param descriptor I²C descriptor to use
 * @param slave_address I²C slave address
 * @param first_register Number of the first register to read
 * @param start_tick FreeRTOS tick at the start of the sequence
 * @retval 0 Success
 * @retval 1 Timeout while waiting for start signal to be done
 * @retval 2 Timeout while waiting for ADDR byte to be sent
 * @retval 3 Timeout while waiting for the register address to be sent
 */
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
static ErrorCode initiateTransaction(I2C_TypeDef* descriptor, uint8_t slave_address, uint8_t first_register,
                                     uint32_t start_tick) {
    uint8_t timeout_value = 0;

    //send a start signal
    LL_I2C_GenerateStartCondition(descriptor);
    EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_SB(descriptor), kI2Ctimeout_ms, kInitiateReception, 1)

    //send the slave address
    LL_I2C_TransmitData8(descriptor, (uint8_t)(slave_address << 1U) | (uint8_t)kWRITE);
    EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_ADDR(descriptor), kI2Ctimeout_ms, kInitiateReception, 2)
    LL_I2C_AcknowledgeNextData(descriptor, LL_I2C_NACK);
    LL_I2C_ClearFlag_ADDR(descriptor);

    //send the register address
    EXIT_ON_TIMEOUT(LL_I2C_IsActiveFlag_TXE(descriptor), kI2Ctimeout_ms, kInitiateReception, 3)
    LL_I2C_TransmitData8(descriptor, first_register);

    return kSuccessCode;
}
