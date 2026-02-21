/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file hal_i2c.h
 * @author Gilles Henrard
 */
#ifndef HARDWARE_HAL_HAL_I2C_H
#define HARDWARE_HAL_HAL_I2C_H

#include <stdint.h>
#include <stm32f103xb.h>

#include "errorstack.h"

ErrorCode readI2Cregisters(I2C_TypeDef* descriptor, uint8_t slave_address, uint8_t first_register, uint8_t data[],
                           uint8_t nb_bytes);
ErrorCode writeI2CRegisters(I2C_TypeDef* descriptor, uint8_t slave_address, uint8_t first_register,
                            const uint8_t data[], uint8_t nb_bytes);

#endif
