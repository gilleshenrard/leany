/*
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef HARDWARE_BATTERY_BQ25619_H
#define HARDWARE_BATTERY_BQ25619_H

#include <main.h>

#include "bq25619_registers.inc"
#include "errorstack.h"

ErrorCode readI2Cregisters(I2C_TypeDef* descriptor, BQ25619register first_register, uint8_t data[], uint8_t nb_bytes);
ErrorCode writeI2CRegisters(I2C_TypeDef* descriptor, BQ25619register first_register, const uint8_t data[],
                            uint8_t nb_bytes);

#endif
