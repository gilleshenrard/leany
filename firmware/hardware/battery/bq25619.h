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

ErrorCode initiateTransaction(I2C_TypeDef* descriptor, BQ25619register first_register, TickType_t start_tick);
ErrorCode readRegisters(I2C_TypeDef* descriptor, BQ25619register first_register, uint8_t data[], uint8_t nb_bytes);
ErrorCode writeRegisters(I2C_TypeDef* descriptor, BQ25619register first_register, const uint8_t data[],
                         uint8_t nb_bytes);

#endif
