/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file bq25619.h
 * @author Gilles Henrard
 */
#ifndef HARDWARE_BATTERY_BQ25619_H
#define HARDWARE_BATTERY_BQ25619_H
#include "bq25619_registers.inc"
#include "errorstack.h"

ErrorCode testBQ25619identifier(void);
ErrorCode resetBQ25619(void);
ErrorCode configureBQ25619(void);
ErrorCode updateBQ25619status(ChargerStatus* new_status, ChargerStatus* changes);

#endif
