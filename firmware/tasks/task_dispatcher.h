/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef CONTROLLER_DISPATCHER_H
#define CONTROLLER_DISPATCHER_H
#include "errorstack.h"

ErrorCode createMessageDispatchertask(void);
uint8_t setLastErrorCode(ErrorCode error);
uint8_t getLastErrorCode(ErrorCode* error);

#endif
