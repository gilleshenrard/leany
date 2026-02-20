/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file task_dispatcher.h
 * @author Gilles Henrard
 */
#ifndef CONTROLLER_DISPATCHER_H
#define CONTROLLER_DISPATCHER_H
#include "errorstack.h"

ErrorCode createMessageDispatchertask(void);
uint8_t setLastErrorCode(ErrorCode error);
uint8_t getLastErrorCode(ErrorCode* error);

#endif
