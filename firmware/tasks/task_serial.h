/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file task_serial.h
 * @author Gilles Henrard
 */
#ifndef TASKS_TASK_SERIAL_H
#define TASKS_TASK_SERIAL_H
#include "errorstack.h"
#include "serial_command_types.h"

void uartInterruptTriggered(void);
void createSerialtask(void);
void logSerial(ErrorLevel level, const char format[], ...);
bool popSerialCommand(SerialCommand* command_received);
void setLogLevel(ErrorLevel level);
ErrorLevel getLogLevel(void);

#endif
