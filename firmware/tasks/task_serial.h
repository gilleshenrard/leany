/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file task_serial.h
 * @author Gilles Henrard
 */
#ifndef HARDWARE_SERIAL_SERIAL_H
#define HARDWARE_SERIAL_SERIAL_H
#include <stdint.h>

#include "errorstack.h"
#include "serial_command_types.h"

void uartInterruptTriggered(void);
void createSerialtask(void);
void logSerial(ErrorLevel level, const char format[], ...);
uint8_t popSerialCommand(SerialCommand* command_received);
void setLogLevel(ErrorLevel level);
ErrorLevel getLogLevel(void);

#endif
