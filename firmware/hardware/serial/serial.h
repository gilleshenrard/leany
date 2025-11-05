/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef HARDWARE_SERIAL_SERIAL_H
#define HARDWARE_SERIAL_SERIAL_H
#include <stdint.h>

#include "errorstack.h"
#include "generic_command.inc"

enum {
    kFloatBufferSize = 20U,  ///< Number of bytes a float to string buffer can hold
};

void uartInterruptTriggered(void);
void createSerialtask(void);
void logSerial(ErrorLevel level, const char format[], ...);
uint8_t popSerialCommand(GenericCommand* command_received);
void floatToString(float value, char out_buffer[], uint8_t buffer_size, uint8_t precision);
void intToString(uint32_t value, char out_buffer[], uint8_t buffer_size);
void setLogLevel(ErrorLevel level);
ErrorLevel getLogLevel(void);

#endif
