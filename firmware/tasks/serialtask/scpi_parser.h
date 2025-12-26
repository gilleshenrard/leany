/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef HARDWARE_SERIAL_SCPI_PARSER_H
#define HARDWARE_SERIAL_SCPI_PARSER_H
#include <stdint.h>

#include "generic_command.inc"

void resetSCPIparser(void);
uint8_t pushSCPIcharacter(uint8_t new_char, GenericCommand* command_received);

#endif
