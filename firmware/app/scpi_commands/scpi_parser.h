/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file scpi_parser.h
 * @author Gilles Henrard
 */
#ifndef APP_SCPI_COMMANDS_SCPI_PARSER_H
#define APP_SCPI_COMMANDS_SCPI_PARSER_H
#include <stdint.h>

#include "serial_command_types.h"

void resetSCPIparser(void);
uint8_t pushSCPIcharacter(uint8_t new_char, SerialCommand* command_received);

#endif
