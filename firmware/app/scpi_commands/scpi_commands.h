/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file scpi_commands.h
 * @author Gilles Henrard
 */
#ifndef APP_SCPI_COMMANDS_SCPI_COMMANDS_H
#define APP_SCPI_COMMANDS_SCPI_COMMANDS_H
#include <stdint.h>

#include "serial_command_types.h"

enum {
    kSCPImaxCommandSize = 25U,  ///< Maximum size of a SCPI command
};

/**
 * Enumeration of command modes
 */
typedef enum {
    kNA = 0,  ///< Access mode is not applicable
    kRO = 1,  ///< Read-only command
    kRW = 2,  ///< Read/write command
    kWO = 3,  ///< Write-only command
} Mode;

/**
 * Structure defining a SCPI wrapper of a command
 */
typedef struct {
    char* short_name;                ///< Short command name
    char* long_name;                 ///< Long command name
    Mode mode;                       ///< Command mode (read-only / read-write)
    SerialParameterType param_type;  ///< Type of the parameter value
    SerialCommandCode code;          ///< Command code
} SCPIcommand;

/**
 * Structure defining a command tree node
 */
typedef struct CommandNode {
    SCPIcommand scpi;                    ///< Command attached to the node
    const struct CommandNode* children;  ///< Array of children nodes
    uint8_t nb_children;                 ///< Number of children nodes
} Node;

const Node* getRootNode(void);
#endif
