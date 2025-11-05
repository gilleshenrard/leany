/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef HARDWARE_SERIAL_SCPI_COMMANDS_H
#define HARDWARE_SERIAL_SCPI_COMMANDS_H
#include <stdint.h>

#include "generic_command.inc"

enum {
    kSCPIcommandAlignment = 32U,  ///< Memory alignment of the Command structure
    kNodeAlignment = 64U,         ///< Memory alignment of the Node structure
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
    char* short_name;          ///< Short command name
    char* long_name;           ///< Long command name
    Mode mode;                 ///< Command mode (read-only / read-write)
    ParameterType param_type;  ///< Type of the parameter value
    CommandCode code;          ///< Command code
} __attribute((aligned(kSCPIcommandAlignment))) SCPIcommand;

/**
 * Structure defining a command tree node
 */
typedef struct CommandNode {
    SCPIcommand scpi;                    ///< Command attached to the node
    const struct CommandNode* children;  ///< Array of children nodes
    uint8_t nb_children;                 ///< Number of children nodes
} __attribute((aligned(kNodeAlignment))) Node;

const Node* getRootNode(void);
#endif
