/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef HARDWARE_SERIAL_GENERIC_COMMAND_INC
#define HARDWARE_SERIAL_GENERIC_COMMAND_INC
#include <stdint.h>

enum {
    kCommandAlignment = 16U,  ///< Memory alignment of the Command structure
};

/**
 * Enumeration of the command codes available
 */
typedef enum {
    kCmdNoBehaviour = 0,      ///< Command node has no behaviour (default)
    kCmdKP = 1,               ///< Set/Get the filter kP value
    kCmdKI = 2,               ///< Set/Get the filter kI value
    kCmdAlignmentEnable = 3,  ///< Enable/Disable the alignment check
    kCmdLogLevel = 4,         ///< Get/Set the logging level
} CommandCode;

/**
 * Enumeration of the parameter types
 */
typedef enum {
    kParamInteger = 0,  ///< Parameter is an integer
    kParamFloat,        ///< Parameter is a float
} ParameterType;

/**
 * Generic space occupied by a parameter
 */
typedef union {
    uint32_t int_value;  ///< Integer value
    float float_value;   ///< Float value
} GenericParameter;

/**
 * Structure defining a generic command
 */
typedef struct {
    CommandCode code;            ///< Command corresponding code
    uint8_t is_read;             ///< Whether the command is a read request
    ParameterType param_type;    ///< Type of the parameter
    GenericParameter parameter;  ///< Parameter value, if any
} __attribute((aligned(kCommandAlignment))) GenericCommand;

/**
 * Reset a command's fields value to default
 *
 * @param[out] command Command to reset
 */
static inline void resetCommand(GenericCommand* command) { *command = (GenericCommand){0}; }

#endif
