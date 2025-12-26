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
    kCmdToggleHold = 4,       ///< Toggle measurements holding
    kCmdToggleZero = 5,       ///< Toggle measurements zeroing
    kCmdLogLevel = 6,         ///< Get/Set the logging level
    kCmdOrientation = 7,      ///< Get/Set the current display orientation
    kCmdBatteryPercent = 8,   ///< Get/Set the battery percentage
    kCmdBatteryCharge = 9,    ///< Get/Set the battery charge status
    kCmdToggleScreen = 10,    ///< Toggle main/system screen
    kCmdLedColour = 11,       ///< Set the LED colour
    kCmdLedEffect = 12,       ///< Set the LED effect
    kCmdErrorCode = 13,       ///< Set an error code
    kCmdHelp = 14,            ///< Get the commands help
} CommandCode;

/**
 * Enumeration of the parameter types
 */
typedef enum {
    kParamInteger = 0,  ///< Parameter is a decimal integer
    kParamHexa,         ///< Parameter is a hexadecimal integer
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
