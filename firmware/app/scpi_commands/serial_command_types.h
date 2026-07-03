/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 *
 * @file serial_command_types.h
 * @details Describe generic serial commands
 * @author Gilles Henrard
 */
#ifndef APPS_SCPI_COMMANDS_SERIAL_COMMAND_TYPES_H
#define APPS_SCPI_COMMANDS_SERIAL_COMMAND_TYPES_H
#include <stdint.h>

enum : uint8_t {
    kCommandAlignment = 16U,  ///< Memory alignment of the Command structure
};

/**
 * Enumeration of the command codes available
 */
typedef enum : uint8_t {
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
    kCmdBatteryOff = 15,      ///< Turn the battery charger OFF
} SerialCommandCode;

/**
 * Enumeration of the parameter types
 */
typedef enum : uint8_t {
    kParamInteger = 0,  ///< Parameter is a decimal integer
    kParamHexa,         ///< Parameter is a hexadecimal integer
    kParamFloat,        ///< Parameter is a float
} SerialParameterType;

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
    SerialCommandCode code;          ///< Command corresponding code
    bool is_read;                    ///< Whether the command is a read request
    SerialParameterType param_type;  ///< Type of the parameter
    GenericParameter parameter;      ///< Parameter value, if any
} __attribute((aligned(kCommandAlignment))) SerialCommand;

/**
 * Reset a command's fields value to default
 *
 * @param[out] command Command to reset
 */
static inline void resetCommand(SerialCommand* command) { *command = (SerialCommand){0}; }

#endif
