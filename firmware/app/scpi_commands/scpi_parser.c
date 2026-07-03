/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file scpi_parser.c
 * @brief Implement a SCPI (IEEE 488.2) command parser 
 * @author Gilles Henrard
 *
 * @details
 * SCPI-1999 Documentation : https://www.ivifoundation.org/downloads/SCPI/scpi-99.pdf
 * Wikipedia : https://en.wikipedia.org/wiki/Standard_Commands_for_Programmable_Instruments
 * IVI Foundation : https://www.ivifoundation.org/About-IVI/scpi.html
 */
#include "scpi_parser.h"

#include <ctype.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "errorstack.h"
#include "leany_std.h"
#include "scpi_commands.h"
#include "serial_command_types.h"
#include "task_serial.h"

enum : uint8_t {
    kReceiveBufferSize = 32U,  ///< Number of bytes the reception buffer can hold
    kParameterSize = 20U,      ///< Maximum number of characters a parameter can span
};

/**
 * Enumeration of the possible SCPI parser states
 */
typedef enum : uint8_t {
    kState_waiting = 0,    ///< Waiting for a sequence start character
    kState_buffering = 1,  ///< Buffering characters received
    kState_parameter = 2,  ///< Buffering parameter characters
} ParserState;

//internal functions
static void stateWaitingForStartChararcter(char new_char);
static bool stateBufferingCharacters(char new_char, SerialCommand* command_received);
static bool stateBufferingParameter(char new_char, SerialCommand* command_received);
static const Node* searchMatchingChildNode(const Node* tree_node, const char* command, uint8_t command_size);
static bool populateCommand(const Node* scpi_node, SerialCommand* command);
static bool finaliseCommand(const Node* node, const char buffer[kReceiveBufferSize], uint8_t index,
                            SerialCommand* command_received);

//constants
static constexpr char kNewBranch = ':';    ///< Character used to indicate a new hierarchy branch
static constexpr char kRequest = '?';      ///< Character used to indicate the command is a request
static constexpr char kConcatenate = ';';  ///< Character used to concatenate commands
static constexpr char kParameter = ' ';    ///< Character used to indicate a parameter
static constexpr char kEndLine = '\n';     ///< Character used to indicate an end of line
static constexpr char kUnknownMessage[] =  ///< Message indicating "unknown command"
    "Unknown command. Type ':help?' for more info";

static char reception_buffer[kReceiveBufferSize];  ///< Buffer used to store bytes for slower computations
static uint8_t buffer_index;                       ///< Current index in the reception buffer
static ParserState state;                          ///< Current parser state
static const Node* current_node = nullptr;         ///< Current command tree node being parsed
static char parameter[kParameterSize];             ///< Buffer used to store a parameter
static uint8_t parameter_index = 0;                ///< Index of the current character saved as parameter

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Reset the SCPI parser state
 */
void resetSCPIparser(void) {
    buffer_index = 0;
    parameter_index = 0;
    current_node = getRootNode();
    state = kState_waiting;
}

/**
 * Receive a new character and parse a SCPI command/request
 *
 * @param new_char Character to parse
 * @param[out] command_received Command to fill with parsed information
 * @retval 0 The current command is not done parsing
 * @return Code of a command done parsing, if any
 */
bool pushSCPIcharacter(uint8_t new_char, SerialCommand* command_received) {
    switch (state) {
        case kState_waiting:
            stateWaitingForStartChararcter(new_char);
            break;

        case kState_buffering:
            return stateBufferingCharacters(new_char, command_received);
            break;

        case kState_parameter:
            return stateBufferingParameter(new_char, command_received);
            break;

        default:
            break;
    }

    return false;
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * State in which the parser waits for a sequence start character
 *
 * @param new_char Character to check
 */
static void stateWaitingForStartChararcter(char new_char) {
    if (new_char == kEndLine) {
        logSerial(kMaxErrorLevel, kUnknownMessage);
        return;
    }

    //character not a sequence start, exit
    if (new_char != kNewBranch) {
        return;
    }

    state = kState_buffering;
}

/**
 * State in which incoming characters are stored into a circular command buffer
 *
 * @param new_char New character to store
 * @param[out] command_received Command to fill with parsed information
 * @retval true Command is done parsing
 * @retval false Command still needs parsing
 */
static bool stateBufferingCharacters(char new_char, SerialCommand* command_received) {
    (void)command_received;

    //character indicates end of the command
    if ((new_char == kConcatenate) || (new_char == kEndLine)) {
        return finaliseCommand(current_node, reception_buffer, buffer_index, command_received);
    }

    //new hierarchy branch requested
    if (new_char == kNewBranch) {
        //if matching node found, keep parsing against its children
        const Node* found = searchMatchingChildNode(current_node, reception_buffer, buffer_index);

        command_received->is_read = false;
        if (found) {
            current_node = found;
            buffer_index = 0;
        } else {
            resetSCPIparser();
        }

        return false;
    }

    if (new_char == kParameter) {
        //NOLINTNEXTLINE(clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
        memset(parameter, '\0', kParameterSize);
        state = kState_parameter;
        return false;
    }

    //current command is a request
    if (new_char == kRequest) {
        command_received->is_read = true;
        return false;
    }

    //store the character in the buffer
    reception_buffer[buffer_index] = new_char;
    buffer_index = (buffer_index + 1U) % kReceiveBufferSize;

    return false;
}

/**
 * State in which incoming characters are stored into a circular parameter buffer
 *
 * @param new_char New character to store
 * @param[out] command_received Command to fill with parsed information
 * @return Parameter buffering status
 */
static bool stateBufferingParameter(char new_char, SerialCommand* command_received) {
    if ((new_char == kConcatenate) || (new_char == kEndLine)) {
        return stateBufferingCharacters(new_char, command_received);
    }

    if (!isalnum(new_char) && (new_char != '.')) {
        logSerial(kErrorError, "Invalid parameter : %c", new_char);
        resetSCPIparser();
        return false;
    }

    if (parameter_index >= (kParameterSize - 1U)) {
        parameter[parameter_index] = '\0';
        return stateBufferingCharacters(kEndLine, command_received);
    }

    parameter[parameter_index++] = new_char;

    return false;
}

/**
 * Search for the command in the node's children
 *
 * @param tree_node     Node in which childrens search for the command
 * @param command       Command to check 
 * @param command_size  Number of characters in the command
 * @return Address of the node if found, nullptr otherwise
 */
static const Node* searchMatchingChildNode(const Node* tree_node, const char* command, const uint8_t command_size) {
    //no node or node does not have any children
    if (!tree_node || !tree_node->children) {
        return nullptr;
    }

    //search for a match in all the children
    for (uint8_t index = 0; index < tree_node->nb_children; index++) {
        const SCPIcommand* child_command = &tree_node->children[index].scpi;
        const uint32_t short_length = getStringLength(child_command->short_name, kSCPImaxCommandSize);
        const uint32_t long_length = getStringLength(child_command->long_name, kSCPImaxCommandSize);

        //if string length does not match either commands length, no need to compare
        if ((command_size != short_length) && (command_size != long_length)) {
            continue;
        }

        //if command matches either the short or the long name, return the node's address
        if (!compareString(child_command->short_name, short_length, command, short_length) ||
            !compareString(child_command->long_name, short_length, command, long_length)) {
            return &tree_node->children[index];
        }
    }

    return nullptr;
}

/**
 * Populate a command with the contents of a SCPI node
 *
 * @param scpi_node Node from which get the contents
 * @param[out] command Command to populate
 * @retval true Success
 * @retval false Failure
 */
static bool populateCommand(const Node* scpi_node, SerialCommand* command) {
    if (!command) {
        logSerial(kErrorError, "No command given to populate");
        return false;
    }

    if (!scpi_node) {
        command->code = kCmdNoBehaviour;
        return false;
    }

    command->code = scpi_node->scpi.code;
    command->param_type = scpi_node->scpi.param_type;

    switch (command->param_type) {
        case kParamInteger:
            command->parameter.int_value = stringToInt(parameter);
            break;

        case kParamHexa:
            command->parameter.int_value = stringHexToInt(parameter);
            break;

        case kParamFloat:
            command->parameter.float_value = stringToFloat(parameter);
            break;

        default:
            logSerial(kErrorError, "Parameter type not known : %u", command->param_type);
            return false;
    }

    return true;
}

/**
 * Check a matching command node and populate the command if so
 *
 * @param node Current node
 * @param buffer Serial command reception buffer
 * @param index Index in the reception buffer
 * @param command_received Command received
 * @return Command validity
 */
static bool finaliseCommand(const Node* node, const char buffer[kReceiveBufferSize], const uint8_t index,
                            SerialCommand* command_received) {
    const Node* found = searchMatchingChildNode(node, buffer, index);
    const bool valid = populateCommand(found, command_received);
    resetSCPIparser();
    if (!found) {
        logSerial(kMaxErrorLevel, kUnknownMessage);
    }
    return (bool)((found != nullptr) && valid);
}
