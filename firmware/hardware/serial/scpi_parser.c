/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file scpi_parser.c
 * @brief Implement a SCPI (IEEE 488.2) command parser 
 * @author Gilles Henrard
 * @date 02/10/2025
 *
 * @details
 * SCPI-1999 Documentation : https://www.ivifoundation.org/downloads/SCPI/scpi-99.pdf
 * Wikipedia : https://en.wikipedia.org/wiki/Standard_Commands_for_Programmable_Instruments
 * IVI Foundation : https://www.ivifoundation.org/About-IVI/scpi.html
 */
#include "scpi_parser.h"

#include <ctype.h>
#include <stdint.h>
#include <string.h>

#include "errorstack.h"
#include "generic_command.inc"
#include "scpi_commands.h"
#include "serial.h"

enum {
    kReceiveBufferSize = 32U,  ///< Number of bytes the reception buffer can hold
    kParameterSize = 20U,      ///< Maximum number of characters a parameter can span
};

/**
 * Enumeration of the possible SCPI parser states
 */
typedef enum {
    kState_waiting = 0,    ///< Waiting for a sequence start character
    kState_buffering = 1,  ///< Buffering characters received
    kState_parameter = 2,  ///< Buffering parameter characters
} ParserState;

//internal functions
static void stateWaitingForStartChararcter(char new_char);
static uint8_t stateBufferingCharacters(char new_char, GenericCommand* command_received);
static uint8_t stateBufferingParameter(char new_char, GenericCommand* command_received);
static float toFloat(const char string[]);
static uint32_t toInt(const char string[]);
static Node* searchMatchingChildNode(const Node* tree_node, const char* command, uint8_t command_size);
static inline uint8_t isnumberdot(char character);
static inline uint8_t issign(char character);
static uint8_t populateCommand(const Node* scpi_node, GenericCommand* command);

//constants
static const char kNewBranch = ':';    ///< Character used to indicate a new hierarchy branch
static const char kRequest = '?';      ///< Character used to indicate the command is a request
static const char kConcatenate = ';';  ///< Character used to concatenate commands
static const char kParameter = ' ';    ///< Character used to indicate a parameter
static const char kEndLine = '\n';     ///< Character used to indicate an end of line

static char reception_buffer[kReceiveBufferSize];  ///< Buffer used to store bytes for slower computations
static uint8_t buffer_index;                       ///< Current index in the reception buffer
static ParserState state;                          ///< Current parser state
static const Node* current_node = NULL;            ///< Current command tree node being parsed
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
uint8_t pushSCPIcharacter(uint8_t new_char, GenericCommand* command_received) {
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

    return 0;
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * State in which the parser waits for a sequence start character
 *
 * @param new_char Character to check
 */
static void stateWaitingForStartChararcter(char new_char) {
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
 * @retval 1 Command is done parsing
 * @retval 0 Command still needs parsing
 */
static uint8_t stateBufferingCharacters(char new_char, GenericCommand* command_received) {
    (void)command_received;

    Node* found = NULL;

    //character indicates end of the command
    if ((new_char == kConcatenate) || (new_char == kEndLine)) {
        found = searchMatchingChildNode(current_node, reception_buffer, buffer_index);
        uint8_t valid = populateCommand(found, command_received);
        resetSCPIparser();
        return ((found != NULL) && valid);
    }

    //new hierarchy branch requested
    if (new_char == kNewBranch) {
        //if matching node found, keep parsing against its children
        found = searchMatchingChildNode(current_node, reception_buffer, buffer_index);

        command_received->is_read = 0;
        if (found) {
            current_node = found;
            buffer_index = 0;
        } else {
            resetSCPIparser();
        }

        return 0;
    }

    if (new_char == kParameter) {
        //NOLINTNEXTLINE(clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
        memset(parameter, '\0', kParameterSize);
        state = kState_parameter;
        return 0;
    }

    //current command is a request
    if (new_char == kRequest) {
        command_received->is_read = 1;
        return 0;
    }

    //store the character in the buffer
    reception_buffer[buffer_index] = (char)toupper(new_char);
    buffer_index = (buffer_index + 1U) % kReceiveBufferSize;

    return 0;
}

/**
 * State in which incoming characters are stored into a circular parameter buffer
 *
 * @param new_char New character to store
 * @param[out] command_received Command to fill with parsed information
 * @return 0
 */
static uint8_t stateBufferingParameter(char new_char, GenericCommand* command_received) {
    if ((new_char == kConcatenate) || (new_char == kEndLine)) {
        return stateBufferingCharacters(new_char, command_received);
    }

    if (!isalnum(new_char) && (new_char != '.')) {
        logSerial(kErrorError, "Invalid parameter : %c", new_char);
        resetSCPIparser();
        return 0;
    }

    if (parameter_index >= (kParameterSize - 1U)) {
        parameter[parameter_index] = '\0';
        return stateBufferingCharacters(kEndLine, command_received);
    }

    parameter[parameter_index++] = new_char;

    return 0;
}

/**
 * Search for the command in the node's children
 *
 * @param tree_node     Node in which childrens search for the command
 * @param command       Command to check 
 * @param command_size  Number of characters in the command
 * @return Address of the node if found, NULL otherwise
 */
static Node* searchMatchingChildNode(const Node* tree_node, const char* command, const uint8_t command_size) {
    //no node or node does not have any children
    if (!tree_node || !tree_node->children) {
        return NULL;
    }

    //search for a match in all the children
    for (uint8_t index = 0; index < tree_node->nb_children; index++) {
        const SCPIcommand* child_command = &tree_node->children[index].scpi;
        const uint32_t short_length = strlen(child_command->short_name);
        const uint32_t long_length = strlen(child_command->long_name);

        //if string length does not match either commands length, no need to compare
        if ((command_size != short_length) && (command_size != long_length)) {
            continue;
        }

        //if command matches either the short or the long name, return the node's address
        if (!strncmp(child_command->short_name, command, short_length) ||
            !strncmp(child_command->long_name, command, long_length)) {
            return (Node*)&tree_node->children[index];
        }
    }

    return NULL;
}

/**
 * Parse a string to a float
 *
 * @param string String to parse
 * @return Parsed float value
 */
static float toFloat(const char string[]) {
    float final = 0.0F;

    uint8_t length = 0;
    if (issign(string[0])) {  //allow the first character to be a sign
        length = 1;
    }
    while ((length < (uint8_t)kFloatBufferSize) && isnumberdot(string[length])) {
        length++;
    }

    uint8_t index = (issign(string[0]) ? 1 : 0);
    while ((index < length) && (string[index] != '.')) {
        final *= 10.0F;  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
        final += (float)(string[index] - '0');
        index++;
    }

    if (string[index] == '.') {
        index++;
    }

    const float divide_10 = 0.1F;
    float multiplier = divide_10;
    while (index < length) {
        final += ((float)(string[index] - '0') * multiplier);
        multiplier *= divide_10;
        index++;
    }

    if (string[0] == '-') {
        final = -final;
    }

    return final;
}

/**
 * Parse a string to an integer
 *
 * @param string String to parse
 * @return Parsed int value
 */
static uint32_t toInt(const char string[]) {
    uint8_t length = 0;
    while ((length < (uint8_t)kFloatBufferSize) && (string[length] != '.') && isnumberdot(string[length])) {
        length++;
    }

    uint32_t value = 0;
    for (uint8_t index = 0; index < length; index++) {
        value *= 10U;  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
        value += (uint32_t)(string[index] - '0');
    }

    return value;
}

/**
 * Check if a character is a letter or a dot
 *
 * @param character Character to check
 * @retval 1 Character is a letter or a dot
 * @retval 0 Character is not
 */
static inline uint8_t isnumberdot(char character) {
    return ((character == '.') || ((character >= '0') && (character <= '9')));
}

/**
 * Check if a character is a sign ('+' or '-')
 *
 * @param character Character to check
 * @retval 1 Character is a sign
 * @retval 0 Character is not
 */
static inline uint8_t issign(char character) { return ((character == '+') || (character == '-')); }

/**
 * Populate a command with the contents of a SCPI node
 *
 * @param scpi_node Node from which get the contents
 * @param[out] command Command to populate
 * @retval 1 Success
 * @retval 0 Failure
 */
static uint8_t populateCommand(const Node* scpi_node, GenericCommand* command) {
    if (!command) {
        logSerial(kErrorError, "No command given to populate");
        return 0;
    }

    if (!scpi_node) {
        command->code = kCmdNoBehaviour;
        return 0;
    }

    command->code = scpi_node->scpi.code;
    command->param_type = scpi_node->scpi.param_type;

    switch (command->param_type) {
        case kParamInteger:
            command->parameter.int_value = toInt(parameter);
            break;

        case kParamFloat:
            command->parameter.float_value = toFloat(parameter);
            break;

        default:
            logSerial(kErrorError, "Parameter type not known : %u", command->param_type);
            return 0;
    }

    return 1;
}
