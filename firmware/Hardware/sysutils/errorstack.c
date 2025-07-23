/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file errorstack.c
 * @brief Implement a structure to create a crude error codes stack trace.
 * @author Gilles Henrard
 * @date 25/07/2025
 *
 * @details
 * This library allows users to trace an error through several function calls back to its source.
 *
 * It is especially useful because it remains modular and does not require a discrete error code table,
 * while providing a granular tracing of each error.
 *
 * The error codes structure is as follows :
 *
 * |   Level    | Module ID  | Function ID |  Layer 0   |  Layer 1  | Layer 2  | Layer 3  |
 * |------------|------------|-------------|------------|-----------|----------|----------|
 * |  Up to 4   | Up to 128  |  Up to 128  |  Up to 16  | Up to 16  | Up to 16 | Up to 16 |
 * |------------|------------|-------------|------------|-----------|----------|----------|
 * | bits 31-30 | bits 29-23 | bits 22-16  | bits 15-12 | bits 11-8 | bits 7-4 | bits 3-0 |
 *
 * - Level : Level of the error (info, warning, error, critical)
 * - Module ID : The entity using all modules (e.g. main()) assigns an ID to each module
 * - Function ID : A module assigns an ID to all of its functions and states
 * - Layers : Each function receiving an error code stacks it above all else to allow for a chaining
 *
 * Layer 0 is the latest layer called, and layer 3 is the earliest.
 *
 * Example :
 * A function A() calls a function B(), which in turn calls a function C(). Let's imagine C() returns an error.
 * The error is a warning (level 1)
 *
 * - C() sets the error level, its function ID and an error code at layer 0. (e.g. function 4, code 2)
 *
 * | 1  | 0  | 4 |  2   |  0  | 0  | 0  |
 *
 * - B() receives said code, updates the function ID and pushes its own error code in layer 0. (e.g. function 6, code 4)
 *
 * | 1  | 0  | 6 |  4   |  2  | 0  | 0  |
 *
 * - A() receives the code, updates the function ID and pushes its own error code in layer 0. (e.g. function 8, code 6)
 *
 * | 1  | 0  | 8 |  6   |  4  | 2  | 0  |
 *
 * - main() receives the final code, applies a module ID and logs the error (e.g. module ID 4)
 *
 * | 1  | 4  | 8 |  6   |  4  | 2  | 0  |
 *
 * @note When pushing a code in the stack. Any code already stored in layer 3 is lost.
 */
#include "errorstack.h"

#include <stdint.h>

//definitions
static const uint32_t kSuccessValue = 0x00000000UL;  ///< Value assigned to successes
static const uint8_t kLayer0codeOffset = 12U;        ///< Number of bits to shift a code to reach the layer 0
static const uint8_t kFunctionIDoffset = 16U;        ///< Number of bits to shift an ID to reach the function ID
static const uint8_t kLevelOffset = 30U;             ///< Number of bits to shift an level to reach the level field
static const uint8_t kFunctionIDclamp =
    0x7FU;  ///< Value used to clamp function ID arguments to the proper number of bits
static const uint8_t kErrorCodeClamp =
    0x0FU;  ///< Value used to clamp error code arguments to the proper number of bits

//global variables
const ErrorCode kSuccessCode = {.dword = kSuccessValue};  ///< Variable used as a success code

/**
 * @brief Create a code with a layer 0
 * @note Input values will be clamped not to overflow their respective fields
 *
 * @param function_id Function ID to replace with
 * @param new_error Return code to set at layer 0
 * @param level Error level
 * @return New code
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
ErrorCode createErrorCode(uint8_t function_id, uint8_t new_error, ErrorLevel level) {
    ErrorCode code = kSuccessCode;

    //if code means success, return success
    if (new_error == kSuccessValue) {
        return (kSuccessCode);
    }

    //set the fields values (clamped if necessary)
    code.dword |= ((uint32_t)level << kLevelOffset);
    code.dword |= ((uint32_t)(function_id & kFunctionIDclamp) << kFunctionIDoffset);
    code.dword |= ((uint32_t)(new_error & kErrorCodeClamp) << kLayer0codeOffset);

    return (code);
}

/**
 * @brief Create a code with a layer 0 and a layer 1
 * @note Input values will be clamped not to overflow their respective fields
 * @note The layer 1 code can be an external library error code (such as HAL's HAL_ERROR)
 *
 * @param function_id Function ID to replace with
 * @param new_error Return code to set at layer 0
 * @param layer1_code Return code to set at layer 1
 * @param level Error level
 * @return New code
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
ErrorCode createErrorCodeLayer1(uint8_t function_id, uint8_t new_error, uint8_t layer1_code, ErrorLevel level) {
    const uint8_t layer1_code_offset = 8U;  ///< Number of bits to shift a code to reach the layer 1
    ErrorCode code = kSuccessCode;

    //if code means success, return success
    if (new_error == kSuccessValue) {
        return (kSuccessCode);
    }

    //set the fields values (clamped if necessary)
    code.dword |= ((uint32_t)level << kLevelOffset);
    code.dword |= ((uint32_t)(function_id & kFunctionIDclamp) << kFunctionIDoffset);
    code.dword |= ((uint32_t)(new_error & kErrorCodeClamp) << kLayer0codeOffset);
    code.dword |= ((uint32_t)(layer1_code & kErrorCodeClamp) << layer1_code_offset);

    return (code);
}

/**
 * @brief Push a new layer upon an error code
 * @note Input values will be clamped not to overflow their respective fields
 *
 * @param old_code Error code to update
 * @param function_id Function ID to replace with
 * @param new_error Return code to push in the stack
 * @return Formatted code
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
ErrorCode pushErrorCode(ErrorCode old_code, uint8_t function_id, uint8_t new_error) {
    const uint32_t code_stack_mask = 0xFFFF0000U;   ///< Value used to erase the codes stack
    const uint32_t function_id_mask = 0xFF80FFFFU;  ///< Value used to erase the function ID
    uint32_t new_error_stack = (old_code.dword & ~code_stack_mask);

    //if code means success, return success
    if (new_error == kSuccessValue) {
        return (kSuccessCode);
    }

    //erase and replace the function ID
    old_code.dword &= function_id_mask;
    old_code.dword |= ((uint32_t)(function_id & kFunctionIDclamp) << kFunctionIDoffset);

    //shift the code stack and push a new code
    //	(code already in layer 3 is lost)
    new_error_stack >>= (uint32_t)kErrorLayerNbBits;
    new_error_stack |= ((uint32_t)(new_error & kErrorCodeClamp) << kLayer0codeOffset);

    //erase the codes stack and replace it with the new one
    old_code.dword &= code_stack_mask;
    old_code.dword |= new_error_stack;

    //return the final code
    return (old_code);
}
