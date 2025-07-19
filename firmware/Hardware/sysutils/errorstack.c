/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file errorstack.c
 * @brief Implement a structure to create a crude error codes stack trace.
 * @author Gilles Henrard
 * @date 20/08/2024
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
static const uint32_t SUCCESS_VALUE     = 0x00000000UL;  ///< Value assigned to successes
static const uint8_t  LAYER0CODE_OFFSET = 12U;           ///< Number of bits to shift a code to reach the layer 0
static const uint8_t  FUNCTIONID_OFFSET = 16U;           ///< Number of bits to shift an ID to reach the function ID
static const uint8_t  LEVEL_OFFSET      = 30U;           ///< Number of bits to shift an level to reach the level field
static const uint8_t  FUNCTIONID_CLAMP =
    0x7FU;  ///< Value used to clamp function ID arguments to the proper number of bits
static const uint8_t ERRORCODE_CLAMP =
    0x0FU;  ///< Value used to clamp error code arguments to the proper number of bits

//global variables
const errorCode_u ERR_SUCCESS = {.dword = SUCCESS_VALUE};  ///< Variable used as a success code

/**
 * @brief Create a code with a layer 0
 * @note Input values will be clamped not to overflow their respective fields
 *
 * @param functionID Function ID to replace with
 * @param newError Return code to set at layer 0
 * @param level Error level
 * @return New code
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
errorCode_u createErrorCode(uint8_t functionID, uint8_t newError, errorLevel_e level) {
    errorCode_u code = ERR_SUCCESS;

    //if code means success, return success
    if(newError == SUCCESS_VALUE) {
        return (ERR_SUCCESS);
    }

    //set the fields values (clamped if necessary)
    code.dword |= ((uint32_t)level << LEVEL_OFFSET);
    code.dword |= ((uint32_t)(functionID & FUNCTIONID_CLAMP) << FUNCTIONID_OFFSET);
    code.dword |= ((uint32_t)(newError & ERRORCODE_CLAMP) << LAYER0CODE_OFFSET);

    return (code);
}

/**
 * @brief Create a code with a layer 0 and a layer 1
 * @note Input values will be clamped not to overflow their respective fields
 * @note The layer 1 code can be an external library error code (such as HAL's HAL_ERROR)
 *
 * @param functionID Function ID to replace with
 * @param newError Return code to set at layer 0
 * @param layer1Code Return code to set at layer 1
 * @param level Error level
 * @return New code
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
errorCode_u createErrorCodeLayer1(uint8_t functionID, uint8_t newError, uint8_t layer1Code, errorLevel_e level) {
    const uint8_t LAYER1CODE_OFFSET = 8U;  ///< Number of bits to shift a code to reach the layer 1
    errorCode_u   code              = ERR_SUCCESS;

    //if code means success, return success
    if(newError == SUCCESS_VALUE) {
        return (ERR_SUCCESS);
    }

    //set the fields values (clamped if necessary)
    code.dword |= ((uint32_t)level << LEVEL_OFFSET);
    code.dword |= ((uint32_t)(functionID & FUNCTIONID_CLAMP) << FUNCTIONID_OFFSET);
    code.dword |= ((uint32_t)(newError & ERRORCODE_CLAMP) << LAYER0CODE_OFFSET);
    code.dword |= ((uint32_t)(layer1Code & ERRORCODE_CLAMP) << LAYER1CODE_OFFSET);

    return (code);
}

/**
 * @brief Push a new layer upon an error code
 * @note Input values will be clamped not to overflow their respective fields
 *
 * @param oldCode Error code to update
 * @param functionID Function ID to replace with
 * @param newError Return code to push in the stack
 * @return Formatted code
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
errorCode_u pushErrorCode(errorCode_u oldCode, uint8_t functionID, uint8_t newError) {
    const uint32_t CODESTACK_MASK  = 0xFFFF0000U;  ///< Value used to erase the codes stack
    const uint32_t FUNCTIONID_MASK = 0xFF80FFFFU;  ///< Value used to erase the function ID
    uint32_t       newErrorStack   = (oldCode.dword & ~CODESTACK_MASK);

    //if code means success, return success
    if(newError == SUCCESS_VALUE) {
        return (ERR_SUCCESS);
    }

    //erase and replace the function ID
    oldCode.dword &= FUNCTIONID_MASK;
    oldCode.dword |= ((uint32_t)(functionID & FUNCTIONID_CLAMP) << FUNCTIONID_OFFSET);

    //shift the code stack and push a new code
    //	(code already in layer 3 is lost)
    newErrorStack >>= (uint32_t)ERR_LAYER_NBBITS;
    newErrorStack |= ((uint32_t)(newError & ERRORCODE_CLAMP) << LAYER0CODE_OFFSET);

    //erase the codes stack and replace it with the new one
    oldCode.dword &= CODESTACK_MASK;
    oldCode.dword |= newErrorStack;

    //return the final code
    return (oldCode);
}
