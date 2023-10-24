/**
 * @file errors.c
 * @brief Implement a structure to create a crude error codes stack trace.
 * @author Gilles Henrard
 * @date 24/10/2023
 *
 * @details
 * This library allows users to trace an error through several function calls back to its source.
 *
 * It is especially useful because it remains modular and does not require a discrete error code table,
 * while providing a granular tracing of each error.
 *
 * The error codes structure is as follows :
 *
 * | Module ID  | Function ID |  Layer 0   |  Layer 1  | Layer 2  | Layer 3  |
 * |------------|-------------|------------|-----------|----------|----------|
 * | Up to 64   | Up to 64    | Up to 16   | Up to 16  | Up to 16 | Up to 16 |
 * |------------|-------------|------------|-----------|----------|----------|
 * | bits 22-27 | bits 16-21  | bits 12-15 | bits 8-11 | bits 4-7 | bits 0-3 |
 *
 * - Module ID : The entity using all modules (e.g. main()) assigns an ID to each module
 * - Function ID : A module assigns an ID to all of its functions and states
 * - Layers : Each function receiving an error code stacks it above all else to allow for a chaining
 *
 * Layer 0 is the highest layer called, and layer 3 is the lowest.
 *
 * Example :
 * A function A calls a function B, which in turn calls a function C. Let's imagine C returns an error.
 *
 * - C sets its function ID and an error code at layer 0. (function 4, code 1)
 *
 * | 0  | 4 |  1   |  0  | 0  | 0  |
 *
 * - B receives said code, updates the function ID and pushes its own error code in layer 0. (function 6, code 3)
 *
 * | 0  | 6 |  3   |  1  | 0  | 0  |
 *
 * - A receives the code, updates the function ID and pushes its own error code in layer 0. (function 8, code 2)
 *
 * | 0  | 8 |  2   |  3  | 1  | 0  |
 *
 * - main() receives the final code, applies a module ID and logs the error (module ID 4)
 *
 * | 4  | 8 |  2   |  3  | 1  | 0  |
 */
#include "errors.h"
#include "stm32f1xx.h"

#define SUCCESS_VALUE	0x00000000U
const errorCode_u ERR_SUCCESS = { .dword = SUCCESS_VALUE };

/**
 * @brief Update an error code from a lower layer function
 *
 * @param received Error code to update
 * @param functionID Function ID to replace with
 * @param newCode Return code to push in the stack
 * @return Formatted code
 */
errorCode_u errorCode(errorCode_u received, uint32_t functionID, uint32_t newCode){
	UNUSED(received);
	UNUSED(functionID);
	UNUSED(newCode);

	return (ERR_SUCCESS);
}
