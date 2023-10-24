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
 *
 * @note When pushing a code in the stack. Any code already stored in layer 3 is lost.
 */
#include "errors.h"

//definitions
#define ERR_IDS_MASK		0xFFFF0000U		///< Value used to erase the codes stack
#define ERR_STACK_MASK		0x0000FFFFU		///< Value used to erase the IDs fields
#define ERR_FUNCTION_MASK	0xFFC0FFFFU		///< Value used to erase the function ID
#define SUCCESS_VALUE		0x00000000U		///< Value assigned to successes
#define ERR_LAYER0_OFFSET	12U				///< Number of bits to shift a code to reach the layer 0
#define ERR_LAYER1_OFFSET	8U				///< Number of bits to shift a code to reach the layer 1
#define ERR_FUNCTION_OFFSET	16U				///< Number of bits to shift an ID to reach the function ID

//global variables
const errorCode_u ERR_SUCCESS = { .dword = SUCCESS_VALUE };	///< Variable used as a success code

/**
 * @brief Update an error code from a lower layer function
 *
 * @param received Error code to update
 * @param functionID Function ID to replace with
 * @param newCode Return code to push in the stack
 * @return Formatted code
 */
errorCode_u errorCode(errorCode_u received, uint32_t functionID, uint32_t newCode){
	uint32_t code;

	//if code means success, return success
	if(newCode == SUCCESS_VALUE)
		return (ERR_SUCCESS);

	//if function ID too large, do nothing
	if(functionID >= (1 << ERR_ID_NBBITS))
		return (received);

	//if error code too large, do nothing
	if(newCode >= (1 << ERR_LAYER_NBBITS))
		return (received);

	//erase and replace the function ID
	received.dword &= ERR_FUNCTION_MASK;
	received.dword |= (functionID << ERR_FUNCTION_OFFSET);

	//isolate the codes stack, shift it and push a new code
	//	(code already in layer 3 is lost)
	code = (received.dword & ERR_STACK_MASK);
	code >>= ERR_LAYER_NBBITS;
	code |= (newCode << ERR_LAYER0_OFFSET);

	//erase the codes stack and replace it
	received.dword &= ERR_IDS_MASK;
	received.dword |= code;

	//return the final code
	return (received);
}

/**
 * @brief Create a code with a layer 0 and a layer 1
 * @note This is to be used when receiving a code from a lower level library (such as HAL)
 *
 * @param functionID Function ID to replace with
 * @param newCode Return code to set at layer 1
 * @param layer0Code Return code to set at layer 0
 * @return New code
 */
errorCode_u errorCodeLayer0(uint32_t functionID, uint32_t newCode, uint32_t layer0Code){
	errorCode_u code = ERR_SUCCESS;

	//if code means success, return success
	if(layer0Code == SUCCESS_VALUE)
		return (ERR_SUCCESS);

	//if function ID too large, do nothing
	if(functionID >= (1 << ERR_ID_NBBITS))
		return (ERR_SUCCESS);

	//if error code too large, do nothing
	if(newCode >= (1 << ERR_LAYER_NBBITS))
		return (ERR_SUCCESS);

	//if error code too large, do nothing
	if(layer0Code >= (1 << ERR_LAYER_NBBITS))
		return (ERR_SUCCESS);

	//update with the codes received
	code.dword |= (functionID << ERR_FUNCTION_OFFSET);
	code.dword |= (newCode << ERR_LAYER1_OFFSET);
	code.dword |= (layer0Code << ERR_LAYER0_OFFSET);

	return (code);
}
