/**
 * @file errors.c
 * @brief Implement a structure to create a crude error codes stack trace.
 * @author Gilles Henrard
 * @date 28/10/2023
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
 * Layer 0 is the highest layer called, and layer 3 is the lowest.
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
#include "errors.h"

//definitions
#define ERR_LEVEL_MASK		0x3FFFFFFFU		///< Value used to erase the error level
#define ERR_STACK_MASK		0xFFFF0000U		///< Value used to erase the codes stack
#define ERR_IDS_MASK		0x0000FFFFU		///< Value used to erase the IDs fields
#define ERR_FUNCTION_MASK	0xFFC0FFFFU		///< Value used to erase the function ID
#define SUCCESS_VALUE		0x00000000U		///< Value assigned to successes
#define ERR_LAYER0_OFFSET	12U				///< Number of bits to shift a code to reach the layer 0
#define ERR_LAYER1_OFFSET	8U				///< Number of bits to shift a code to reach the layer 1
#define ERR_FUNCTION_OFFSET	16U				///< Number of bits to shift an ID to reach the function ID
#define ERR_LEVEL_OFFSET	30U				///< Number of bits to shift an level to reach the level field

//global variables
const errorCode_u ERR_SUCCESS = { .dword = SUCCESS_VALUE };	///< Variable used as a success code

/**
 * @brief Update an error code from a lower layer function
 *
 * @param received Error code to update
 * @param functionID Function ID to replace with
 * @param newCode Return code to push in the stack
 * @param level Error level
 * @return Formatted code
 */
errorCode_u errorCode(errorCode_u received, uint32_t functionID, uint32_t newCode/*, errorLevel_e level*/){
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
/*
	//if no level set yet, update it
	if(!(received.dword && ~ERR_LEVEL_MASK)){
		received.dword &= ERR_LEVEL_MASK;
		received.dword |= (level << ERR_LEVEL_OFFSET);
	}
*/
	//erase and replace the function ID
	received.dword &= ERR_FUNCTION_MASK;
	received.dword |= (functionID << ERR_FUNCTION_OFFSET);

	//isolate the codes stack, shift it and push a new code
	//	(code already in layer 3 is lost)
	code = (received.dword & ERR_IDS_MASK);
	code >>= ERR_LAYER_NBBITS;
	code |= (newCode << ERR_LAYER0_OFFSET);

	//erase the codes stack and replace it
	received.dword &= ERR_STACK_MASK;
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
 * @param level Error level
 * @return New code
 */
errorCode_u errorCodeLayer0(uint32_t functionID, uint32_t newCode, uint32_t layer0Code/*, errorLevel_e level*/){
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
/*
	//set the error level
	code.dword &= ERR_LEVEL_MASK;
	code.dword |= (level << ERR_LEVEL_OFFSET);
*/
	//update with the codes received
	code.dword |= (functionID << ERR_FUNCTION_OFFSET);
	code.dword |= (newCode << ERR_LAYER1_OFFSET);
	code.dword |= (layer0Code << ERR_LAYER0_OFFSET);

	return (code);
}
