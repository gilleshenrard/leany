#ifndef INC_ERRORS_ERRORS_H_
#define INC_ERRORS_ERRORS_H_
#include <stdint.h>

#define ERR_LAYER_NBBITS	4U	///< Amount of bits in a return code layer field
#define ERR_ID_NBBITS		6U	///< Amount of bits in function ID and module ID fields
#define ERR_UNUSED_NBBITS	4U	///< Amount of unused bits

/**
 * @brief Union implementing the error codes structure
 */
typedef union{
	/**
	 * @brief Structure defining the discrete fields of an error code
	 */
	struct{
		uint32_t layer3		: ERR_LAYER_NBBITS;		///< Layer 3 return code (lowest layer)
		uint32_t layer2		: ERR_LAYER_NBBITS;		///< Layer 2 return code
		uint32_t layer1		: ERR_LAYER_NBBITS;		///< Layer 1 return code
		uint32_t layer0		: ERR_LAYER_NBBITS;		///< Layer 0 return code (highest layer)
		uint32_t functionID	: ERR_ID_NBBITS;		///< ID of the highest function which returns the code
		uint32_t moduleID	: ERR_ID_NBBITS;		///< ID of the module returning the code
		uint32_t unused		: ERR_UNUSED_NBBITS;	///< Unused bits (padding to 32 bits)
	}fields;

	uint32_t dword;	///< All 32 bits of the code at once
}errorCode_u;

//global variables
extern const errorCode_u ERR_SUCCESS;

//error management functions
errorCode_u errorCode(errorCode_u received, uint32_t functionID, uint32_t newCode);

#endif /* INC_ERRORS_ERRORS_H_ */
