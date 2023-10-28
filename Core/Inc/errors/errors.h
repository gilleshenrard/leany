#ifndef INC_ERRORS_ERRORS_H_
#define INC_ERRORS_ERRORS_H_
#include <stdint.h>

//definitions
#define ERR_LAYER_NBBITS	4U	///< Amount of bits in a return code layer field
#define ERR_ID_NBBITS		7U	///< Amount of bits in function ID and module ID fields
#define ERR_LEVEL_NBBITS	2U	///< Amount of level bits

//macros
#define IS_SUCCESS(value)	(value.fields.layer0 == 0)	///< Macro used to know if a code means success
#define IS_ERROR(value)		(value.fields.layer0 != 0)	///< Macro used to know if a code means error

/**
 * @brief Error levels possible
 */
typedef enum{
	ERR_INFO = 0,	///< Simple information
	ERR_WARNING, 	///< Warning
	ERR_ERROR,   	///< Non-critical error
	ERR_CRITICAL 	///< Critical error
}errorLevel_e;

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
		uint32_t level		: ERR_LEVEL_NBBITS;		///< Error level
	}fields;

	uint32_t dword;									///< All 32 bits of the code at once
}errorCode_u;

//global variables
extern const errorCode_u ERR_SUCCESS;

//error management functions
errorCode_u errorCode(errorCode_u received, uint32_t functionID, uint32_t newCode/*, errorLevel_e level*/);
errorCode_u errorCodeLayer0(uint32_t functionID, uint32_t newCode, uint32_t layer0Code/*, errorLevel_e level*/);

#endif /* INC_ERRORS_ERRORS_H_ */
