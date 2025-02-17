#ifndef INC_ERRORS_ERRORS_H_
#define INC_ERRORS_ERRORS_H_
#include <stdint.h>

//definitions
enum {
    ERR_LAYER_NBBITS = 4U,   ///< Amount of bits in a return code layer field
    ERR_ID_NBBITS    = 7U,   ///< Amount of bits in function ID and module ID fields
    ERR_LEVEL_NBBITS = 2U,   ///< Amount of level bits
    STACK_ALIGNMENT  = 32U,  ///< Memory alignment of the error stack structure
};

/**
 * @brief Error levels possible
 */
typedef enum {
    ERR_INFO = 0,  ///< Simple information
    ERR_WARNING,   ///< Warning
    ERR_ERROR,     ///< Non-critical error
    ERR_CRITICAL   ///< Critical error
} errorLevel_e;

/**
 * @brief Union implementing the error codes structure
 */
typedef union {
    /**
     * @brief Structure defining the discrete fields of an error code
     *
     * @return Value with an optimised alignment
     */
    struct {
        uint32_t layer3 : ERR_LAYER_NBBITS;   ///< Layer 3 return code (lowest layer)
        uint32_t layer2 : ERR_LAYER_NBBITS;   ///< Layer 2 return code
        uint32_t layer1 : ERR_LAYER_NBBITS;   ///< Layer 1 return code
        uint32_t layer0 : ERR_LAYER_NBBITS;   ///< Layer 0 return code (highest layer)
        uint32_t functionID : ERR_ID_NBBITS;  ///< ID of the highest function which returns the code
        uint32_t moduleID : ERR_ID_NBBITS;    ///< ID of the module returning the code
        uint32_t level : ERR_LEVEL_NBBITS;    ///< Error level
    } __attribute__((aligned(STACK_ALIGNMENT)));

    uint32_t dword;  ///< All 32 bits of the code at once
} errorCode_u;

//global variables
extern const errorCode_u ERR_SUCCESS;

//error management functions
//NOLINTBEGIN(bugprone-easily-swappable-parameters)
errorCode_u createErrorCode(uint8_t functionID, uint8_t newError, errorLevel_e level);
errorCode_u createErrorCodeLayer1(uint8_t functionID, uint8_t newError, uint8_t layer1Code, errorLevel_e level);
errorCode_u pushErrorCode(errorCode_u oldCode, uint8_t functionID, uint8_t newError);
//NOLINTEND(bugprone-easily-swappable-parameters)

/**
 * @brief Check if a code represents an error
 * @note This is done by verifying if a layer 0 code exists
 * 
 * @param   code    Code to check
 * @return  Non-zero if error
 */
inline uint8_t isError(const errorCode_u code) {
    return (code.layer0);
}

#endif /* INC_ERRORS_ERRORS_H_ */
