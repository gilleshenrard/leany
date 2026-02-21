/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef INC_ERRORS_ERRORS_H
#define INC_ERRORS_ERRORS_H
#include <main.h>
#include <stdint.h>

//definitions
enum {
    kErrorLayerNbBits = 4U,  ///< Amount of bits in a return code layer field
    kErrorIDnbBits = 7U,     ///< Amount of bits in function ID and module ID fields
    kErrorLevelNbBits = 2U,  ///< Amount of level bits
};

/**
 * @brief Error levels possible
 */
typedef enum {
    kErrorDebug = 0,  ///< Debug information (not advised, very verbose)
    kErrorInfo,       ///< Simple information
    kErrorWarning,    ///< Warning
    kErrorError,      ///< Non-critical error
    kErrorCritical,   ///< Critical error
    kMaxErrorLevel    ///< Maximum error level
} ErrorLevel;

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
        uint32_t layer3 : kErrorLayerNbBits;    ///< Layer 3 return code (lowest layer)
        uint32_t layer2 : kErrorLayerNbBits;    ///< Layer 2 return code
        uint32_t layer1 : kErrorLayerNbBits;    ///< Layer 1 return code
        uint32_t layer0 : kErrorLayerNbBits;    ///< Layer 0 return code (highest layer)
        uint32_t function_id : kErrorIDnbBits;  ///< ID of the highest function which returns the code
        uint32_t module_id : kErrorIDnbBits;    ///< ID of the module returning the code
        uint32_t level : kErrorLevelNbBits;     ///< Error level
    };

    uint32_t dword;  ///< All 32 bits of the code at once
} ErrorCode;

//global variables
extern const ErrorCode kSuccessCode;

//error management functions
//NOLINTBEGIN(bugprone-easily-swappable-parameters)
ErrorCode createErrorCode(uint8_t function_id, uint8_t new_error, ErrorLevel level);
ErrorCode createErrorCodeLayer1(uint8_t function_id, uint8_t new_error, uint8_t layer1_code, ErrorLevel level);
ErrorCode pushErrorCode(ErrorCode old_code, uint8_t function_id, uint8_t new_error);
//NOLINTEND(bugprone-easily-swappable-parameters)

/**
 * @brief Check if a code represents an error
 * @note This is done by verifying if a layer 0 code exists
 * 
 * @param   code    Code to check
 * @return  Non-zero if error
 */
inline uint8_t isError(const ErrorCode code) { return (code.layer0); }

/**
 * Macro used to simplify error-on-exit code
 */
#define EXIT_ON_ERROR(result, functionID, errorCode)         \
    if (isError(result)) {                                   \
        return pushErrorCode(result, functionID, errorCode); \
    }

/**
 * Macro used to avoid bloating the code with timeout loops
 */
#define EXIT_ON_TIMEOUT(condition, timeout_ms, function_id, error_code) \
    timeout_value = 0;                                                  \
    while (!(condition) && !timeout_value) {                            \
        timeout_value = timeout(start_tick, timeout_ms);                \
    };                                                                  \
    if (timeout_value) {                                                \
        return createErrorCode(function_id, error_code, kErrorError);   \
    }

#endif /* INC_ERRORS_ERRORS_H */
