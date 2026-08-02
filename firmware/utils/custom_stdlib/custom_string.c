/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 *
 * @file custom_string.c
 * @brief Lightweight standard functions implementation for embedded systems
 */
#include "custom_string.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#include "custom_stringparser.h"

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Format and write to a string with size limit
 * 
 * This function formats a string according to the format specifier and
 * writes it to the buffer, ensuring null-termination and preventing
 * buffer overflow.
 * 
 * @param[out] buffer Destination buffer
 * @param size Size of destination buffer (including null terminator)
 * @param format Format string
 * @param args Variable argument list
 * @return Number of characters written (excluding null terminator), or -1 on error
 * 
 * @note The function always null-terminates the buffer if size > 0
 * @note If the formatted string would exceed size-1, it is truncated
 */
int32_t custom_vsnprintf(char* buffer, size_t size, const char* format, va_list* args) {
    /* Validate parameters */
    if (!buffer || !format || !size) {
        return -1;
    }

    ParserContext context;
    if (!initialiseContext(&context, buffer, size)) {
        return -1;
    }

    uint32_t format_index = 0U;
    for (format_index = 0U; format_index < kMaxFormatLength; format_index++) {
        (void)pushCharacter(&context, format[format_index], args);
    }

    return 0;
}

/**
 * @brief Format and write to a string with size limit
 * 
 * This function formats a string according to the format specifier and
 * writes it to the buffer, ensuring null-termination and preventing
 * buffer overflow.
 * 
 * @param[out] buffer Destination buffer
 * @param[in] size Size of destination buffer (including null terminator)
 * @param[in] format Format string
 * @param[in] ... Variable arguments
 * @return Number of characters written (excluding null terminator), or -1 on error
 * 
 * @note The function always null-terminates the buffer if size > 0
 * @note If the formatted string would exceed size-1, it is truncated
 */
int32_t custom_snprintf(char* buffer, size_t size, const char* format, ...) {
    va_list args;  // NOLINT (cppcoreguidelines-init-variables)

    va_start(args, format);
    int32_t result = custom_vsnprintf(buffer, size, format, &args);
    va_end(args);

    return result;
}

/**
 * Check if a character is a number
 *
 * @param character Character to check
 * @retval true Character is a number
 * @retval false Character is not
 */
bool isnumber(char character) { return (bool)((character >= '0') && (character <= '9')); }

/**
 * Check if a character is a sign ('+' or '-')
 *
 * @param character Character to check
 * @retval true Character is a sign
 * @retval false Character is not
 */
bool issign(char character) { return (bool)((character == '+') || (character == '-')); }

/**
 * Check if a character is an upper case alpha character
 *
 * @param character Character to check
 * @retval true The character is upper case
 * @retval false The character is not upper case
 */
bool isAlphaUppercase(char character) { return (bool)((character >= (uint8_t)'A') && (character <= (uint8_t)'Z')); }

/**
 * Check if a character is a lower case alpha character
 *
 * @param character Character to check
 * @retval true The character is lower case
 * @retval false The character is not lower case
 */
bool isAlphaLowercase(char character) { return (bool)((character >= (uint8_t)'a') && (character <= (uint8_t)'z')); }
