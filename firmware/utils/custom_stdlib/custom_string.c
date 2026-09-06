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

enum : uint8_t {
    kFloatBufferSize = 20U,    ///< Number of bytes a float to string buffer can hold
    kDecimalUint32Size = 10U,  ///< Number of bytes a decimal 32-bits number can hold
    kHexaUint32Size = 8U,      ///< Number of bytes a hexadecimal 32-bits number can hold
};

static uint32_t qualifyHexCharacter(char character);

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

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
        if (format[format_index] == '\0') {
            break;
        }
        (void)pushCharacter(&context, format[format_index], args);
    }

    // Null-terminate
    if (context.output.current_index < size) {
        context.output.buffer[context.output.current_index] = '\0';
    } else {
        context.output.buffer[size - 1U] = '\0';
    }

    return 0;
}

/**
 * Parse a string to a float
 *
 * @param string String to parse
 * @return Parsed float value
 */
float custom_strtof(const char string[]) {
    float final = 0.0F;

    uint8_t index = 0;
    uint8_t length = 0;
    if (issign(string[0])) {  //allow the first character to be a sign
        length = 1;
        index = 1;
    }
    while ((length < kFloatBufferSize) && ((string[length] == '.') || isnumber(string[length]))) {
        length++;
    }

    while ((index < length) && (string[index] != '.')) {
        final *= 10.0F;  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
        final += (float)(string[index] - '0');
        index++;
    }

    if (string[index] == '.') {
        index++;
    }

    const float divide_10 = 0.1F;
    float multiplier = divide_10;
    while (index < length) {
        final += ((float)(string[index] - '0') * multiplier);
        multiplier *= divide_10;
        index++;
    }

    if (string[0] == '-') {
        final = -final;
    }

    return final;
}

/**
 * Parse a string to an integer
 *
 * @param string String to parse
 * @return Parsed int value
 */
uint32_t custom_strtoi(const char string[]) {
    uint8_t length = 0;
    while ((length < kDecimalUint32Size) && isnumber(string[length])) {
        length++;
    }

    uint32_t value = 0;
    for (uint8_t index = 0; index < length; index++) {
        value *= 10U;  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
        value += (uint32_t)(string[index] - '0');
    }

    return value;
}

/**
 * Parse a string representing a hexadecimal number to an integer
 *
 * @param string String to parse
 * @return Parsed hex int value
 */
uint32_t custom_strtoi_hexa(const char string[]) {
    uint8_t length = 0;
    for (length = 0; length < kHexaUint32Size; length++) {
        if (!isnumber(string[length]) && !isAlphaUppercase(string[length]) && !isAlphaLowercase(string[length])) {
            break;
        }
    }

    if (!length) {
        return 0U;
    }

    uint32_t value = 0;
    for (uint8_t index = 0; index < length; index++) {
        value *= 16U;  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
        uint32_t digit_value = qualifyHexCharacter(string[index]);

        //check for overflow
        if (value > (UINT32_MAX - digit_value)) {
            return 0U;
        }

        value += digit_value;
    }

    return value;
}

/**
 * Get the lower case of an ascii character
 *
 * @param character Character to lower
 * @return Lower case value
 */
char toLowerAscii(char character) {
    if (isAlphaUppercase(character)) {
        return (char)(character + ('a' - 'A'));
    }

    return character;
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

/**
 * Get the number of characters in a string
 *
 * @param string String to check
 * @param max_length Maximum number of characters the string can hold
 * @return Number of characters, not counting '\0'
 */
size_t getStringLength(const char* string, size_t max_length) {
    if (!string) {
        return 0;
    }

    uint32_t length = 0U;
    while ((length < max_length) && (string[length] != '\0')) {
        length++;
    }

    return length;
}

/**
 * Compare two strings in a non case-sensitive manner
 *
 * @param first First string to compare
 * @param first_size Maximum size of the first string
 * @param second Second string to compare
 * @param second_size Maximum size of the second string
 * @retval Positive value if first alphabetically after second
 * @retval Null value if first equals second
 * @retval Negative value if first alphabetically before second 
 */
int8_t compareString(const char* first, size_t first_size, const char* second, size_t second_size) {
    size_t remaining = first_size;
    if (remaining > second_size) {
        remaining = second_size;
    }

    if (remaining == 0U) {
        return 0;
    }

    while ((remaining != 0U) && (*first != '\0') && (*second != '\0')) {
        const char folded_first = toLowerAscii(*first);
        const char folded_second = toLowerAscii(*second);

        if (folded_first != folded_second) {
            return (int8_t)((int16_t)folded_first - (int16_t)folded_second);
        }

        first++;
        second++;
        remaining--;
    }

    if (first_size > second_size) {
        return 1;
    }

    if (first_size < second_size) {
        return -1;
    }

    return 0;
}

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Get the integer value for a hexadecimal character
 *
 * @param character Character to translate
 * @return Integer value of the corresponding character
 */
static uint32_t qualifyHexCharacter(char character) {
    const uint8_t hexa_a_decimal_value = 10U;
    uint32_t digit_value = 0U;

    if (isnumber(character)) {
        digit_value = (uint32_t)(character - '0');
    } else if (isAlphaUppercase(character)) {
        digit_value = (uint32_t)(character - 'A') + hexa_a_decimal_value;
    } else if (isAlphaLowercase(character)) {
        digit_value = (uint32_t)(character - 'a') + hexa_a_decimal_value;
    }

    return digit_value;
}
