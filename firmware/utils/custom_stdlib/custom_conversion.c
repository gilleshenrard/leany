/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 *
 * @file custom_conversion.c
 * @brief Implement parameters to string conversion
 */
#include "custom_conversion.h"

#include <stddef.h>
#include <stdint.h>

#include "custom_string.h"
#include "custom_stringparser.h"

enum : uint8_t {
    kMaxWidth = 128U,  ///< Maximum width specifier value
};

//utility functions
static void outputPadding(OutputBuffer* output, char pad_char, size_t count);
static char qualifySignCharacter(const ArgumentMetadata* metadata, bool is_negative);
static size_t getPaddingSize(const ArgumentMetadata* metadata, size_t output_len);

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Output a single character to buffer with bounds checking
 * 
 * @param[out] output Output buffer metadata
 * @param character Character to write
 */
void outputChar(OutputBuffer* output, char character) {
    if (!output || (output->current_index >= output->buffer_size)) {
        return;
    }

    output->buffer[output->current_index] = character;
    output->current_index++;
}

/**
 * Format and output a string with metadata
 * 
 * @param[out] output Output buffer metadata
 * @param str String to output
 * @param metadata Format metadata
 */
void outputString(OutputBuffer* output, const char* str, const ArgumentMetadata* metadata) {
    if (!output || !str || !metadata) {
        return;
    }

    size_t output_len = getStringLength(str, kMaxFormatLength);
    const size_t padding_size = getPaddingSize(metadata, output_len);

    if (!metadata->left_justify) {
        outputPadding(output, (char)((char)metadata->zero_pad ? '0' : ' '), padding_size);
    }

    // Output string
    for (uint32_t index = 0U; index < output_len; index++) {
        outputChar(output, str[index]);
    }

    if (metadata->left_justify) {
        outputPadding(output, ' ', padding_size);
    }
}

/**
 * Format and output an integer with metadata
 * 
 * @param[out] output Output buffer metadata
 * @param result_string String buffer into which store the converted integer
 * @param result_length Number of characters the raw output integer takes
 * @param metadata Format metadata
 * @param is_negative Flag indicating whether the number is negative or not
 */
void outputInteger(OutputBuffer* output, const char* result_string, size_t result_length,
                   const ArgumentMetadata* metadata, bool is_negative) {
    if (!output || !result_string || !result_length || !metadata) {
        return;
    }

    char sign_char = qualifySignCharacter(metadata, is_negative);

    size_t total_len = result_length;
    if (sign_char != '\0') {
        total_len++;
    }

    // Calculate padding
    const size_t padding_size = getPaddingSize(metadata, total_len);
    size_t string_start_index = 0;
    size_t padding_start_index = 0;
    char padding_char = ' ';

    if (metadata->left_justify) {
        padding_start_index = total_len;
    } else {
        string_start_index = padding_size;
        if (metadata->zero_pad) {
            padding_char = '0';
        }
    }

    /* Output sign */
    if (sign_char != '\0') {
        outputChar(output, sign_char);
    }

    const size_t output_index_backup = output->current_index;

    output->current_index = output_index_backup + padding_start_index;
    outputPadding(output, padding_char, padding_size);

    output->current_index = output_index_backup + string_start_index;

    for (size_t index = 0U; index < result_length; index++) {
        outputChar(output, result_string[index]);
    }

    output->current_index = output_index_backup + total_len + padding_size;
}

/**
 * Convert unsigned integer to string
 * 
 * @param value Value to convert
 * @param[out] buffer Buffer to store result
 * @param radix Numeric base (2, 8, 10, or 16)
 * @param uppercase Use uppercase for hex digits
 * @return Length of converted string
 */
uint32_t convertUnsigned(uint32_t value, char* buffer, uint32_t radix, uint8_t uppercase) {
    if (!buffer) {
        return 0U;
    }

    const char* digits_lower = "0123456789abcdef";
    const char* digits_upper = "0123456789ABCDEF";
    const char* digits = (uppercase ? digits_upper : digits_lower);

    //Handle 0 value
    if (value == 0U) {
        buffer[0] = '0';
        buffer[1] = '\0';
        return 1U;
    }

    uint32_t length = 0U;

    // Convert to string in reverse
    char temp[kMaxIntBuffer];
    while ((length < (kMaxIntBuffer - 1U)) && value) {
        temp[length] = digits[value % radix];
        value /= radix;
        length++;
    }

    // Reverse the string
    for (uint32_t index = 0U; index < length; index++) {
        buffer[index] = temp[length - 1U - index];
    }

    buffer[length] = '\0';
    return length;
}

/**
 * Convert signed integer to string
 * 
 * @param value Value to convert
 * @param[out] buffer Buffer to store result
 * @param radix Numeric base (typically 10)
 * @return Length of converted string (excluding sign)
 */
uint32_t convertSigned(int32_t value, char* buffer, uint32_t radix) {
    if (!buffer) {
        return 0U;
    }

    uint32_t unsigned_value = 0;
    if (value < 0) {
        // Handle INT32_MIN edge case
        unsigned_value = ((value == INT32_MIN) ? ((uint32_t)INT32_MAX + 1U) : (uint32_t)(-value));
    } else {
        unsigned_value = (uint32_t)value;
    }

    uint32_t length = convertUnsigned(unsigned_value, buffer, radix, 0);
    return length;
}

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Get the number of characters the padding spans
 *
 * @param metadata Prefix metadata
 * @param output_len Number of characters the raw output takes
 * @return Number of characters in the padding
 */
static size_t getPaddingSize(const ArgumentMetadata* metadata, size_t output_len) {
    const size_t left_padding = ((metadata->width > output_len) ? (metadata->width - output_len) : 0U);
    return left_padding;
}

/**
 * Output padding characters
 * 
 * @param[out] output Output buffer metadata
 * @param pad_char Character to use for padding
 * @param count Number of padding characters
 */
static void outputPadding(OutputBuffer* output, char pad_char, size_t count) {
    if (!output) {
        return;
    }

    const size_t padding_limit = ((count < kMaxWidth) ? count : kMaxWidth);
    for (uint32_t padding_index = 0U; padding_index < padding_limit; padding_index++) {
        outputChar(output, pad_char);
    }
}

/**
 * Parse a sign character
 *
 * @param metadata Prefix metadata to qualify
 * @param is_negative Flag indicating whether the sign is negative
 * @return Corresponding character
 */
static char qualifySignCharacter(const ArgumentMetadata* metadata, bool is_negative) {
    if (!metadata) {
        return '\0';
    }

    if (is_negative) {
        return '-';
    }

    if (metadata->show_sign) {
        return '+';
    }

    if (metadata->space_sign) {
        return ' ';
    }

    return '\0';
}
