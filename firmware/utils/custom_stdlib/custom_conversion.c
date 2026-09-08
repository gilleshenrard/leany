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
    kMaxWidth = 128U,             ///< Maximum width specifier value
    kMaxDecimalsCharacters = 6U,  ///< Maximum number of decimals printed for a float
};

//utility functions
static void outputPadding(OutputBuffer* output, char pad_char, size_t count);
static char qualifySignCharacter(const ArgumentMetadata* metadata, bool is_negative);
static size_t getPaddingSize(const ArgumentMetadata* metadata, size_t output_len);
static uint32_t getFloatPrecision(const ConversionPrecision* precision);
static uint32_t getPowerOf10(uint32_t exponent);
static void trimUnsignedIntegerToLength(uint32_t* value, const ArgumentMetadata* metadata);

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Reset an argument's metadata
 *
 * @param metadata Metadata to reset
 */
void resetArgumentMetadata(ArgumentMetadata* metadata) {
    if (!metadata) {
        return;
    }

    *metadata = (ArgumentMetadata){
        .introductory_consumed = false,
        .zero_pad = false,
        .left_justify = false,
        .show_sign = false,
        .space_sign = false,
        .field_width = 0U,
        .prefix_length = 0U,
        .long_length_modifiers = 0U,
        .short_length_modifiers = 0U,
        .precision =
            {
                .decimal_char_consumed = false,
                .magnitude = 0U,
            },
    };
}

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
    if ((metadata->precision.magnitude) && (output_len > metadata->precision.magnitude)) {
        output_len = metadata->precision.magnitude;
    }

    // Calculate padding
    const size_t padding_size = getPaddingSize(metadata, output_len);
    size_t string_start_index = 0;
    size_t padding_start_index = 0;

    if (metadata->left_justify) {
        padding_start_index = output_len;
    } else {
        string_start_index = padding_size;
    }

    const size_t output_index_backup = output->current_index;

    output->current_index = output_index_backup + padding_start_index;
    outputPadding(output, ' ', padding_size);

    output->current_index = output_index_backup + string_start_index;

    for (size_t index = 0U; index < output_len; index++) {
        outputChar(output, str[index]);
    }

    output->current_index = output_index_backup + output_len + padding_size;
}

/**
 * Format and output a number with metadata
 * 
 * @param[out] output Output buffer metadata
 * @param result_string String buffer into which store the converted number
 * @param result_length Number of characters the raw output number takes
 * @param metadata Format metadata
 * @param is_negative Flag indicating whether the number is negative or not
 */
void outputNumber(OutputBuffer* output, const char* result_string, size_t result_length,
                  const ArgumentMetadata* metadata, bool is_negative) {
    if (!output || !result_string || !metadata) {
        return;
    }

    char sign_char = qualifySignCharacter(metadata, is_negative);
    const uint8_t sign_width = ((sign_char != '\0') ? 1 : 0);

    const size_t padding_size = getPaddingSize(metadata, (result_length + sign_width));

    size_t string_start_index = 0;
    size_t padding_start_index = 0;
    size_t sign_index = 0;

    if (metadata->left_justify) {
        string_start_index = sign_width;
        padding_start_index = result_length + string_start_index;
    } else if (metadata->zero_pad) {
        sign_index = 0;
        padding_start_index = sign_width;
        string_start_index = padding_size + sign_width;
    } else {
        sign_index = padding_size;
        string_start_index = padding_size + sign_width;
    }

    const size_t output_index_backup = output->current_index;

    if (sign_char != '\0') {
        output->current_index = output_index_backup + sign_index;
        outputChar(output, sign_char);
    }

    output->current_index = output_index_backup + padding_start_index;
    const char padding_character = (char)((char)(metadata->zero_pad) ? '0' : ' ');
    outputPadding(output, padding_character, padding_size);

    output->current_index = output_index_backup + string_start_index;
    for (size_t index = 0U; index < result_length; index++) {
        outputChar(output, result_string[index]);
    }

    output->current_index = output_index_backup + sign_width + result_length + padding_size;
}

/**
 * Convert unsigned integer to string
 * 
 * @param value Value to convert
 * @param[out] buffer Buffer to store result
 * @param radix Numeric base (2, 8, 10, or 16)
 * @param uppercase Use uppercase for hex digits
 * @param metadata Metadata of the current argument
 * @return Length of converted string
 */
uint32_t convertUnsigned(uint32_t value, char* buffer, uint32_t radix, bool uppercase,
                         const ArgumentMetadata* metadata) {
    if (!buffer) {
        return 0U;
    }

    const char* digits_lower = "0123456789abcdef";
    const char* digits_upper = "0123456789ABCDEF";
    const char* digits = ((uint8_t)uppercase ? digits_upper : digits_lower);

    //Handle 0 value
    if (value == 0U) {
        buffer[0] = '0';
        buffer[1] = '\0';
        return 1U;
    }

    trimUnsignedIntegerToLength(&value, metadata);

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
 * @param metadata Metadata of the current argument
 * @return Length of converted string (excluding sign)
 */
uint32_t convertSigned(int32_t value, char* buffer, uint32_t radix, const ArgumentMetadata* metadata) {
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

    uint32_t length = convertUnsigned(unsigned_value, buffer, radix, false, metadata);
    return length;
}

/**
 * Convert a float to string
 *
 * @param value Value to convert
 * @param[out] buffer Buffer to store result
 * @param precision Number of characters after the decimal point
 * @param metadata Metadata of the current argument
 * @return Length of converted float (excluding sign)
 */
uint32_t convertFloat(float value, char* buffer, const ConversionPrecision* precision,
                      const ArgumentMetadata* metadata) {
    constexpr uint8_t decimal_radix = 10U;

    const uint32_t precision_magnitude = getFloatPrecision(precision);
    uint64_t precise_multiplier = getPowerOf10(precision_magnitude);
    int64_t precise_value = (int64_t)(value * (float)precise_multiplier);

    // convert integer part to string
    uint32_t buffer_index =
        convertSigned((int32_t)(precise_value / (int64_t)precise_multiplier), buffer, decimal_radix, metadata);

    if (precision_magnitude == 0) {
        return buffer_index;
    }

    //add '.'
    buffer[buffer_index] = '.';
    buffer_index++;

    //isolate the decimal part as an absolute value
    uint64_t decimal_part = (uint64_t)((precise_value >= 0) ? precise_value : -precise_value);
    decimal_part %= precise_multiplier;

    //push as many leading 0 as needed to the decimal part
    precise_multiplier /= decimal_radix;
    while (precise_multiplier > decimal_part) {
        buffer[buffer_index] = '0';
        buffer_index++;
        precise_multiplier /= decimal_radix;
    }

    //if required, convert the decimal part to string
    if (decimal_part) {
        buffer_index += convertUnsigned((uint32_t)decimal_part, &buffer[buffer_index], decimal_radix, false, metadata);
    }

    return buffer_index;
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
    if (output_len >= metadata->field_width) {
        return 0;
    }

    return (metadata->field_width - output_len);
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

/**
 * Decide which precision should be printed on a float
 *
 * @param precision Precision metadata extracted from the argument modifiers
 * @return Final magnitude of the precision
 */
static uint32_t getFloatPrecision(const ConversionPrecision* precision) {
    if (!precision) {
        return 0;
    }

    if (!precision->decimal_char_consumed || (precision->magnitude > kMaxDecimalsCharacters)) {
        return kMaxDecimalsCharacters;
    }

    return precision->magnitude;
}

/**
 * Compute a power of 10
 *
 * @param exponent Exponent of the power of 10
 * @return 10^exponent
 */
static uint32_t getPowerOf10(const uint32_t exponent) {
    constexpr uint8_t multiplier10 = 10U;

    uint32_t multiplier = 1U;
    for (uint32_t rank = 0; rank < exponent; rank++) {
        multiplier *= multiplier10;
    }

    return multiplier;
}

/**
 * Reset bytes of a 32-bits value depending on its length modifiers
 *
 * @param value Value to trim
 * @param metadata Argument metadata
 */
static void trimUnsignedIntegerToLength(uint32_t* value, const ArgumentMetadata* metadata) {
    const uint8_t bitwise_modifiers =
        ((uint8_t)(metadata->long_length_modifiers << 2U) | metadata->short_length_modifiers);

    // NOLINTBEGIN (cppcoreguidelines-avoid-magic-numbers)
    switch (bitwise_modifiers) {
        case 1:  //'h'
            *value &= 0x0000FFFFU;
            break;

        case 2:  //'hh'
            *value &= 0x000000FFU;
            break;

        case 4:  //'l'
        case 8:  //'ll'
            *value &= 0xFFFFFFFFU;
            break;

        case 0:   //no modifier
        default:  // invalid combination
            break;
    }
    // NOLINTEND (cppcoreguidelines-avoid-magic-numbers)
}
