/*
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file leany_std.c
 * @brief Lightweight standard functions implementation for embedded systems
 */

#include "leany_std.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum {
    kMaxPrefixLength = 8U,                       ///< Maximum format prefix length to prevent infinite loops
    kMaxFormatLength = 256U - kMaxPrefixLength,  ///< Maximum format string length to prevent infinite loops
    kMaxIntBuffer = 65U,                         ///< Maximum integer conversion buffer size (64-bit in binary + sign)
    kMaxWidth = 128U,                            ///< Maximum width specifier value
};

/**
 * @brief Format flags parsed from format specifier
 */
typedef struct {
    bool zero_pad;      /**< Use zero padding instead of spaces */
    bool left_justify;  /**< Left justify the output */
    bool show_sign;     /**< Always show sign for signed numbers */
    bool space_sign;    /**< Use space for positive numbers */
    uint32_t width;     /**< Minimum field width */
    uint32_t precision; /**< Precision for strings/numbers */
    bool has_precision; /**< Whether precision was specified */
} FormatFlags;

/**
 * Structure used to lower the number of arguments to outputInteger()
 */
typedef struct {
    const char* input_string;  ///< String which will be copied
    uint32_t input_length;     ///< Length of the input string
    bool is_negative;          ///< Whether the number is negative
} IntegerInput;

/**
 * Output buffer into which perform the formatting
 */
typedef struct {
    char* buffer;          ///< Buffer into which store the formatted output
    size_t buffer_size;    ///< Size of the buffer
    size_t current_index;  ///< Current index in the buffer
} OutputBuffer;

static void outputChar(OutputBuffer* output, char character);
static void outputPadding(OutputBuffer* output, char pad_char, uint32_t count);
static uint32_t convertUnsigned(uint32_t value, char* buffer, uint32_t base, bool uppercase);
static uint32_t convertSigned(int32_t value, char* buffer, uint32_t base, bool* is_negative);
static uint32_t parseFlags(const char* format, FormatFlags* flags);
static uint8_t qualifyFormatPrefix(const char* format, FormatFlags* flags);
static void outputInteger(OutputBuffer* output, const IntegerInput* input, const FormatFlags* flags);
static char qualifySignCharacter(const FormatFlags* flags, bool is_negative);
static void outputString(OutputBuffer* output, const char* str, const FormatFlags* flags);
static void qualifyConversionSpecifier(char specifier, OutputBuffer* output, const FormatFlags* flags, va_list* args);
static uint32_t qualifyHexCharacter(char character);
static uint32_t qualifyLengthModifier(const char format[], uint32_t* length);
static void applyStringJustification(OutputBuffer* output, const FormatFlags* flags, uint32_t output_len);

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

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
int32_t leany_vsnprintf(char* buffer, size_t size, const char* format, va_list args) {
    /* Validate parameters */
    if (!buffer || !format || !size) {
        return -1;
    }

    OutputBuffer output = {buffer, size, 0};

    /* Process format string */
    uint32_t format_index = 0U;
    for (format_index = 0U; format_index < kMaxFormatLength; format_index++) {
        if (format[format_index] == '\0') {
            break;
        }

        if (format[format_index] != '%') {
            outputChar(&output, format[format_index]);
            continue;
        }

        // Skip '%'
        format_index++;

        // Handle %%
        if (format[format_index] == '%') {
            outputChar(&output, '%');
            continue;
        }

        // Parse flags and conversion specifier
        FormatFlags flags;
        format_index += parseFlags(&format[format_index], &flags);
        qualifyConversionSpecifier(format[format_index], &output, &flags, &args);
    }

    // Null-terminate
    if (output.current_index < size) {
        output.buffer[output.current_index] = '\0';
    } else {
        output.buffer[size - 1U] = '\0';
    }

    return (int32_t)output.current_index;
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
int32_t leany_snprintf(char* buffer, size_t size, const char* format, ...) {
    va_list args;

    va_start(args, format);
    int32_t result = leany_vsnprintf(buffer, size, format, args);
    va_end(args);

    return result;
}

/**
 * Parse a float to a string
 *
 * @param value Value to parse 
 * @param[out] out_buffer Buffer in which store the parsed string
 * @param buffer_size Maximum number of characters in the buffer
 * @param precision Number of decimals to parse after the separator
 */
// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
void floatToString(float value, char out_buffer[], uint8_t buffer_size, uint8_t precision) {
    // Extract integer part
    uint16_t ipart = (uint16_t)value;

    // Extract floating part
    float fpart = value - (float)ipart;

    // convert integer part to string
    //NOLINTNEXTLINE(clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
    const uint8_t written = (uint8_t)leany_snprintf(out_buffer, (buffer_size - 2U), "%u", ipart);
    out_buffer[written] = '.';

    if (!precision) {
        out_buffer[written + 1] = '0';
        out_buffer[written + 2] = '\0';
        return;
    }

    while (precision--) {
        fpart *= 10.0F;  // NOLINT (cppcoreguidelines-avoid-magic-numbers)
    }
    //NOLINTNEXTLINE(clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
    (void)leany_snprintf(&out_buffer[written + 1], (buffer_size - written - 1U), "%u", (uint16_t)fpart);
}

/**
 * Parse an int to a string
 *
 * @param value Value to parse 
 * @param[out] out_buffer Buffer in which store the parsed string
 * @param buffer_size Maximum number of characters in the buffer
 */
void intToString(uint32_t value, char out_buffer[], uint8_t buffer_size) {
    if (!out_buffer || !buffer_size) {
        return;
    }

    const char ascii_digit_offset = '0';
    const uint8_t decimal_base = 10U;

    if (value == 0U) {
        out_buffer[0] = ascii_digit_offset;
        out_buffer[1] = '\n';
        return;
    }

    // Calculate the correct magnitude (highest power of 10 <= value)
    uint32_t magnitude = 1U;
    uint32_t temp_value = value;

    while (temp_value >= decimal_base) {
        temp_value /= decimal_base;
        magnitude *= decimal_base;
    }

    // Convert digits from most significant to least significant
    uint8_t index = 0U;
    while ((index < buffer_size - 1U) && (magnitude > 0U)) {
        uint8_t digit = (uint8_t)(value / magnitude);
        out_buffer[index] = ascii_digit_offset + (char)digit;
        value %= magnitude;
        magnitude /= decimal_base;
        index++;
    }

    out_buffer[index] = '\0';
}

/**
 * Parse a string to a float
 *
 * @param string String to parse
 * @return Parsed float value
 */
float stringToFloat(const char string[]) {
    float final = 0.0F;

    uint8_t index = 0;
    uint8_t length = 0;
    if (issign(string[0])) {  //allow the first character to be a sign
        length = 1;
        index = 1;
    }
    while ((length < (uint8_t)kFloatBufferSize) && ((string[length] == '.') || isnumber(string[length]))) {
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
uint32_t stringToInt(const char string[]) {
    uint8_t length = 0;
    while ((length < (uint8_t)kFloatBufferSize) && isnumber(string[length])) {
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
uint32_t stringHexToInt(const char string[]) {
    uint8_t length = 0;
    for (length = 0; length < (uint8_t)kHexaUint32Size; length++) {
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
uint8_t toLowerAscii(uint8_t character) {
    if (isAlphaUppercase(character)) {
        return (uint8_t)(character + ((uint8_t)'a' - (uint8_t)'A'));
    }

    return character;
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
    const uint8_t* p_first = (const uint8_t*)first;
    const uint8_t* p_second = (const uint8_t*)second;

    size_t remaining = first_size;
    if (remaining > second_size) {
        remaining = second_size;
    }

    if (remaining == 0U) {
        return 0;
    }

    while ((remaining != 0U) && (*p_first != (uint8_t)'\0') && (*p_second != (uint8_t)'\0')) {
        const uint8_t folded_first = toLowerAscii(*p_first);
        const uint8_t folded_second = toLowerAscii(*p_second);

        if (folded_first != folded_second) {
            return (int8_t)((int16_t)folded_first - (int16_t)folded_second);
        }

        p_first++;
        p_second++;
        remaining--;
    }

    if (remaining == 0U) {
        return 0;
    }

    return (int8_t)((int16_t)toLowerAscii(*p_first) - (int16_t)toLowerAscii(*p_second));
}

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

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Output a single character to buffer with bounds checking
 * 
 * @param[out] output Output buffer metadata
 * @param character Character to write
 */
static void outputChar(OutputBuffer* output, char character) {
    if (!output || (output->current_index >= output->buffer_size)) {
        return;
    }

    output->buffer[output->current_index] = character;
    output->current_index++;
}

/**
 * Output padding characters
 * 
 * @param[out] output Output buffer metadata
 * @param pad_char Character to use for padding
 * @param count Number of padding characters
 */
static void outputPadding(OutputBuffer* output, char pad_char, uint32_t count) {
    if (!output) {
        return;
    }

    const uint32_t padding_limit = ((count < kMaxWidth) ? count : kMaxWidth);
    for (uint32_t padding_index = 0U; padding_index < padding_limit; padding_index++) {
        outputChar(output, pad_char);
    }
}

/**
 * Convert unsigned integer to string
 * 
 * @param value Value to convert
 * @param[out] buffer Buffer to store result
 * @param base Numeric base (2, 8, 10, or 16)
 * @param uppercase Use uppercase for hex digits
 * @return Length of converted string
 */
static uint32_t convertUnsigned(uint32_t value, char* buffer, uint32_t base, bool uppercase) {
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
        temp[length] = digits[value % base];
        value /= base;
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
 * @param base Numeric base (typically 10)
 * @param[out] is_negative Set to true if value was negative
 * @return Length of converted string (excluding sign)
 */
static uint32_t convertSigned(int32_t value, char* buffer, uint32_t base, bool* is_negative) {
    if (!buffer || !is_negative) {
        return 0U;
    }

    *is_negative = ((value < 0) ? true : false);

    uint32_t unsigned_value = 0;
    if (value < 0) {
        // Handle INT32_MIN edge case
        unsigned_value = ((value == INT32_MIN) ? ((uint32_t)INT32_MAX + 1U) : (uint32_t)(-value));
    } else {
        unsigned_value = (uint32_t)value;
    }

    uint32_t length = convertUnsigned(unsigned_value, buffer, base, false);
    return length;
}

/**
 * Parse format flags from format string
 * 
 * @param format Pointer to format string (at flags position)
 * @param[out] flags Parsed flags structure
 * @return Number of characters consumed
 */
static uint32_t parseFlags(const char* format, FormatFlags* flags) {
    if (!format || !flags) {
        return 0U;
    }

    uint32_t length = qualifyFormatPrefix(format, flags);
    length += qualifyLengthModifier(&format[length], &flags->width);

    if (format[length] != '.') {
        return length;
    }

    length += qualifyLengthModifier(&format[length], &flags->precision);

    return length;
}

/**
 * Parse a format's prefix characters
 * @details The prefix comprises all modifier characters coming before any length or type specifier
 *
 * @param format Format to parse
 * @param flags Prefix flags to qualify
 * @return Length of the prefix
 */
static uint8_t qualifyFormatPrefix(const char* format, FormatFlags* flags) {
    if (!format || !flags) {
        return 0U;
    }

    bool parsing = true;
    uint8_t length = 0;

    // Initialize flags
    *flags = (FormatFlags){
        .zero_pad = false,
        .left_justify = false,
        .show_sign = false,
        .space_sign = false,
        .width = 0U,
        .precision = 0U,
        .has_precision = false,
    };

    // Qualify flag characters
    while (parsing && (length < (uint8_t)kMaxPrefixLength)) {
        switch (format[length]) {
            case '0':
                flags->zero_pad = true;
                length++;
                break;

            case '-':
                flags->left_justify = true;
                length++;
                break;

            case '+':
                flags->show_sign = true;
                length++;
                break;

            case ' ':
                flags->space_sign = true;
                length++;
                break;

            default:
                parsing = false;
                break;
        }
    }

    return length;
}

/**
 * Format and output an integer with flags
 * 
 * @param[out] output Output buffer metadata
 * @param input Characteristics of the input number
 * @param flags Format flags
 */
static void outputInteger(OutputBuffer* output, const IntegerInput* input, const FormatFlags* flags) {
    if (!output || !input || !flags) {
        return;
    }

    char sign_char = qualifySignCharacter(flags, input->is_negative);

    uint32_t total_len = input->input_length;
    if (sign_char != '\0') {
        total_len++;
    }

    // Calculate padding
    const uint32_t padding = ((flags->width > total_len) ? (flags->width - total_len) : 0U);

    /* Output left padding (if not zero-pad or left-justify) */
    if (!flags->left_justify && !flags->zero_pad) {
        outputPadding(output, ' ', padding);
    }

    /* Output sign */
    if (sign_char != '\0') {
        outputChar(output, sign_char);
    }

    outputString(output, input->input_string, flags);
}

/**
 * Parse a sign character
 *
 * @param flags Prefix flags to qualify
 * @param is_negative Flag indicating whether the sign is negative
 * @return Corresponding character
 */
static char qualifySignCharacter(const FormatFlags* flags, bool is_negative) {
    if (!flags) {
        return '\0';
    }

    if (is_negative) {
        return '-';
    }

    if (flags->show_sign) {
        return '+';
    }

    if (flags->space_sign) {
        return ' ';
    }

    return '\0';
}

/**
 * Format and output a string with flags
 * 
 * @param[out] output Output buffer metadata
 * @param str String to output
 * @param flags Format flags
 */
static void outputString(OutputBuffer* output, const char* str, const FormatFlags* flags) {
    if (!output || !str || !flags) {
        return;
    }

    uint32_t output_len = getStringLength(str, kMaxFormatLength);

    // Apply precision limit
    if (flags->has_precision && (flags->precision < output_len)) {
        output_len = flags->precision;
    }

    applyStringJustification(output, flags, output_len);

    // Output string
    for (uint32_t index = 0U; index < output_len; index++) {
        outputChar(output, str[index]);
    }
}

/**
 * Qualify a conversion specifier and its arguments
 * 
 * @param specifier Specifier (e.g. 'd' for an integer)
 * @param[out] output Output buffer metadata
 * @param flags Modifier flags (e.g. '+' to force adding the sign)
 * @param args Variable list of arguments
 */
static void qualifyConversionSpecifier(char specifier, OutputBuffer* output, const FormatFlags* flags, va_list* args) {
    const uint8_t decimal_radix = 10U;
    const uint8_t hexa_radix = 16U;
    char num_buffer[kMaxIntBuffer];
    const bool is_uppercase_hex = (specifier == 'X');

    IntegerInput input = {.input_string = num_buffer, .is_negative = false};
    switch (specifier) {
        case 'd':
        case 'i':
            input.input_length = convertSigned(va_arg(*args, int32_t), num_buffer, decimal_radix, &input.is_negative);
            outputInteger(output, &input, flags);
            break;

        case 'u':
            input.input_length = convertUnsigned(va_arg(*args, uint32_t), num_buffer, decimal_radix, false);
            outputInteger(output, &input, flags);
            break;

        case 'X':
        case 'x':
            input.input_length = convertUnsigned(va_arg(*args, uint32_t), num_buffer, hexa_radix, is_uppercase_hex);
            outputInteger(output, &input, flags);
            break;

        case 's':
            outputString(output, va_arg(*args, char*), flags);
            break;

        case 'c':
            outputChar(output, (char)va_arg(*args, int));
            break;

        case 'p':
            /* Pointer as hex with 0x prefix */
            outputChar(output, '0');
            outputChar(output, 'x');
            input.input_length =
                convertUnsigned((uint32_t)(uintptr_t)va_arg(*args, void*), num_buffer, hexa_radix, false);
            outputInteger(output, &input, flags);
            break;

        default:
            /* Unknown specifier - output as-is */
            outputChar(output, '%');
            outputChar(output, specifier);
            break;
    }
}

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

/**
 * Qualify a sprintf argument length modifier
 *
 * @param format Format of the modifier
 * @param[out] length Number of characters the argument will be formatted in
 * @return Number of characters of the length modifier
 */
static uint32_t qualifyLengthModifier(const char format[], uint32_t* length) {
    const uint8_t multiplier_10 = 10U;

    uint8_t index = 0;
    while ((format[index] >= '0') && (format[index] <= '9') && (index < kMaxFormatLength)) {
        *length = (*length * multiplier_10) + (uint32_t)(format[index] - '0');
        if (*length > kMaxWidth) {
            *length = kMaxWidth;
        }
        index++;
    }

    return index;
}

/**
 * Apply string justificaton (left, right, padding)
 *
 * @param[out] output Output buffer metadata
 * @param flags Modifier flags
 * @param output_len Length of the string to justify in the buffer
 */
static void applyStringJustification(OutputBuffer* output, const FormatFlags* flags, uint32_t output_len) {
    // Apply precision limit
    if (flags->has_precision && (flags->precision < output_len)) {
        output_len = flags->precision;
    }

    // Calculate padding
    const uint32_t padding = ((flags->width > output_len) ? (flags->width - output_len) : 0U);

    /* Output zero padding (after sign) */
    if (!flags->left_justify) {
        const char pad_character = (flags->zero_pad ? '0' : ' ');
        outputPadding(output, pad_character, padding);
    }

    const size_t restore_index = output->current_index;

    /* Output right padding */
    if (flags->left_justify) {
        outputPadding(output, ' ', padding);
    }

    output->current_index = restore_index;
}
