/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 *
 * @file custom_conversion.h
 */
#ifndef UTILS_CUSTOM_STDLIB_CUSTOM_CONVERSION_H
#define UTILS_CUSTOM_STDLIB_CUSTOM_CONVERSION_H
#include <stddef.h>
#include <stdint.h>

/**
 * Output buffer into which perform the formatting
 */
typedef struct {
    char* buffer;          ///< Buffer into which store the formatted output
    size_t buffer_size;    ///< Size of the buffer
    size_t current_index;  ///< Current index in the buffer
} OutputBuffer;

/**
 * Metadata specific to whether precision is required in the argument's conversion
 */
typedef struct {
    bool decimal_char_consumed;  ///< Whether the argument as a decimal character already consumed
    uint32_t magnitude;          ///< Minimum number of characters to hit the precision
} ConversionPrecision;

/**
 * Argument format metadata parsed from format specifier
 */
typedef struct {
    bool introductory_consumed;     ///< Whether the percent character has already been consumed
    bool zero_pad;                  ///< Use zero padding instead of spaces
    bool left_justify;              ///< Left justify the output
    bool show_sign;                 ///< Always show sign for signed numbers
    bool space_sign;                ///< Use space for positive numbers
    uint32_t width;                 ///< Minimum field width
    uint8_t prefix_length;          ///< Argument's prefix length
    ConversionPrecision precision;  ///< Arguments' precision metadata
} ArgumentMetadata;

void outputChar(OutputBuffer* output, char character);
void outputString(OutputBuffer* output, const char* str, const ArgumentMetadata* metadata);
void outputFloat(OutputBuffer* output, const char* result_string, size_t result_length,
                 const ArgumentMetadata* metadata, bool is_negative);
uint32_t convertUnsigned(uint32_t value, char* buffer, uint32_t radix, bool uppercase);
uint32_t convertSigned(int32_t value, char* buffer, uint32_t radix);
void outputInteger(OutputBuffer* output, const char* result_string, size_t result_length,
                   const ArgumentMetadata* metadata, bool is_negative);
uint32_t convertFloat(float value, char* buffer, uint32_t precision);

#endif
