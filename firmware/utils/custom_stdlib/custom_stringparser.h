/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 *
 * @file custom_stringparser.h
 * @brief Lightweight parametric string parser
 */

#ifndef UTILS_CUSTOM_STDLIB_CUSTOM_STRINGPARSER_H
#define UTILS_CUSTOM_STDLIB_CUSTOM_STRINGPARSER_H
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

enum : uint8_t {
    kMaxPrefixLength = 8U,                       ///< Maximum format prefix length to prevent infinite loops
    kMaxFormatLength = 256U - kMaxPrefixLength,  ///< Maximum format string length to prevent infinite loops
    kMaxIntBuffer = 65U,                         ///< Maximum integer conversion buffer size (64-bit in binary + sign)
    kMaxWidth = 128U,                            ///< Maximum width specifier value
};

/**
 * Result of the last parser round
 */
typedef enum : uint8_t {
    kParserPending = 0,  ///< Parser is still pending for new characters
    kParserInvalid = 1,  ///< Parser received an invalid input
    kParserDone = 2,     ///< Parser is done
} ParserResult;

/**
 * Output buffer into which perform the formatting
 */
typedef struct {
    char* buffer;          ///< Buffer into which store the formatted output
    size_t buffer_size;    ///< Size of the buffer
    size_t current_index;  ///< Current index in the buffer
} OutputBuffer;

ParserResult pushCharacter(OutputBuffer* output_string, char input_character, va_list args);

#endif
