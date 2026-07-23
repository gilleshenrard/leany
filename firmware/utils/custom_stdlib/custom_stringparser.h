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
    kParserPending = 0,       ///< Parser is still pending for new characters
    kParserSkipArgument = 1,  ///< The current argument can be considered as parsed and can be skipped
    kParserReevaluate = 2,    ///< The parser needs to re-evaluate the last character
    kParserInvalid = 3,       ///< Parser received an invalid input
    kParserDone = 4,          ///< Parser is done
} ParserResult;

/**
 * States of the parser
 */
typedef enum : uint8_t {
    kStateCopying = 1,                ///< State during which characters pushed are not parameters or modifiers
    kStateParsingPrefix = 2,          ///< State during which the argument prefix (length, padding, ...) is parsed
    kStateParsingLengthModifier = 3,  ///< State during which the length modifier is parsed
} ParserState;

/**
 * Output buffer into which perform the formatting
 */
typedef struct {
    char* buffer;          ///< Buffer into which store the formatted output
    size_t buffer_size;    ///< Size of the buffer
    size_t current_index;  ///< Current index in the buffer
} OutputBuffer;

/**
 * @brief Format metadata parsed from format specifier
 */
typedef struct {
    bool introductory_consumed;  ///< Whether the percent character has already been consumed
    bool zero_pad;               ///< Use zero padding instead of spaces
    bool left_justify;           ///< Left justify the output
    bool show_sign;              ///< Always show sign for signed numbers
    bool space_sign;             ///< Use space for positive numbers
    uint32_t width;              ///< Minimum field width
    uint8_t prefix_length;       ///< Argument's prefix length
} ArgumentMetadata;

typedef struct {
    ParserState state;
    OutputBuffer output;
    ArgumentMetadata current_argument;
} ParserContext;

bool initialiseContext(ParserContext* context, char* output_buffer, size_t output_size);
ParserResult pushCharacter(ParserContext* context, char input_character, va_list args);

#endif
