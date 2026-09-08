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

#include "custom_conversion.h"

enum : uint8_t {
    kMaxPrefixLength = 8U,                       ///< Maximum format prefix length to prevent infinite loops
    kMaxFormatLength = 256U - kMaxPrefixLength,  ///< Maximum format string length to prevent infinite loops
    kMaxIntBuffer = 65U,                         ///< Maximum integer conversion buffer size (64-bit in binary + sign)
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
    kStateCopying = 0,                     ///< State during which characters pushed are not parameters or modifiers
    kStateParsingPrefix = 1,               ///< State during which the argument prefix (length, padding, ...) is parsed
    kStateParsingFieldWidth = 2,           ///< State during which the field width is parsed
    kStateParsingLengthModifiers = 3,      ///< State during which the length modifiers are parsed
    kStateParsingConversionSpecifier = 4,  ///< State during which the conversion specifier is parsed
} ParserState;

/**
 * String parser context
 */
typedef struct {
    ParserState state;                  ///< Current parser state
    OutputBuffer output;                ///< Output metadata
    ArgumentMetadata current_argument;  ///< Current argument metadata
} ParserContext;

bool initialiseContext(ParserContext* context, char* output_buffer, size_t output_size);
ParserResult pushCharacter(ParserContext* context, char input_character, va_list* args);

#endif
