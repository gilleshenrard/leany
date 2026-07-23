/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 *
 * @file custom_stringparser.c
 * @brief Lightweight parametric string parser
 */
#include "custom_stringparser.h"

#include <stdarg.h>
#include <stdint.h>

#include "custom_string.h"

/**
 * States of the parser
 */
typedef enum : uint8_t {
    kStateCopying = 1,                ///< State during which characters pushed are not parameters or modifiers
    kStateParsingPrefix = 2,          ///< State during which the argument prefix (length, padding, ...) is parsed
    kStateParsingLengthModifier = 3,  ///< State during which the length modifier is parsed
} MachineState;

/**
 * @brief Format metadata parsed from format specifier
 */
typedef struct {
    bool zero_pad;      ///< Use zero padding instead of spaces
    bool left_justify;  ///< Left justify the output
    bool show_sign;     ///< Always show sign for signed numbers
    bool space_sign;    ///< Use space for positive numbers
    uint32_t width;     ///< Minimum field width
} ArgumentMetadata;

static ParserResult stateCopyingCharacters(OutputBuffer* output_string, char input_character);
static ParserResult stateParsingArgumentPrefix(OutputBuffer* output_string, char input_character,
                                               ArgumentMetadata* metadata);
static ParserResult stateParsingLengthModifier(OutputBuffer* output_string, char input_character,
                                               ArgumentMetadata* metadata);

static constexpr char kIntroductoryCharacter = '%';  ///< Character indicating the introduction of a formatted parameter

static MachineState current_state = kStateCopying;  ///< Current parser machine state
static ArgumentMetadata current_metadata;           ///< Current argument metadata

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Push an input character to be treated in the parser's state machine as an output parameter

 * @param output_string Metadata of the output buffer state
 * @param input_character Next character to treat
 * @return 0
 */
ParserResult pushCharacter(OutputBuffer* output_string, char input_character, va_list args) {
    if (!output_string) {
        return kParserInvalid;
    }

    switch (current_state) {
        case kStateCopying:
            return stateCopyingCharacters(output_string, input_character);

        case kStateParsingPrefix:
            return stateParsingArgumentPrefix(output_string, input_character, &current_metadata);

        case kStateParsingLengthModifier:
            return stateParsingLengthModifier(output_string, input_character, &current_metadata);
            break;

        default:
            return kParserInvalid;
    }

    return kParserDone;
}

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

static ParserResult stateCopyingCharacters(OutputBuffer* output_string, char input_character) {
    if (input_character == kIntroductoryCharacter) {
        current_metadata = (ArgumentMetadata){
            .zero_pad = false,
            .left_justify = false,
            .show_sign = false,
            .space_sign = false,
            .width = 0U,
        };

        current_state = kStateParsingPrefix;
        return kParserPending;
    }

    if (output_string->current_index >= (output_string->buffer_size - 1U)) {
        output_string->buffer[output_string->buffer_size - 1U] = '\0';
        return kParserDone;
    }

    output_string->buffer[output_string->current_index] = input_character;
    output_string->current_index++;
    return kParserPending;
}

static ParserResult stateParsingArgumentPrefix(OutputBuffer* output_string, char input_character,
                                               ArgumentMetadata* metadata) {
    // A double '%' will parse as a '%' symbol in the output string
    if ((output_string->buffer[output_string->current_index - 1U] == kIntroductoryCharacter) &&
        (input_character == kIntroductoryCharacter)) {
        if (output_string->current_index >= (output_string->buffer_size - 1U)) {
            output_string->buffer[output_string->buffer_size - 1U] = '\0';
            return kParserDone;
        }

        output_string->buffer[output_string->current_index] = kIntroductoryCharacter;
        output_string->current_index++;
        return kParserPending;
    }

    static uint8_t prefix_length = 0;
    if (prefix_length >= kMaxPrefixLength) {
        prefix_length = 0;
        return kParserInvalid;
    }

    switch (input_character) {
        case '0':  //Left-pads the number with zeroes (0) instead of spaces when padding is specified
            metadata->zero_pad = true;
            prefix_length++;
            break;

        case '-':  //Left-justify within the given field width; Right justification is the default
            metadata->left_justify = true;
            prefix_length++;
            break;

        case '+':  //Forces to preceed the result with a plus or minus sign (+ or -) even for positive numbers
            metadata->show_sign = true;
            prefix_length++;
            break;

        case ' ':  //If no sign is going to be written, a blank space is inserted before the value.
            metadata->space_sign = true;
            prefix_length++;
            break;

        default:
            current_state = kStateParsingLengthModifier;
            prefix_length = 0;
            break;
    }

    return kParserPending;
}

static ParserResult stateParsingLengthModifier(OutputBuffer* output_string, char input_character,
                                               ArgumentMetadata* metadata) {
    if (isnumber(input_character)) {
    }

    return kParserPending;
}
