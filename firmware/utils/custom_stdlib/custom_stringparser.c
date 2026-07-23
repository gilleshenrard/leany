/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 *
 * @file custom_stringparser.c
 * @brief Lightweight parametric string parser
 */
#include "custom_stringparser.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#include "custom_string.h"

enum : uint8_t {
    kMaxCharacterEvaluations = 10U,  ///< Maximum times the parser can reevaluate a character
};

// state machine functions
static ParserResult stateCopyingCharacters(ParserContext* context, char input_character);
static ParserResult stateParsingArgumentPrefix(ParserContext* context, char input_character);
static ParserResult stateParsingLengthModifier(ParserContext* context, char input_character);

//utility functions
static void resetContext(ParserContext* context);
static ParserResult treatDoublePercentCharacter(ParserContext* context, char input_character);

//constants
static constexpr char kPercentCharacter = '%';  ///< Character indicating the introduction of a formatted parameter

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

// NOLINTNEXTLINE (dyreadability-non-const-parameter)
bool initialiseContext(ParserContext* context, char* output_buffer, size_t output_size) {
    if (!context || !output_buffer || !output_size) {
        return false;
    }

    *context = (ParserContext){.output = {.buffer = output_buffer, .buffer_size = output_size, .current_index = 0}};
    resetContext(context);
    return true;
}

/**
 * Push an input character to be treated in the parser's state machine as an output parameter

 * @param context Current context of the parser
 * @param input_character Next character to parse
 * @return 0
 */
ParserResult pushCharacter(ParserContext* context, char input_character, va_list args) {
    if (!context || !context->output.buffer || !context->output.buffer_size) {
        return kParserInvalid;
    }

    ParserResult result = kParserPending;
    uint8_t evaluations = kMaxCharacterEvaluations;
    do {
        switch (context->state) {
            case kStateCopying:
                result = stateCopyingCharacters(context, input_character);
                break;

            case kStateParsingPrefix:
                result = stateParsingArgumentPrefix(context, input_character);
                break;

            case kStateParsingLengthModifier:
                result = stateParsingLengthModifier(context, input_character);
                break;

            default:
                result = kParserInvalid;
        }
    } while (--evaluations && (result == kParserReevaluate));

    if (!evaluations) {
        resetContext(context);
        result = kParserInvalid;
    }

    return result;
}

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

static void resetContext(ParserContext* context) {
    if (!context) {
        return;
    }

    context->state = kStateCopying;
    context->current_argument = (ArgumentMetadata){
        .introductory_consumed = false,
        .zero_pad = false,
        .left_justify = false,
        .show_sign = false,
        .space_sign = false,
        .width = 0U,
        .prefix_length = 0U,
    };
}

static ParserResult treatDoublePercentCharacter(ParserContext* context, char input_character) {
    if ((!context->current_argument.introductory_consumed) || (input_character != kPercentCharacter)) {
        return kParserPending;
    }

    resetContext(context);

    //readability variables
    char* const output_string = context->output.buffer;
    size_t* const current_index = &context->output.current_index;
    const size_t buffer_size = context->output.buffer_size;

    //output full -> cropping
    if (*current_index >= (buffer_size - 1U)) {
        output_string[buffer_size - 1U] = '\0';
        return kParserDone;
    }

    //copy the character in the output buffer
    output_string[*current_index] = kPercentCharacter;
    *current_index = (*current_index + 1U);
    return kParserSkipArgument;
}

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

static ParserResult stateCopyingCharacters(ParserContext* context, char input_character) {
    //consume the '%' character if any
    if (input_character == kPercentCharacter) {
        context->current_argument.introductory_consumed = true;
        context->state = kStateParsingPrefix;
        return kParserPending;
    }

    //readability variables
    char* const output_string = context->output.buffer;
    size_t* const current_index = &context->output.current_index;
    const size_t buffer_size = context->output.buffer_size;

    //output full -> cropping
    if (*current_index >= (buffer_size - 1U)) {
        output_string[buffer_size - 1U] = '\0';
        return kParserDone;
    }

    //copy the character in the output buffer
    output_string[*current_index] = input_character;
    *current_index = (*current_index + 1U);
    return kParserPending;
}

static ParserResult stateParsingArgumentPrefix(ParserContext* context, char input_character) {
    // A double '%' will parse as a '%' symbol in the output string
    ParserResult doublepercent_result = treatDoublePercentCharacter(context, input_character);
    if (doublepercent_result == kParserDone) {
        return kParserDone;
    }

    if (doublepercent_result == kParserSkipArgument) {
        return kParserPending;
    }

    ArgumentMetadata* const argument = &context->current_argument;

    //prefix too long, dropping
    if (argument->prefix_length >= kMaxPrefixLength) {
        resetContext(context);
        return kParserInvalid;
    }

    //qualify the prefix modifier
    switch (input_character) {
        case '0':  //Left-pads the number with zeroes (0) instead of spaces when padding is specified
            argument->zero_pad = true;
            argument->prefix_length++;
            break;

        case '-':  //Left-justify within the given field width; Right justification is the default
            argument->left_justify = true;
            argument->prefix_length++;
            break;

        case '+':  //Forces to preceed the result with a plus or minus sign (+ or -) even for positive numbers
            argument->show_sign = true;
            argument->prefix_length++;
            break;

        case ' ':  //If no sign is going to be written, a blank space is inserted before the value.
            argument->space_sign = true;
            argument->prefix_length++;
            break;

        default:
            context->state = kStateParsingLengthModifier;
            argument->prefix_length = 0;
            return kParserReevaluate;
    }

    return kParserPending;
}

static ParserResult stateParsingLengthModifier(ParserContext* context, char input_character) {
    if (isnumber(input_character)) {
    }

    return kParserPending;
}
