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

#include "custom_conversion.h"
#include "custom_string.h"

enum : uint8_t {
    kMaxCharacterEvaluations = 10U,  ///< Maximum times the parser can reevaluate a character
};

// state machine functions
static ParserResult stateCopyingCharacters(ParserContext* context, char input_character);
static ParserResult stateParsingArgumentPrefix(ParserContext* context, char input_character);
static ParserResult stateParsingFieldWidth(ParserContext* context, char input_character);
static ParserResult stateParsingConversionSpecifier(ParserContext* context, char input_character, va_list* args);

//utility functions
static void resetContext(ParserContext* context);
static ParserResult treatDoublePercentCharacter(ParserContext* context, char input_character);
static void parsePlusSignPrefix(ArgumentMetadata* argument);
static void parseSpaceSignPrefix(ArgumentMetadata* argument);
static void parseArgumentPrefixFlag(ArgumentMetadata* argument, bool* flag_modified);
static void sanitiseUnsignedFlags(ArgumentMetadata* argument);
static void sanitisePointerFlags(ArgumentMetadata* argument);

//constants
static constexpr char kPercentCharacter = '%';  ///< Character indicating the introduction of a formatted parameter
static constexpr char kDecimalCharacter = '.';  ///< Character indicating a decimal parameter

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Initialise the parser context
 * @details This ensures the context variables are properly set before use
 *
 * @param[out] context Parser context to initialise
 * @param output_buffer String buffer to attach to the context
 * @param output_size Size in bytes of the buffer attached
 * @retval true Context initialised
 * @retval false Context could not be initialised
 */
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
 * @param args List of unnamed arguments
 * @retval kParserPending The parser is waiting for new characters
 * @retval kParserInvalid The parser encountered an error due to a malformed argument
 * @retval kParserDone The parser is done parsing
 */
ParserResult pushCharacter(ParserContext* context, char input_character, va_list* args) {
    // #lizard forgives(cyclomatic_complexity)
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

            case kStateParsingFieldWidth:
                result = stateParsingFieldWidth(context, input_character);
                break;

            case kStateParsingConversionSpecifier:
                result = stateParsingConversionSpecifier(context, input_character, args);
                break;

            default:
                result = kParserInvalid;
        }
    } while (--evaluations && (result == kParserReevaluate));

    // Ceiling kept as a safety net; not reachable via current state graph
    if (result == kParserReevaluate) {
        result = kParserInvalid;
    }

    if (result != kParserPending) {
        resetContext(context);
    }

    return result;
}

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Reset the context's state and flags
 *
 * @param[out] context Context to reset
 */
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
        .field_width = 0U,
        .prefix_length = 0U,
        .precision =
            {
                .decimal_char_consumed = false,
                .magnitude = 0U,
            },
    };
}

/**
 * Examinate a percent character against the current context to see if it is a second one
 * @details In case of %%, a single '%' character is to be outputted to the output string
 *
 * @param[in,out] context Parser context to examinate
 * @param input_character New character to examinate against
 * @retval kParserPending Character is not a new percent character, move on
 * @retval kParserDone Output string needs to be cropped
 * @retval kParserSkipArgument A double percent occurred and the argument is to be skipped
 */
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

/**
 * Set a flag representing a parsed prefix modifier
 *
 * @param argument Argument being parsed
 * @param flag_modified Flag to modify
 */
static void parseArgumentPrefixFlag(ArgumentMetadata* const argument, bool* const flag_modified) {
    if (!flag_modified) {
        return;
    }

    if (*flag_modified) {
        return;
    }

    *flag_modified = true;
    argument->prefix_length++;
}

/**
 * Parse a '+' in an argument prefix
 * @details A '+' sign will nullify any previous ' ' parsed in the prefix
 *
 * @param argument Argument being parsed
 */
static void parsePlusSignPrefix(ArgumentMetadata* const argument) {
    if (argument->show_sign) {
        return;
    }

    if (argument->space_sign) {
        argument->space_sign = false;
        argument->prefix_length--;
    }

    parseArgumentPrefixFlag(argument, &argument->show_sign);
}

/**
 * Parse a ' ' in an argument prefix
 * @details A '+' sign will make the parser ignore any ' ' in the prefix
 *
 * @param argument Argument being parsed
 */
static void parseSpaceSignPrefix(ArgumentMetadata* const argument) {
    if (argument->show_sign) {
        return;
    }

    parseArgumentPrefixFlag(argument, &argument->space_sign);
}

/**
 * Sanitise flags in unsigned integer arguments which are undefined behaviour in the C standard
 *
 * @param argument Argument to sanitise
 */
static void sanitiseUnsignedFlags(ArgumentMetadata* const argument) {
    argument->show_sign = false;
    argument->space_sign = false;
}

/**
 * Sanitise flags in pointer arguments which are undefined behaviour in the C standard
 *
 * @param argument Argument to sanitise
 */
static void sanitisePointerFlags(ArgumentMetadata* const argument) {
    argument->show_sign = false;
    argument->zero_pad = false;
    argument->space_sign = false;
}

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * State during which characters are simply copied to the output
 *
 * @param[out] context Parser context
 * @param input_character New character to copy
 * @retval kParserPending The parser is waiting for a new character
 * @retval kParserDone The string had to be cropped, parser's done
 */
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

/**
 * State during which argument qualifiers are parsed to the context variables
 *
 * @param[out] context Parser context
 * @param input_character New character to evaluate
 * @retval kParserPending The parser is waiting for a new character
 * @retval kParserDone The string had to be cropped, parser's done
 * @retval kParserInvalid The parser encountered an error due to a malformed argument
 * @retval kParserReevaluate The current character needs reevaluation by another state
 */
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

    // Ceiling kept as a safety net; not reachable via current state graph
    if (argument->prefix_length >= kMaxPrefixLength) {
        resetContext(context);
        return kParserInvalid;
    }

    //collect the qualifier to interpret
    switch (input_character) {
        case '0':  //Left-pads the number with zeroes (0) instead of spaces when padding is specified
            parseArgumentPrefixFlag(argument, &argument->zero_pad);
            break;

        case '-':  //Left-justify within the given field width; Right justification is the default
            parseArgumentPrefixFlag(argument, &argument->left_justify);
            break;

        case '+':  //Forces to preceed the result with a plus or minus sign (+ or -) even for positive numbers
            parsePlusSignPrefix(argument);
            break;

        case ' ':  //If no sign is going to be written, a blank space is inserted before the value.
            parseSpaceSignPrefix(argument);
            break;

        default:
            context->state = kStateParsingFieldWidth;
            argument->field_width = 0;
            return kParserReevaluate;
    }

    return kParserPending;
}

/**
 * State during which the argument's field width are evaluated
 *
 * @param[out] context Parser context
 * @param input_character Character to evaluate
 * @retval kParserReevaluate The current character needs reevaluation by another state
 * @retval kParserPending The parser is waiting for a new character
 */
static ParserResult stateParsingFieldWidth(ParserContext* context, char input_character) {
    if (input_character == kDecimalCharacter) {
        context->current_argument.precision.decimal_char_consumed = true;
        return kParserPending;
    }

    if (!isnumber(input_character)) {
        context->state = kStateParsingConversionSpecifier;
        return kParserReevaluate;
    }

    constexpr uint32_t multiplier_10 = 10U;
    constexpr uint32_t base_number_character = '0';

    uint32_t* width_to_modify = nullptr;
    if (context->current_argument.precision.decimal_char_consumed) {
        width_to_modify = &context->current_argument.precision.magnitude;
    } else {
        width_to_modify = &context->current_argument.field_width;
    }

    *width_to_modify = (*width_to_modify * multiplier_10) + ((uint32_t)input_character - base_number_character);

    if (*width_to_modify > kMaxFormatLength) {
        *width_to_modify = kMaxFormatLength;
    }

    return kParserPending;
}

/**
 * State during which the argument's conversion specifier is evaluated
 *
 * @param[out] context Parser context
 * @param input_character Character to evaluate
 * @param args Current arg value to evaluate
 * @retval kParserDone The string had to be cropped, parser's done
 */
static ParserResult stateParsingConversionSpecifier(ParserContext* context, char input_character, va_list* args) {
    constexpr uint8_t decimal_radix = 10U;
    constexpr uint8_t hexa_radix = 16U;
    const bool is_uppercase_hex = (input_character == 'X');
    char conversion_buffer[kMaxIntBuffer];
    int32_t signed_value = 0;
    float float_value = 0.0F;
    uint32_t result_length = 0;

    switch (input_character) {
        case 'd':
        case 'i':
            signed_value = va_arg(*args, int32_t);
            result_length = convertSigned(signed_value, conversion_buffer, decimal_radix);
            outputNumber(&context->output, conversion_buffer, result_length, &context->current_argument,
                         (signed_value < 0));
            break;

        case 'u':
            sanitiseUnsignedFlags(&context->current_argument);
            result_length = convertUnsigned(va_arg(*args, uint32_t), conversion_buffer, decimal_radix, false);
            outputNumber(&context->output, conversion_buffer, result_length, &context->current_argument, false);
            break;

        case 'X':
        case 'x':
            sanitiseUnsignedFlags(&context->current_argument);
            result_length = convertUnsigned(va_arg(*args, uint32_t), conversion_buffer, hexa_radix, is_uppercase_hex);
            outputNumber(&context->output, conversion_buffer, result_length, &context->current_argument, false);
            break;

        case 's':
            outputString(&context->output, va_arg(*args, char*), &context->current_argument);
            break;

        case 'c':
            outputChar(&context->output, (char)va_arg(*args, int));
            break;

        case 'p':
            sanitisePointerFlags(&context->current_argument);
            result_length =
                convertUnsigned((uint32_t)(uintptr_t)va_arg(*args, void*), conversion_buffer, hexa_radix, false);
            outputNumber(&context->output, conversion_buffer, result_length, &context->current_argument, false);
            break;

        case 'f':
            float_value = (float)va_arg(*args, double);
            result_length = convertFloat(float_value, conversion_buffer, &context->current_argument.precision);
            outputNumber(&context->output, conversion_buffer, result_length, &context->current_argument,
                         (float_value < 0.0F));
            break;

        default:  //not-implemented specifier
            outputChar(&context->output, '%');
            outputChar(&context->output, input_character);
            return kParserSkipArgument;
    }
    return kParserDone;
}
