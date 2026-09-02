/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 *
 * @file test_stdlib.c
 * @brief Unit tests for the lightweight printf-style string parser (custom_stringparser).
 */
#include <stdint.h>
#include <string.h>
#include <unity.h>
#include <unity_internals.h>

#include "custom_string.h"
#include "custom_stringparser.h"

enum : uint8_t {
    kStringBufferSize = 16U,  ///< Size of the output string buffer in bytes
};

static void test_null_pointer_guards(void);
static void test_context_initialisation(void);
static void test_string_without_arguments(void);
static void test_double_percents(void);
static void test_arguments_evaluation(void);
static void test_invalid_argument_format_length(void);
static void test_arguments_reevaluation_overflow(void);
static void test_arguments_width(void);
static void test_unimplemented_type_specifier(void);
static void test_signed_integer_output(void);
static void test_unsigned_integer_output(void);
static void test_hexa_integer_output(void);
static void test_string_output(void);

static ParserContext parser_context;         ///< Parser context to use on all tests
static char test_buffer[kStringBufferSize];  ///< String buffer to use as output on all tests

/*********************************************************************************************************************************/
// PUBLIC FUNCTIONS
/*********************************************************************************************************************************/

/**
 * Tests runner
 *
 * @return UNITY_END result 
 */
int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_null_pointer_guards);
    RUN_TEST(test_context_initialisation);
    RUN_TEST(test_string_without_arguments);
    RUN_TEST(test_double_percents);
    RUN_TEST(test_arguments_evaluation);
    RUN_TEST(test_invalid_argument_format_length);
    RUN_TEST(test_arguments_reevaluation_overflow);
    RUN_TEST(test_arguments_width);
    RUN_TEST(test_unimplemented_type_specifier);
    RUN_TEST(test_signed_integer_output);
    RUN_TEST(test_unsigned_integer_output);
    RUN_TEST(test_hexa_integer_output);
    RUN_TEST(test_string_output);
    return UNITY_END();
}

/**
 * Initialise the parser context to a clean, known state before each test.
 */
void setUp(void) {
    // NOLINTNEXTLINE (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
    memset(test_buffer, '\0', kStringBufferSize);
    (void)initialiseContext(&parser_context, test_buffer, kStringBufferSize);
}

/**
 * Free up the resources used during each test
 */
void tearDown(void) {}

/*********************************************************************************************************************************/
// TEST FUNCTIONS
/*********************************************************************************************************************************/

/**
 * Check initialiseContext() rejects a null context, a null output buffer, and a zero-size buffer
 */
static void test_null_pointer_guards(void) {
    TEST_ASSERT_FALSE_MESSAGE(initialiseContext(nullptr, test_buffer, kStringBufferSize), "nullptr context failed");
    TEST_ASSERT_FALSE_MESSAGE(initialiseContext(&parser_context, nullptr, kStringBufferSize),
                              "nullptr output buffer failed");
    TEST_ASSERT_FALSE_MESSAGE(initialiseContext(&parser_context, test_buffer, 0U), "zero-size output buffer failed");
}

/**
 * Check initialiseContext() resets a fully "poisoned" context back to its clean default state
 */
static void test_context_initialisation(void) {
    parser_context = (ParserContext){
        // clang-format off
        .state = kStateParsingPrefix,
        .output = {.buffer = nullptr, .buffer_size = 0, .current_index = SIZE_MAX},
        .current_argument = {.prefix_length = UINT8_MAX,
                             .width = UINT32_MAX,
                             .introductory_consumed = true,
                             .left_justify = true,
                             .show_sign = true,
                             .space_sign = true,
                             .zero_pad = true},
        // clang-format on
    };

    TEST_ASSERT_TRUE_MESSAGE(initialiseContext(&parser_context, test_buffer, kStringBufferSize),
                             "context initialisation failed");

    TEST_ASSERT_EQUAL_PTR_MESSAGE(test_buffer, parser_context.output.buffer, "Invalid output buffer address");
    TEST_ASSERT_EQUAL_size_t_MESSAGE(kStringBufferSize, parser_context.output.buffer_size, "Invalid buffer size");
    TEST_ASSERT_EQUAL_size_t_MESSAGE(0, parser_context.output.current_index, "Invalid buffer index");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(kStateCopying, parser_context.state, "Invalid parser state");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, parser_context.current_argument.prefix_length, "Invalid prefix length");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, parser_context.current_argument.width, "Invalid argument width");
    TEST_ASSERT_FALSE_MESSAGE(parser_context.current_argument.introductory_consumed, "Invalid introduction flag value");
    TEST_ASSERT_FALSE_MESSAGE(parser_context.current_argument.left_justify, "Invalid justification flag value");
    TEST_ASSERT_FALSE_MESSAGE(parser_context.current_argument.show_sign, "Invalid sign forcing flag value");
    TEST_ASSERT_FALSE_MESSAGE(parser_context.current_argument.space_sign, "Invalid space sign flag value");
    TEST_ASSERT_FALSE_MESSAGE(parser_context.current_argument.zero_pad, "Invalid zero padding flag value");
}

/**
 * Check plain characters (no '%' introducer) are copied verbatim, and that the output
 * is truncated once the destination buffer is full
 */
static void test_string_without_arguments(void) {
    const char testchar[kStringBufferSize] = "000000000000000";
    ParserResult result = kParserDone;

    for (uint8_t character = 0; character < (kStringBufferSize - 1U); character++) {
        result = pushCharacter(&parser_context, '0', nullptr);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(kParserPending, result, "Invalid state while pushing characters");
    }
    TEST_ASSERT_EQUAL_STRING_MESSAGE(testchar, parser_context.output.buffer, "Characters not pushed properly");

    for (uint8_t character = 0; character < (kStringBufferSize - 1U); character++) {
        result = pushCharacter(&parser_context, '0', nullptr);
        TEST_ASSERT_EQUAL_UINT8_MESSAGE(kParserDone, result, "Invalid cropping result");
    }
    TEST_ASSERT_EQUAL_STRING_MESSAGE(testchar, parser_context.output.buffer, "Final string not cropped properly");
}

/**
 * Check "%%" is collapsed to a single literal '%' in the output, and that a further
 * "%" after that correctly re-enters prefix parsing rather than being copied verbatim
 */
static void test_double_percents(void) {
    (void)pushCharacter(&parser_context, '%', nullptr);
    (void)pushCharacter(&parser_context, '%', nullptr);
    (void)pushCharacter(&parser_context, 'd', nullptr);
    TEST_ASSERT_EQUAL_STRING_MESSAGE("%d", parser_context.output.buffer, "Double percents not accounted for");

    setUp();
    (void)pushCharacter(&parser_context, '%', nullptr);
    (void)pushCharacter(&parser_context, '%', nullptr);
    (void)pushCharacter(&parser_context, '%', nullptr);
    TEST_ASSERT_EQUAL_STRING_MESSAGE("%", parser_context.output.buffer, "Double percents not accounted for");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(kStateParsingPrefix, parser_context.state, "Invalid state");
}

/**
 * Check if argument modifier flags are properly set
 */
static void test_arguments_evaluation(void) {
    (void)pushCharacter(&parser_context, '%', nullptr);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(kStateParsingPrefix, parser_context.state, "Invalid state");

    (void)pushCharacter(&parser_context, '0', nullptr);
    TEST_ASSERT_TRUE_MESSAGE(parser_context.current_argument.zero_pad, "Invalid zero padding flag");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(1, parser_context.current_argument.prefix_length, "Invalid index length");

    (void)pushCharacter(&parser_context, '-', nullptr);
    TEST_ASSERT_TRUE_MESSAGE(parser_context.current_argument.left_justify, "Invalid left-justification flag");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(2, parser_context.current_argument.prefix_length, "Invalid index length");

    (void)pushCharacter(&parser_context, '+', nullptr);
    TEST_ASSERT_TRUE_MESSAGE(parser_context.current_argument.show_sign, "Invalid sign show focring flag");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(3, parser_context.current_argument.prefix_length, "Invalid index length");

    (void)pushCharacter(&parser_context, ' ', nullptr);
    TEST_ASSERT_FALSE_MESSAGE(parser_context.current_argument.space_sign, "Space filling flag not properly ignored");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(3, parser_context.current_argument.prefix_length, "Invalid index length");

    parser_context.current_argument.show_sign = false;
    (void)pushCharacter(&parser_context, ' ', nullptr);
    TEST_ASSERT_TRUE_MESSAGE(parser_context.current_argument.space_sign, "Invalid space filling flag");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(4, parser_context.current_argument.prefix_length, "Invalid index length");

    (void)pushCharacter(&parser_context, '5', nullptr);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(4, parser_context.current_argument.prefix_length,
                                    "Invalid index length at length modifier evaluation");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(5, parser_context.current_argument.width, "Invalid format length resetting");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(kStateParsingLengthModifier, parser_context.state, "Invalid state");
}

/**
 * Check an over-long length modifier (width/precision digit run) is rejected as invalid
 * and the context is reset to a clean copying state
 *
 * @todo Implementation pending stateParsingLengthModifier()
 */
static void test_invalid_argument_format_length(void) {
    // const char invalid_argument_modifiers[] = "%- 012.013d";
    // ParserResult result = kParserPending;

    // const char* iterator = invalid_argument_modifiers;
    // while (*iterator != '\0') {
    //     result = pushCharacter(&parser_context, *iterator, nullptr);
    //     iterator++;
    // }

    // TEST_ASSERT_EQUAL_UINT8_MESSAGE(kParserInvalid, result, "Argument not shown as invalid");

    // TEST_ASSERT_EQUAL_size_t_MESSAGE(0, parser_context.output.current_index, "Invalid buffer index");
    // TEST_ASSERT_EQUAL_UINT8_MESSAGE(kStateCopying, parser_context.state, "Invalid parser state");
    // TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, parser_context.current_argument.prefix_length, "Invalid prefix length");
    // TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, parser_context.current_argument.width, "Invalid argument width");
    // TEST_ASSERT_FALSE_MESSAGE(parser_context.current_argument.introductory_consumed, "Invalid introduction flag value");
    // TEST_ASSERT_FALSE_MESSAGE(parser_context.current_argument.left_justify, "Invalid justification flag value");
    // TEST_ASSERT_FALSE_MESSAGE(parser_context.current_argument.show_sign, "Invalid sign forcing flag value");
    // TEST_ASSERT_FALSE_MESSAGE(parser_context.current_argument.space_sign, "Invalid space sign flag value");
    // TEST_ASSERT_FALSE_MESSAGE(parser_context.current_argument.zero_pad, "Invalid zero padding flag value");
}

/**
 * Check that exceeding kMaxCharacterEvaluations re-evaluations for a single character
 * is reported as invalid rather than looping indefinitely
 *
 * @todo Implementation pending
 */
static void test_arguments_reevaluation_overflow(void) {}

/**
 * Check the format width modifiers and the overflow protection
 */
static void test_arguments_width(void) {
    (void)pushCharacter(&parser_context, '%', nullptr);

    (void)pushCharacter(&parser_context, '5', nullptr);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(5, parser_context.current_argument.width,
                                    "Invalid format length at first character");

    (void)pushCharacter(&parser_context, '5', nullptr);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(55, parser_context.current_argument.width,
                                    "Invalid format length at second character");

    (void)pushCharacter(&parser_context, '5', nullptr);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(kMaxFormatLength, parser_context.current_argument.width,
                                    "Invalid format length at third character");
}

/**
 * Test the serialisation of an unimplemented type specifier
 */
static void test_unimplemented_type_specifier(void) {
    constexpr uint8_t buffer_size = 10U;
    char result[buffer_size];

    custom_snprintf(result, buffer_size, "%w", 5);  //NOLINT (cppcoreguidelines-avoid-magic-numbers)
    TEST_ASSERT_EQUAL_STRING("%w", result);
}

/**
 * Test the serialisation of a signed integer to a string
 */
static void test_signed_integer_output(void) {
    // NOLINTBEGIN (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling, cppcoreguidelines-avoid-magic-numbers)
    constexpr uint8_t buffer_size = 10U;
    char result[buffer_size];

    custom_snprintf(result, buffer_size, "%d", 5);
    TEST_ASSERT_EQUAL_STRING("5", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%d", -5);
    TEST_ASSERT_EQUAL_STRING("-5", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%d", INT32_MAX);
    TEST_ASSERT_EQUAL_STRING("214748364", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%d", INT32_MAX + 1U);
    TEST_ASSERT_EQUAL_STRING("-21474836", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%+d", 5);
    TEST_ASSERT_EQUAL_STRING("+5", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%5d", 5);
    TEST_ASSERT_EQUAL_STRING("    5", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%10d", INT32_MAX);
    TEST_ASSERT_EQUAL_STRING("214748364", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%-5d", 5);
    TEST_ASSERT_EQUAL_STRING("5    ", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%05d", 5);
    TEST_ASSERT_EQUAL_STRING("00005", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%05d", -5);
    TEST_ASSERT_EQUAL_STRING("-0005", result);

    // NOLINTEND (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling, cppcoreguidelines-avoid-magic-numbers)
}

/**
 * Test the serialisation of an unsigned integer to a string
 */
static void test_unsigned_integer_output(void) {
    // NOLINTBEGIN (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling, cppcoreguidelines-avoid-magic-numbers)
    constexpr uint8_t buffer_size = 10U;
    char result[buffer_size];

    custom_snprintf(result, buffer_size, "%u", 5);
    TEST_ASSERT_EQUAL_STRING("5", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%u", -5);
    TEST_ASSERT_EQUAL_STRING("429496729", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%u", UINT32_MAX);
    TEST_ASSERT_EQUAL_STRING("429496729", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%10u", UINT32_MAX);
    TEST_ASSERT_EQUAL_STRING("429496729", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%u", UINT32_MAX + 1U);
    TEST_ASSERT_EQUAL_STRING("0", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%+u", 5);
    TEST_ASSERT_EQUAL_STRING("+5", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%5u", 5);
    TEST_ASSERT_EQUAL_STRING("    5", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%-5u", 5);
    TEST_ASSERT_EQUAL_STRING("5    ", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%05u", 5);
    TEST_ASSERT_EQUAL_STRING("00005", result);

    // NOLINTEND (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling, cppcoreguidelines-avoid-magic-numbers)
}

/**
 * Test the serialisation of an hexadecimal integer to a string
 */
static void test_hexa_integer_output(void) {
    // NOLINTBEGIN (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling, cppcoreguidelines-avoid-magic-numbers)
    constexpr uint8_t buffer_size = 10U;
    char result[buffer_size];

    custom_snprintf(result, buffer_size, "%x", 10);
    TEST_ASSERT_EQUAL_STRING("a", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%X", 10);
    TEST_ASSERT_EQUAL_STRING("A", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%x", -5);
    TEST_ASSERT_EQUAL_STRING("fffffffb", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%x", UINT32_MAX);
    TEST_ASSERT_EQUAL_STRING("ffffffff", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%x", UINT32_MAX + 1U);
    TEST_ASSERT_EQUAL_STRING("0", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%+x", 10);
    TEST_ASSERT_EQUAL_STRING("+a", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%5x", 10);
    TEST_ASSERT_EQUAL_STRING("    a", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%-5x", 10);
    TEST_ASSERT_EQUAL_STRING("a    ", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%05x", 10);
    TEST_ASSERT_EQUAL_STRING("0000a", result);

    // NOLINTEND (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling, cppcoreguidelines-avoid-magic-numbers)
}

/**
 * Test the serialisation of a string with modifiers
 */
static void test_string_output(void) {
    // NOLINTBEGIN (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling, cppcoreguidelines-avoid-magic-numbers)
    constexpr uint8_t short_buffer_size = 10U;
    constexpr uint8_t long_buffer_size = 64U;
    constexpr char long_string[] = "This is a very long string oh my god would you look at that";
    char result[long_buffer_size];

    custom_snprintf(result, short_buffer_size, "%s", long_string);
    TEST_ASSERT_EQUAL_STRING("This is a", result);

    memset(result, '\0', long_buffer_size);
    custom_snprintf(result, long_buffer_size, "%s", long_string);
    TEST_ASSERT_EQUAL_STRING(long_string, result);

    memset(result, '\0', long_buffer_size);
    custom_snprintf(result, long_buffer_size, "%64s", long_string);
    TEST_ASSERT_EQUAL_STRING("     This is a very long string oh my god would you look at tha", result);

    memset(result, '\0', long_buffer_size);
    custom_snprintf(result, long_buffer_size, "%63s", long_string);
    TEST_ASSERT_EQUAL_STRING("    This is a very long string oh my god would you look at that", result);

    memset(result, '\0', long_buffer_size);
    custom_snprintf(result, long_buffer_size, "%-63s", long_string);
    TEST_ASSERT_EQUAL_STRING("This is a very long string oh my god would you look at that    ", result);

    // NOLINTEND (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling, cppcoreguidelines-avoid-magic-numbers)
}
