/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 *
 * @file test_stdlib.c
 * @brief Unit tests for the lightweight printf-style string parser (custom_stringparser).
 */
#include <stdint.h>
#include <stdio.h>
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
static void test_arguments_width(void);
static void test_unimplemented_type_specifier(void);
static void test_signed_integer_output(void);
static void test_unsigned_integer_output(void);
static void test_hexa_integer_output(void);
static void test_string_output(void);
static void test_char_output(void);
static void test_pointer_output(void);
static void test_float_output(void);
static void test_string_to_float(void);
static void test_string_to_decimal_integer(void);
static void test_string_to_hexadecimal_integer(void);
static void test_tolower_ascii(void);
static void test_get_string_length(void);
static void test_compare_string(void);
static void test_multiple_arguments(void);
static void test_flag_sanitisation_for_unsupported_conversions(void);

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
    RUN_TEST(test_arguments_width);
    RUN_TEST(test_unimplemented_type_specifier);
    RUN_TEST(test_signed_integer_output);
    RUN_TEST(test_unsigned_integer_output);
    RUN_TEST(test_hexa_integer_output);
    RUN_TEST(test_string_output);
    RUN_TEST(test_char_output);
    RUN_TEST(test_pointer_output);
    RUN_TEST(test_float_output);
    RUN_TEST(test_string_to_float);
    RUN_TEST(test_string_to_decimal_integer);
    RUN_TEST(test_string_to_hexadecimal_integer);
    RUN_TEST(test_tolower_ascii);
    RUN_TEST(test_get_string_length);
    RUN_TEST(test_compare_string);
    RUN_TEST(test_multiple_arguments);
    RUN_TEST(test_flag_sanitisation_for_unsupported_conversions);
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
                             .field_width = UINT32_MAX,
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
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, parser_context.current_argument.field_width, "Invalid field width");
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
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(5, parser_context.current_argument.field_width, "Invalid field width resetting");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(kStateParsingFieldWidth, parser_context.state, "Invalid state");
}

/**
 * Check the format width modifiers and the overflow protection
 */
static void test_arguments_width(void) {
    (void)pushCharacter(&parser_context, '%', nullptr);

    (void)pushCharacter(&parser_context, '5', nullptr);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(5, parser_context.current_argument.field_width,
                                    "Invalid field width at first character");

    (void)pushCharacter(&parser_context, '5', nullptr);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(55, parser_context.current_argument.field_width,
                                    "Invalid field width at second character");

    (void)pushCharacter(&parser_context, '5', nullptr);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(kMaxFormatLength, parser_context.current_argument.field_width,
                                    "Invalid field width at third character");
}

/**
 * Test the serialisation of an unimplemented type specifier
 */
static void test_unimplemented_type_specifier(void) {
    constexpr uint8_t buffer_size = 10U;
    char result[buffer_size];

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat"
#pragma GCC diagnostic ignored "-Wformat-extra-args"
    custom_snprintf(result, buffer_size, "%w", 5);  //NOLINT (cppcoreguidelines-avoid-magic-numbers)
    TEST_ASSERT_EQUAL_STRING("%w", result);
#pragma GCC diagnostic pop
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
    custom_snprintf(result, buffer_size, "%d", (int32_t)(INT32_MAX + 1U));
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

    custom_snprintf(result, buffer_size, "%u", 5U);
    TEST_ASSERT_EQUAL_STRING("5", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%u", (unsigned int)-5);
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat"
    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%+u", 5U);
    TEST_ASSERT_EQUAL_STRING_MESSAGE("5", result, "'+' is undefined behaviour with %%u and should be sanitised out");
#pragma GCC diagnostic pop

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%5u", 5U);
    TEST_ASSERT_EQUAL_STRING("    5", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%-5u", 5U);
    TEST_ASSERT_EQUAL_STRING("5    ", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%05u", 5U);
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

    custom_snprintf(result, buffer_size, "%x", 10U);
    TEST_ASSERT_EQUAL_STRING("a", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%X", 10U);
    TEST_ASSERT_EQUAL_STRING("A", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%x", (unsigned int)-5);
    TEST_ASSERT_EQUAL_STRING("fffffffb", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%x", UINT32_MAX);
    TEST_ASSERT_EQUAL_STRING("ffffffff", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%x", UINT32_MAX + 1U);
    TEST_ASSERT_EQUAL_STRING("0", result);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat"
    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%+x", 10U);
    TEST_ASSERT_EQUAL_STRING_MESSAGE("a", result, "'+' is undefined behaviour with %%x and should be sanitised out");
#pragma GCC diagnostic pop

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%5x", 10U);
    TEST_ASSERT_EQUAL_STRING("    a", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%-5x", 10U);
    TEST_ASSERT_EQUAL_STRING("a    ", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%05x", 10U);
    TEST_ASSERT_EQUAL_STRING("0000a", result);

    // NOLINTEND (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling, cppcoreguidelines-avoid-magic-numbers)
}

/**
 * Test the serialisation of a string with modifiers
 */
static void test_string_output(void) {
    // NOLINTBEGIN (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
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

    memset(result, '\0', long_buffer_size);
    custom_snprintf(result, long_buffer_size, "%.10s", long_string);
    TEST_ASSERT_EQUAL_STRING("This is a ", result);

    // NOLINTEND (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
}

/**
 * Test the serialisation of a character
 */
static void test_char_output(void) {
    // NOLINTBEGIN (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)

    constexpr uint8_t buffer_size = 10U;
    char result[buffer_size];

    custom_snprintf(result, buffer_size, "%c", 'A');
    TEST_ASSERT_EQUAL_STRING("A", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%5c", 'A');
    TEST_ASSERT_EQUAL_STRING("A", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%-5c", 'A');
    TEST_ASSERT_EQUAL_STRING("A", result);

    memset(result, 'X', buffer_size);
    custom_snprintf(result, buffer_size, "%c", '\0');
    TEST_ASSERT_EQUAL_CHAR(0, result[0]);
    TEST_ASSERT_EQUAL_CHAR(0, result[1]);

    // NOLINTEND (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
}

/**
 * Test the serialisation of a pointer as a 32-bits hexadecimal address
 */
static void test_pointer_output(void) {
    // NOLINTBEGIN (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
    constexpr uint8_t buffer_size = 20U;
    char result[buffer_size];
    char expected[buffer_size];
    int dummy_variable = 0;

    const uint32_t truncated_address = (uint32_t)(uintptr_t)&dummy_variable;
    snprintf(expected, buffer_size, "%x", truncated_address);

    custom_snprintf(result, buffer_size, "%p", (void*)&dummy_variable);
    TEST_ASSERT_EQUAL_STRING(expected, result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%p", (void*)nullptr);
    TEST_ASSERT_EQUAL_STRING("0", result);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat"
    memset(result, '\0', buffer_size);
    snprintf(expected, buffer_size, "%8x", truncated_address);
    custom_snprintf(result, buffer_size, "%08p", (void*)&dummy_variable);
    TEST_ASSERT_EQUAL_STRING_MESSAGE(expected, result,
                                     "'0' is undefined behaviour with %%p and should be sanitised out");
#pragma GCC diagnostic pop

    // NOLINTEND (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
}

/**
 * Test the serialisation of a floating-point number
 */
static void test_float_output(void) {
    // #lizard forgives(length)
    // NOLINTBEGIN (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
    constexpr uint8_t buffer_size = 20U;
    char result[buffer_size];

    custom_snprintf(result, buffer_size, "%f", (double)2.5F);
    TEST_ASSERT_EQUAL_STRING("2.500000", result);

    custom_snprintf(result, buffer_size, "%f", (double)14000.0F);
    TEST_ASSERT_EQUAL_STRING("14000.000000", result);

    custom_snprintf(result, buffer_size, "%.3f", (double)20000000.0F);
    TEST_ASSERT_EQUAL_STRING("20000000.000", result);

    custom_snprintf(result, buffer_size, "%f", (double)2.005F);
    TEST_ASSERT_EQUAL_STRING("2.005000", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%f", (double)-2.5F);
    TEST_ASSERT_EQUAL_STRING("-2.500000", result);

    custom_snprintf(result, buffer_size, "%f", (double)-2.005F);
    TEST_ASSERT_EQUAL_STRING("-2.005000", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%19f", (double)2.5F);
    TEST_ASSERT_EQUAL_STRING("           2.500000", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%19f", (double)-2.5F);
    TEST_ASSERT_EQUAL_STRING("          -2.500000", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%3f", (double)2.5F);
    TEST_ASSERT_EQUAL_STRING_MESSAGE("2.500000", result,
                                     "A length modifier smaller than the output size should not have effect");

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%19.3f", (double)2.5F);
    TEST_ASSERT_EQUAL_STRING("              2.500", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%-19.3f", (double)2.5F);
    TEST_ASSERT_EQUAL_STRING("2.500              ", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%-19.3f", (double)-2.5F);
    TEST_ASSERT_EQUAL_STRING("-2.500             ", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%019.3f", (double)2.5F);
    TEST_ASSERT_EQUAL_STRING("000000000000002.500", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%019.3f", (double)-2.5F);
    TEST_ASSERT_EQUAL_STRING("-00000000000002.500", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%+019.3f", (double)2.5F);
    TEST_ASSERT_EQUAL_STRING("+00000000000002.500", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%.0f", (double)2.5F);
    TEST_ASSERT_EQUAL_STRING("2", result);
    // NOLINTEND (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
}

/**
 * Test the string to float conversion
 */
static void test_string_to_float(void) {
    // NOLINTBEGIN (cppcoreguidelines-avoid-magic-numbers)
    float result = custom_strtof("2.5");
    TEST_ASSERT_EQUAL_FLOAT(2.5F, result);

    result = custom_strtof("6.54");
    TEST_ASSERT_EQUAL_FLOAT(6.54F, result);

    result = custom_strtof("0.0");
    TEST_ASSERT_EQUAL_FLOAT(0.0F, result);

    result = custom_strtof("-2.5");
    TEST_ASSERT_EQUAL_FLOAT(-2.5F, result);

    result = custom_strtof("-6.54");
    TEST_ASSERT_EQUAL_FLOAT(-6.54F, result);
    // NOLINTEND (cppcoreguidelines-avoid-magic-numbers)
}

/**
 * Test the string to decimal integer conversion
 */
static void test_string_to_decimal_integer(void) {
    // NOLINTBEGIN (cppcoreguidelines-avoid-magic-numbers)
    uint32_t result = custom_strtoi("10");
    TEST_ASSERT_EQUAL_UINT32(10, result);

    result = custom_strtoi("0");
    TEST_ASSERT_EQUAL_UINT32(0, result);

    result = custom_strtoi("4294967295");
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, result);

    result = custom_strtoi("4294967295636");
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, result);
    // NOLINTEND (cppcoreguidelines-avoid-magic-numbers)
}

/**
 * Test the string to hexadecimal integer conversion
 */
static void test_string_to_hexadecimal_integer(void) {
    // NOLINTBEGIN (cppcoreguidelines-avoid-magic-numbers)
    uint32_t result = custom_strtoi_hexa("A");
    TEST_ASSERT_EQUAL_UINT32(0xA, result);

    result = custom_strtoi_hexa("0");
    TEST_ASSERT_EQUAL_UINT32(0, result);

    result = custom_strtoi_hexa("FFFFFFFF");
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, result);

    result = custom_strtoi_hexa("FFFFFFFFFF");
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, result);
    // NOLINTEND (cppcoreguidelines-avoid-magic-numbers)
}

/**
 * Test the lowering of the case of an ascii character
 */
static void test_tolower_ascii(void) {
    // NOLINTBEGIN (cppcoreguidelines-avoid-magic-numbers)
    char result = toLowerAscii('A');
    TEST_ASSERT_EQUAL_CHAR('a', result);

    result = toLowerAscii('a');
    TEST_ASSERT_EQUAL_CHAR('a', result);

    result = toLowerAscii(('A' - 1));
    TEST_ASSERT_EQUAL_CHAR(('A' - 1), result);

    result = toLowerAscii(('z' + 1));
    TEST_ASSERT_EQUAL_CHAR(('z' + 1), result);

    result = toLowerAscii(('Z' + 1));
    TEST_ASSERT_EQUAL_CHAR(('Z' + 1), result);
    // NOLINTEND (cppcoreguidelines-avoid-magic-numbers)
}

/**
 * Test getting the length of a string, bounded by a maximum length
 */
static void test_get_string_length(void) {
    // NOLINTBEGIN (cppcoreguidelines-avoid-magic-numbers)
    TEST_ASSERT_EQUAL_size_t(0, getStringLength(nullptr, 10));
    TEST_ASSERT_EQUAL_size_t(0, getStringLength("", 10));
    TEST_ASSERT_EQUAL_size_t(5, getStringLength("Hello", 10));
    TEST_ASSERT_EQUAL_size_t(5, getStringLength("Hello, World!", 5));  // truncated by max_length
    TEST_ASSERT_EQUAL_size_t(0, getStringLength("Hello", 0));
    // NOLINTEND (cppcoreguidelines-avoid-magic-numbers)
}

/**
 * Test comparing two strings in a case-insensitive manner, bounded by their respective sizes
 */
static void test_compare_string(void) {
    // NOLINTBEGIN (cppcoreguidelines-avoid-magic-numbers)
    TEST_ASSERT_EQUAL_INT8(0, compareString("hello", 5, "hello", 5));
    TEST_ASSERT_EQUAL_INT8(0, compareString("Hello", 5, "hello", 5));  // case-insensitive
    TEST_ASSERT_EQUAL_INT8(0, compareString("HELLO", 5, "hello", 5));

    TEST_ASSERT_TRUE(compareString("hello", 5, "apple", 5) > 0);  // 'h' > 'a'
    TEST_ASSERT_TRUE(compareString("apple", 5, "hello", 5) < 0);  // 'a' < 'h'

    // one string a prefix of the other, compared over their full lengths
    TEST_ASSERT_TRUE(compareString("hello", 5, "hell", 4) > 0);
    TEST_ASSERT_TRUE(compareString("hell", 4, "hello", 5) < 0);

    // early null terminator inside the given size
    TEST_ASSERT_EQUAL_INT8(0, compareString("hi\0xx", 5, "hi\0yy", 5));

    // comparison bounded by the smaller of the two sizes, even if buffers are longer
    TEST_ASSERT_EQUAL_INT8(0, compareString("hello world", 5, "hello there", 5));

    // zero size on either side
    TEST_ASSERT_EQUAL_INT8(0, compareString("hello", 0, "hello", 5));
    TEST_ASSERT_EQUAL_INT8(0, compareString("hello", 5, "hello", 0));
    // NOLINTEND (cppcoreguidelines-avoid-magic-numbers)
}

/**
 * Test a full string with multiple arguments
 */
static void test_multiple_arguments(void) {
    // NOLINTBEGIN (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling, cppcoreguidelines-avoid-magic-numbers)

    constexpr uint8_t long_buffer_size = 64U;
    char buffer[long_buffer_size];
    char expected[long_buffer_size];

    int32_t expected_length = snprintf(expected, long_buffer_size, "%.10s %08.2f and %-8.2f is %+8.2f, not %02u, %c",
                                       "The sum of", (double)2.45F, (double)6.54F, (double)8.99F, 2U, 'A');

    int32_t length = custom_snprintf(buffer, long_buffer_size, "%.10s %08.2f and %-8.2f is %+8.2f, not %02u, %c",
                                     "The sum of", (double)2.45F, (double)6.54F, (double)8.99F, 2U, 'A');
    TEST_ASSERT_EQUAL_STRING(expected, buffer);
    TEST_ASSERT_EQUAL_INT32(expected_length, length);

    // NOLINTEND (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling, cppcoreguidelines-avoid-magic-numbers)
}

/**
 * Check that flags with no defined meaning for a given conversion — '+' and ' ' on %u/%x,
 * '0' on %p — are silently dropped rather than applied or rejected, and that this doesn't
 * disturb argument consumption for the rest of the format string
 */
static void test_flag_sanitisation_for_unsupported_conversions(void) {
// NOLINTBEGIN (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling, cppcoreguidelines-avoid-magic-numbers)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat"

    constexpr uint8_t buffer_size = 20U;
    char result[buffer_size];

    // '+' is meaningless for %u/%x : output identical to the plain conversion
    custom_snprintf(result, buffer_size, "%+u", 5U);
    TEST_ASSERT_EQUAL_STRING("5", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%+x", 10U);
    TEST_ASSERT_EQUAL_STRING("a", result);

    // ' ' is meaningless for %u/%x too, and untested elsewhere
    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "% u", 5U);
    TEST_ASSERT_EQUAL_STRING("5", result);

    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "% x", 10U);
    TEST_ASSERT_EQUAL_STRING("a", result);

    // '0' is meaningless for %p : falls back to space padding, same as plain width
    int dummy_variable = 0;
    const uint32_t truncated_address = (uint32_t)(uintptr_t)&dummy_variable;
    char expected[buffer_size];

    memset(result, '\0', buffer_size);
    snprintf(expected, buffer_size, "%8x", truncated_address);
    custom_snprintf(result, buffer_size, "%08p", (void*)&dummy_variable);
    TEST_ASSERT_EQUAL_STRING(expected, result);

    // Dropping a flag must not shift argument consumption for what follows
    memset(result, '\0', buffer_size);
    custom_snprintf(result, buffer_size, "%+u-%d", 5U, -3);
    TEST_ASSERT_EQUAL_STRING("5--3", result);

#pragma GCC diagnostic pop
    // NOLINTEND (clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling, cppcoreguidelines-avoid-magic-numbers)
}
