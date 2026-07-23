#include <stdint.h>
#include <unity.h>
#include <unity_internals.h>

#include "custom_stringparser.h"

enum : uint8_t {
    kStringBufferSize = 16U,  ///< Size of the output string buffer in bytes
};

static void test_null_pointer_guards(void);
static void test_context_initialisation(void);

static ParserContext parser_context;

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
    return UNITY_END();
}

/**
 * Initialise the filter context to a clean, known state before each test.
 *
 * @internal
 * resetMahonyFilter() only resets the quaternion, error integrals, and bad counters.
 * All other fields must be set explicitly here.
 */
void setUp(void) {}

/**
 * Free up the resources used during each test
 */
void tearDown(void) {}

/*********************************************************************************************************************************/
// TEST FUNCTIONS
/*********************************************************************************************************************************/

static void test_null_pointer_guards(void) {
    char test_buffer[kStringBufferSize];
    TEST_ASSERT_FALSE_MESSAGE(initialiseContext(nullptr, test_buffer, kStringBufferSize), "nullptr context failed");
    TEST_ASSERT_FALSE_MESSAGE(initialiseContext(&parser_context, nullptr, kStringBufferSize),
                              "nullptr output buffer failed");
    TEST_ASSERT_FALSE_MESSAGE(initialiseContext(&parser_context, test_buffer, 0U), "zero-size output buffer failed");
}

static void test_context_initialisation(void) {
    char test_buffer[kStringBufferSize];

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

    TEST_ASSERT_EQUAL_PTR_MESSAGE(test_buffer, parser_context.output.buffer,
                                  "Invalid output buffer address at initialisation");
    TEST_ASSERT_EQUAL_size_t_MESSAGE(kStringBufferSize, parser_context.output.buffer_size,
                                     "Invalid buffer size at initialisation");
    TEST_ASSERT_EQUAL_size_t_MESSAGE(0, parser_context.output.current_index, "Invalid buffer index at initialisation");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(kStateCopying, parser_context.state, "Invalid parser state at initialisation");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(0, parser_context.current_argument.prefix_length,
                                    "Invalid prefix length at initialisation");
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(0, parser_context.current_argument.width,
                                     "Invalid argument width at initialisation");
    TEST_ASSERT_FALSE_MESSAGE(parser_context.current_argument.introductory_consumed,
                              "Invalid introduction flag value after initialisation");
    TEST_ASSERT_FALSE_MESSAGE(parser_context.current_argument.left_justify,
                              "Invalid justification flag value after initialisation");
    TEST_ASSERT_FALSE_MESSAGE(parser_context.current_argument.show_sign,
                              "Invalid sign forcing flag value after initialisation");
    TEST_ASSERT_FALSE_MESSAGE(parser_context.current_argument.space_sign,
                              "Invalid space sign flag value after initialisation");
    TEST_ASSERT_FALSE_MESSAGE(parser_context.current_argument.zero_pad,
                              "Invalid zero padding flag value after initialisation");
}
