#include <stdint.h>
#include <unity.h>
#include <unity_internals.h>

#include "custom_stringparser.h"

enum : uint8_t {
    kStringBufferSize = 16U,  ///< Size of the output string buffer in bytes
};

static void test_null_pointer_guards(void);

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
