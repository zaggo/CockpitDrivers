#include <unity.h>
#include "CanIdError.h"

void setUp(void) {}
void tearDown(void) {}

void test_anyCanIdHasError_false_when_empty(void) {
    CanIdError errors[4] = {};
    TEST_ASSERT_FALSE(anyCanIdHasError(errors, 0));
}

void test_anyCanIdHasError_false_when_none_active(void) {
    CanIdError errors[2] = {
        {0x300, false, CanErrorType::NONE},
        {0x301, false, CanErrorType::NONE},
    };
    TEST_ASSERT_FALSE(anyCanIdHasError(errors, 2));
}

void test_anyCanIdHasError_true_when_one_active(void) {
    CanIdError errors[3] = {
        {0x300, false, CanErrorType::NONE},
        {0x301, true, CanErrorType::HEARTBEAT_TIMEOUT},
        {0x201, false, CanErrorType::NONE},
    };
    TEST_ASSERT_TRUE(anyCanIdHasError(errors, 3));
}

void test_anyCanIdHasError_ignores_entries_past_count(void) {
    CanIdError errors[2] = {
        {0x300, false, CanErrorType::NONE},
        {0x301, true, CanErrorType::TX_ERROR},
    };
    // Only the first entry (no error) is in range.
    TEST_ASSERT_FALSE(anyCanIdHasError(errors, 1));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_anyCanIdHasError_false_when_empty);
    RUN_TEST(test_anyCanIdHasError_false_when_none_active);
    RUN_TEST(test_anyCanIdHasError_true_when_one_active);
    RUN_TEST(test_anyCanIdHasError_ignores_entries_past_count);
    return UNITY_END();
}
