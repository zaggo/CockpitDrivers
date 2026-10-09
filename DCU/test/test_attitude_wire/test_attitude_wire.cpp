#include <unity.h>
#include <math.h>
#include "AttitudeWire.h"

void setUp(void) {}
void tearDown(void) {}

void test_degrees_scale_to_hundredths_with_rounding(void) {
    TEST_ASSERT_EQUAL_INT16(0, attitudeDeg100(0.0f));
    TEST_ASSERT_EQUAL_INT16(1234, attitudeDeg100(12.34f));
    TEST_ASSERT_EQUAL_INT16(-1234, attitudeDeg100(-12.34f));
    // 0.005 rounds away from zero on both sides, not towards +inf.
    TEST_ASSERT_EQUAL_INT16(1, attitudeDeg100(0.005f));
    TEST_ASSERT_EQUAL_INT16(-1, attitudeDeg100(-0.005f));
}

// A tumbling vacuum gyro reports +-90 pitch and the full +-180 roll; both fit.
void test_the_full_tumble_range_fits(void) {
    TEST_ASSERT_EQUAL_INT16(9000, attitudeDeg100(90.0f));
    TEST_ASSERT_EQUAL_INT16(-18000, attitudeDeg100(-180.0f));
    TEST_ASSERT_EQUAL_INT16(18000, attitudeDeg100(180.0f));
}

// Garbage from an unloaded aircraft must not wrap the int16 into the other
// sign: 400 degrees would otherwise read as -255.36.
void test_values_past_the_int16_are_clamped(void) {
    TEST_ASSERT_EQUAL_INT16(32767, attitudeDeg100(400.0f));
    TEST_ASSERT_EQUAL_INT16(-32768, attitudeDeg100(-400.0f));
    TEST_ASSERT_EQUAL_INT16(32767, attitudeDeg100(1e9f));
}

// lroundf(NaN) is undefined; a NaN dataref read has to come out as level.
void test_nan_becomes_zero(void) {
    TEST_ASSERT_EQUAL_INT16(0, attitudeDeg100(NAN));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_degrees_scale_to_hundredths_with_rounding);
    RUN_TEST(test_the_full_tumble_range_fits);
    RUN_TEST(test_values_past_the_int16_are_clamped);
    RUN_TEST(test_nan_becomes_zero);
    return UNITY_END();
}
