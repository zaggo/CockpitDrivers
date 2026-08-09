#include <unity.h>
#include "AxisMapping.h"

void setUp(void) {}
void tearDown(void) {}

// A representative small-swing hall sensor: 512 centre, +-150 counts of travel.
static const int32_t kCenter   = 512 * 64;
static const int32_t kMax      = 662 * 64;
static const int32_t kMin      = 362 * 64;
static const int32_t kDeadband =  12 * 64;

void test_rawToQ6_scales_by_64(void) {
    TEST_ASSERT_EQUAL_INT32(0, rawToQ6(0));
    TEST_ASSERT_EQUAL_INT32(65472, rawToQ6(1023));
}

void test_centre_deadband_reports_zero(void) {
    TEST_ASSERT_EQUAL_INT16(0, mapRudderQ6(kCenter, kMin, kCenter, kMax, kDeadband));
    TEST_ASSERT_EQUAL_INT16(0, mapRudderQ6(kCenter + kDeadband, kMin, kCenter, kMax, kDeadband));
    TEST_ASSERT_EQUAL_INT16(0, mapRudderQ6(kCenter - kDeadband, kMin, kCenter, kMax, kDeadband));
}

void test_endstops_reach_full_deflection(void) {
    TEST_ASSERT_EQUAL_INT16( 1000, mapRudderQ6(kMax, kMin, kCenter, kMax, kDeadband));
    TEST_ASSERT_EQUAL_INT16(-1000, mapRudderQ6(kMin, kMin, kCenter, kMax, kDeadband));
}

void test_inverted_wiring_still_maps_positive_towards_max(void) {
    // Sensor wired so the raw value falls as the pedal is pushed right.
    const int32_t invMax = 362 * 64;
    const int32_t invMin = 662 * 64;
    TEST_ASSERT_EQUAL_INT16( 1000, mapRudderQ6(invMax, invMin, kCenter, invMax, kDeadband));
    TEST_ASSERT_EQUAL_INT16(-1000, mapRudderQ6(invMin, invMin, kCenter, invMax, kDeadband));
}

void test_unipolar_spans_zero_to_thousand(void) {
    const int32_t lo = 100 * 64;
    const int32_t hi = 900 * 64;
    TEST_ASSERT_EQUAL_UINT16(   0, mapUnipolarQ6(lo, lo, hi));
    TEST_ASSERT_EQUAL_UINT16(1000, mapUnipolarQ6(hi, lo, hi));
    TEST_ASSERT_EQUAL_UINT16( 500, mapUnipolarQ6((lo + hi) / 2, lo, hi));
}

void test_zero_span_calibration_does_not_divide_by_zero(void) {
    TEST_ASSERT_EQUAL_UINT16(0, mapUnipolarQ6(5000, 5000, 5000));
    TEST_ASSERT_EQUAL_UINT16(0, mapHalfQ6(5000, 5000, 5000, kDeadband));
}

void test_sub_lsb_input_change_moves_the_wire_value(void) {
    // A quarter of an ADC count apart. Before the Q6 conversion both inputs
    // rounded to the same raw count and produced an identical wire value.
    const int16_t a = mapRudderQ6(35000, kMin, kCenter, kMax, kDeadband);
    const int16_t b = mapRudderQ6(35016, kMin, kCenter, kMax, kDeadband);
    TEST_ASSERT_NOT_EQUAL(a, b);
    TEST_ASSERT_TRUE(b > a);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_rawToQ6_scales_by_64);
    RUN_TEST(test_centre_deadband_reports_zero);
    RUN_TEST(test_endstops_reach_full_deflection);
    RUN_TEST(test_inverted_wiring_still_maps_positive_towards_max);
    RUN_TEST(test_unipolar_spans_zero_to_thousand);
    RUN_TEST(test_zero_span_calibration_does_not_divide_by_zero);
    RUN_TEST(test_sub_lsb_input_change_moves_the_wire_value);
    return UNITY_END();
}
