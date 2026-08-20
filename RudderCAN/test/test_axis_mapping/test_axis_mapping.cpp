#include <unity.h>
#include "AxisMapping.h"

void setUp(void) {}
void tearDown(void) {}

// A representative small-swing hall sensor: 512 centre, +-150 counts of travel.
static const int32_t kCenter = 512 * 64;
static const int32_t kMax    = 662 * 64;
static const int32_t kMin    = 362 * 64;

// Deadbands are percentages of the travel they sit in; these mirror the shipped
// values in Configuration.h in shape, not in value. 8% of this fixture's 150
// count half travel is exactly 12 raw counts, which is what this axis used to
// carry as an absolute constant — so the assertions below stayed valid across
// the move to proportional deadbands.
static const uint8_t kCenterPercent = 8;
static const uint8_t kEndPercent    = 5;
static const int32_t kDeadband      = 12 * 64;

void test_rawToQ6_scales_by_64(void) {
    TEST_ASSERT_EQUAL_INT32(0, rawToQ6(0));
    TEST_ASSERT_EQUAL_INT32(65472, rawToQ6(1023));
}

void test_centre_deadband_reports_zero(void) {
    TEST_ASSERT_EQUAL_INT16(0, mapRudderQ6(kCenter, kMin, kCenter, kMax, kCenterPercent, kEndPercent));
    TEST_ASSERT_EQUAL_INT16(0, mapRudderQ6(kCenter + kDeadband, kMin, kCenter, kMax, kCenterPercent, kEndPercent));
    TEST_ASSERT_EQUAL_INT16(0, mapRudderQ6(kCenter - kDeadband, kMin, kCenter, kMax, kCenterPercent, kEndPercent));
}

void test_endstops_reach_full_deflection(void) {
    TEST_ASSERT_EQUAL_INT16( 1000, mapRudderQ6(kMax, kMin, kCenter, kMax, kCenterPercent, kEndPercent));
    TEST_ASSERT_EQUAL_INT16(-1000, mapRudderQ6(kMin, kMin, kCenter, kMax, kCenterPercent, kEndPercent));
}

void test_inverted_wiring_still_maps_positive_towards_max(void) {
    // Sensor wired so the raw value falls as the pedal is pushed right.
    const int32_t invMax = 362 * 64;
    const int32_t invMin = 662 * 64;
    TEST_ASSERT_EQUAL_INT16( 1000, mapRudderQ6(invMax, invMin, kCenter, invMax, kCenterPercent, kEndPercent));
    TEST_ASSERT_EQUAL_INT16(-1000, mapRudderQ6(invMin, invMin, kCenter, invMax, kCenterPercent, kEndPercent));
}

void test_unipolar_spans_zero_to_thousand(void) {
    const int32_t lo = 100 * 64;
    const int32_t hi = 900 * 64;
    TEST_ASSERT_EQUAL_UINT16(   0, mapUnipolarQ6(lo, lo, hi, 1));
    TEST_ASSERT_EQUAL_UINT16(1000, mapUnipolarQ6(hi, lo, hi, 1));
    TEST_ASSERT_EQUAL_UINT16( 500, mapUnipolarQ6((lo + hi) / 2, lo, hi, 1));
}

void test_zero_span_calibration_does_not_divide_by_zero(void) {
    TEST_ASSERT_EQUAL_UINT16(0, mapUnipolarQ6(5000, 5000, 5000, 1));
    TEST_ASSERT_EQUAL_UINT16(0, mapHalfQ6(5000, 5000, 5000, kDeadband, kEndPercent));
}

void test_deadbands_wider_than_the_travel_report_zero_not_reversed(void) {
    // Centre and end deadbands summing past 100% leave `from` beyond `to`.
    // Without the guard in mapHalfQ6, mapToWire would divide by a delta of the
    // opposite sign and report a reversed value across the whole travel. Only
    // reachable by misconfiguring the percentages in Configuration.h, which is
    // exactly why the guard is worth keeping.
    const int32_t center = 512 * 64;
    const int32_t end    = 517 * 64;
    const int32_t wideCenterDeadband = deadbandOfSpanQ6(end - center, 60);
    TEST_ASSERT_EQUAL_UINT16(0, mapHalfQ6(end,    center, end, wideCenterDeadband, 50));
    TEST_ASSERT_EQUAL_UINT16(0, mapHalfQ6(center, center, end, wideCenterDeadband, 50));
}

void test_sub_lsb_input_change_moves_the_wire_value(void) {
    // A quarter of an ADC count apart. Before the Q6 conversion both inputs
    // rounded to the same raw count and produced an identical wire value.
    const int16_t a = mapRudderQ6(35000, kMin, kCenter, kMax, kCenterPercent, kEndPercent);
    const int16_t b = mapRudderQ6(35016, kMin, kCenter, kMax, kCenterPercent, kEndPercent);
    TEST_ASSERT_NOT_EQUAL(a, b);
    TEST_ASSERT_TRUE(b > a);
}

void test_brake_end_deadband_percent_sizes_the_dead_zone(void) {
    // Without this, nothing pins the deadband width and a wrong percentage in
    // Configuration.h would pass the suite unnoticed.
    const int32_t lo   = 100 * 64;
    const int32_t hi   = 900 * 64;
    const int32_t onePercent = deadbandOfSpanQ6(hi - lo, 1);

    // Last still-dead input at 1%, and the very next raw count is live.
    TEST_ASSERT_EQUAL_UINT16(0, mapUnipolarQ6(lo + onePercent, lo, hi, 1));
    TEST_ASSERT_TRUE(mapUnipolarQ6(lo + onePercent + 64, lo, hi, 1) > 0);

    // The same input is still dead at 10%, so the percentage really is in play.
    TEST_ASSERT_EQUAL_UINT16(0, mapUnipolarQ6(lo + onePercent + 64, lo, hi, 10));
}

void test_rudder_centre_deadband_percent_sizes_the_dead_zone(void) {
    const int32_t justOutside = kCenter + kDeadband + 64;
    TEST_ASSERT_TRUE(mapRudderQ6(justOutside, kMin, kCenter, kMax, kCenterPercent, kEndPercent) > 0);
    TEST_ASSERT_EQUAL_INT16(0, mapRudderQ6(justOutside, kMin, kCenter, kMax, 20, kEndPercent));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_rawToQ6_scales_by_64);
    RUN_TEST(test_centre_deadband_reports_zero);
    RUN_TEST(test_endstops_reach_full_deflection);
    RUN_TEST(test_inverted_wiring_still_maps_positive_towards_max);
    RUN_TEST(test_unipolar_spans_zero_to_thousand);
    RUN_TEST(test_zero_span_calibration_does_not_divide_by_zero);
    RUN_TEST(test_deadbands_wider_than_the_travel_report_zero_not_reversed);
    RUN_TEST(test_sub_lsb_input_change_moves_the_wire_value);
    RUN_TEST(test_brake_end_deadband_percent_sizes_the_dead_zone);
    RUN_TEST(test_rudder_centre_deadband_percent_sizes_the_dead_zone);
    return UNITY_END();
}
