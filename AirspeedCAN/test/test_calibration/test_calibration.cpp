#include <unity.h>
#include "AirspeedCalibration.h"

void setUp(void) {}
void tearDown(void) {}

// A wiped table plus the two points from the bench walkthrough in the design:
// needle driven to the 40kt mark, cp40, then to the 100kt mark, cp100.
static CalibrationTable twoPointTable(void) {
    CalibrationTable table;
    calibrationWipe(table);
    calibrationSet(table, 40, 480);
    calibrationSet(table, 100, 1200);
    return table;
}

void test_wipe_leaves_only_the_zero_anchor(void) {
    CalibrationTable table;
    calibrationSet(table, 100, 1200); // garbage from a previous session
    calibrationWipe(table);
    TEST_ASSERT_EQUAL_UINT8(1, table.count);
    TEST_ASSERT_EQUAL_UINT16(0, table.points[0].knots);
    TEST_ASSERT_EQUAL_UINT16(0, table.points[0].step);
}

void test_points_are_kept_sorted_regardless_of_entry_order(void) {
    CalibrationTable table;
    calibrationWipe(table);
    TEST_ASSERT_TRUE(calibrationSet(table, 100, 1200));
    TEST_ASSERT_TRUE(calibrationSet(table, 40, 480));
    TEST_ASSERT_EQUAL_UINT8(3, table.count);
    TEST_ASSERT_EQUAL_UINT16(0, table.points[0].knots);
    TEST_ASSERT_EQUAL_UINT16(40, table.points[1].knots);
    TEST_ASSERT_EQUAL_UINT16(480, table.points[1].step);
    TEST_ASSERT_EQUAL_UINT16(100, table.points[2].knots);
}

void test_setting_a_known_knots_value_replaces_its_step(void) {
    CalibrationTable table = twoPointTable();
    TEST_ASSERT_TRUE(calibrationSet(table, 40, 500));
    TEST_ASSERT_EQUAL_UINT8(3, table.count);
    TEST_ASSERT_EQUAL_UINT16(500, table.points[1].step);
}

void test_zero_anchor_can_be_recalibrated(void) {
    CalibrationTable table;
    calibrationWipe(table);
    TEST_ASSERT_TRUE(calibrationSet(table, 0, 24));
    TEST_ASSERT_EQUAL_UINT8(1, table.count);
    TEST_ASSERT_EQUAL_UINT16(24, table.points[0].step);
}

void test_a_full_table_rejects_further_points(void) {
    CalibrationTable table;
    calibrationWipe(table);
    for (uint16_t i = 1; i < kMaxCalibrationPoints; i++) {
        TEST_ASSERT_TRUE(calibrationSet(table, i * 10, i * 100));
    }
    TEST_ASSERT_EQUAL_UINT8(kMaxCalibrationPoints, table.count);
    TEST_ASSERT_FALSE(calibrationSet(table, 999, 3000));
    TEST_ASSERT_EQUAL_UINT8(kMaxCalibrationPoints, table.count);
}

void test_a_full_table_still_accepts_a_replacement(void) {
    CalibrationTable table;
    calibrationWipe(table);
    for (uint16_t i = 1; i < kMaxCalibrationPoints; i++) {
        calibrationSet(table, i * 10, i * 100);
    }
    TEST_ASSERT_TRUE(calibrationSet(table, 10, 111));
    TEST_ASSERT_EQUAL_UINT16(111, table.points[1].step);
}

void test_calibrated_points_map_to_their_own_step(void) {
    CalibrationTable table = twoPointTable();
    TEST_ASSERT_EQUAL_UINT16(480, calibrationStepFor(table, 40.));
    TEST_ASSERT_EQUAL_UINT16(1200, calibrationStepFor(table, 100.));
}

void test_speeds_between_two_points_interpolate_linearly(void) {
    CalibrationTable table = twoPointTable();
    // 55kt sits a quarter of the way from 40 to 100: 480 + 0.25 * 720.
    TEST_ASSERT_EQUAL_UINT16(660, calibrationStepFor(table, 55.));
    TEST_ASSERT_EQUAL_UINT16(840, calibrationStepFor(table, 70.));
}

void test_speeds_below_the_first_segment_interpolate_towards_zero(void) {
    CalibrationTable table = twoPointTable();
    TEST_ASSERT_EQUAL_UINT16(240, calibrationStepFor(table, 20.));
}

void test_speeds_above_the_last_point_park_on_that_point(void) {
    CalibrationTable table = twoPointTable();
    TEST_ASSERT_EQUAL_UINT16(1200, calibrationStepFor(table, 101.));
    TEST_ASSERT_EQUAL_UINT16(1200, calibrationStepFor(table, 500.));
}

void test_zero_and_negative_speeds_park_on_the_zero_anchor(void) {
    CalibrationTable table = twoPointTable();
    calibrationSet(table, 0, 24);
    TEST_ASSERT_EQUAL_UINT16(24, calibrationStepFor(table, 0.));
    TEST_ASSERT_EQUAL_UINT16(24, calibrationStepFor(table, -5.));
}

void test_an_uncalibrated_table_holds_the_needle_at_the_anchor(void) {
    CalibrationTable table;
    calibrationWipe(table);
    TEST_ASSERT_EQUAL_UINT16(0, calibrationStepFor(table, 0.));
    TEST_ASSERT_EQUAL_UINT16(0, calibrationStepFor(table, 120.));
}

void test_the_highest_calibrated_speed_is_reported(void) {
    CalibrationTable table = twoPointTable();
    TEST_ASSERT_EQUAL_UINT16(100, calibrationMaxKnots(table));
    calibrationWipe(table);
    TEST_ASSERT_EQUAL_UINT16(0, calibrationMaxKnots(table));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_wipe_leaves_only_the_zero_anchor);
    RUN_TEST(test_points_are_kept_sorted_regardless_of_entry_order);
    RUN_TEST(test_setting_a_known_knots_value_replaces_its_step);
    RUN_TEST(test_zero_anchor_can_be_recalibrated);
    RUN_TEST(test_a_full_table_rejects_further_points);
    RUN_TEST(test_a_full_table_still_accepts_a_replacement);
    RUN_TEST(test_calibrated_points_map_to_their_own_step);
    RUN_TEST(test_speeds_between_two_points_interpolate_linearly);
    RUN_TEST(test_speeds_below_the_first_segment_interpolate_towards_zero);
    RUN_TEST(test_speeds_above_the_last_point_park_on_that_point);
    RUN_TEST(test_zero_and_negative_speeds_park_on_the_zero_anchor);
    RUN_TEST(test_an_uncalibrated_table_holds_the_needle_at_the_anchor);
    RUN_TEST(test_the_highest_calibrated_speed_is_reported);
    return UNITY_END();
}
