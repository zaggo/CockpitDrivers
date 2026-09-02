#include <unity.h>
#include "VerticalSpeedCalibration.h"

void setUp(void) {}
void tearDown(void) {}

// A wiped table plus three points from a bench walkthrough: the needle driven
// to the 500fpm descent mark (cp-500), the level mark (cp0) and the 500fpm
// climb mark (cp500). Note the level mark sits at step 900, nowhere near the
// mechanical home stop at step 0.
static CalibrationTable threePointTable(void) {
    CalibrationTable table;
    calibrationWipe(table);
    calibrationSet(table, -500, 300);
    calibrationSet(table, 0, 900);
    calibrationSet(table, 500, 1500);
    return table;
}

void test_wipe_leaves_no_points_at_all(void) {
    CalibrationTable table;
    calibrationWipe(table);
    calibrationSet(table, 500, 1500); // garbage from a previous session
    calibrationWipe(table);
    TEST_ASSERT_EQUAL_UINT8(0, table.count);
}

void test_points_are_kept_sorted_regardless_of_entry_order(void) {
    CalibrationTable table;
    calibrationWipe(table);
    TEST_ASSERT_TRUE(calibrationSet(table, 500, 1500));
    TEST_ASSERT_TRUE(calibrationSet(table, -500, 300));
    TEST_ASSERT_TRUE(calibrationSet(table, 0, 900));
    TEST_ASSERT_EQUAL_UINT8(3, table.count);
    TEST_ASSERT_EQUAL_INT16(-500, table.points[0].fpm);
    TEST_ASSERT_EQUAL_UINT16(300, table.points[0].step);
    TEST_ASSERT_EQUAL_INT16(0, table.points[1].fpm);
    TEST_ASSERT_EQUAL_UINT16(900, table.points[1].step);
    TEST_ASSERT_EQUAL_INT16(500, table.points[2].fpm);
}

void test_setting_a_known_fpm_value_replaces_its_step(void) {
    CalibrationTable table = threePointTable();
    TEST_ASSERT_TRUE(calibrationSet(table, -500, 320));
    TEST_ASSERT_EQUAL_UINT8(3, table.count);
    TEST_ASSERT_EQUAL_UINT16(320, table.points[0].step);
}

void test_a_full_table_rejects_further_points(void) {
    CalibrationTable table;
    calibrationWipe(table);
    for (int16_t i = 0; i < (int16_t)kMaxCalibrationPoints; i++) {
        TEST_ASSERT_TRUE(calibrationSet(table, (int16_t)(i * 100 - 600), (uint16_t)(i * 100)));
    }
    TEST_ASSERT_EQUAL_UINT8(kMaxCalibrationPoints, table.count);
    TEST_ASSERT_FALSE(calibrationSet(table, 3000, 3000));
    TEST_ASSERT_EQUAL_UINT8(kMaxCalibrationPoints, table.count);
}

void test_a_full_table_still_accepts_a_replacement(void) {
    CalibrationTable table;
    calibrationWipe(table);
    for (int16_t i = 0; i < (int16_t)kMaxCalibrationPoints; i++) {
        calibrationSet(table, (int16_t)(i * 100 - 600), (uint16_t)(i * 100));
    }
    TEST_ASSERT_TRUE(calibrationSet(table, -600, 111));
    TEST_ASSERT_EQUAL_UINT16(111, table.points[0].step);
}

void test_calibrated_points_map_to_their_own_step(void) {
    CalibrationTable table = threePointTable();
    TEST_ASSERT_EQUAL_UINT16(300, calibrationStepFor(table, -500.));
    TEST_ASSERT_EQUAL_UINT16(900, calibrationStepFor(table, 0.));
    TEST_ASSERT_EQUAL_UINT16(1500, calibrationStepFor(table, 500.));
}

void test_rates_between_two_points_interpolate_linearly(void) {
    CalibrationTable table = threePointTable();
    // -250fpm sits halfway from -500 to 0: 300 + 0.5 * 600.
    TEST_ASSERT_EQUAL_UINT16(600, calibrationStepFor(table, -250.));
    // 250fpm sits halfway from 0 to 500: 900 + 0.5 * 600.
    TEST_ASSERT_EQUAL_UINT16(1200, calibrationStepFor(table, 250.));
}

void test_interpolation_works_across_the_sign_change(void) {
    CalibrationTable table;
    calibrationWipe(table);
    calibrationSet(table, -500, 300);
    calibrationSet(table, 500, 1500);
    // No point at 0 — the segment spans the sign change: 300 + 0.5 * 1200.
    TEST_ASSERT_EQUAL_UINT16(900, calibrationStepFor(table, 0.));
    TEST_ASSERT_EQUAL_UINT16(600, calibrationStepFor(table, -250.));
}

void test_rates_below_the_lowest_point_park_on_that_point(void) {
    CalibrationTable table = threePointTable();
    TEST_ASSERT_EQUAL_UINT16(300, calibrationStepFor(table, -501.));
    TEST_ASSERT_EQUAL_UINT16(300, calibrationStepFor(table, -5000.));
}

void test_rates_above_the_highest_point_park_on_that_point(void) {
    CalibrationTable table = threePointTable();
    TEST_ASSERT_EQUAL_UINT16(1500, calibrationStepFor(table, 501.));
    TEST_ASSERT_EQUAL_UINT16(1500, calibrationStepFor(table, 5000.));
}

void test_zero_is_not_the_home_step(void) {
    // The whole point of the VSI table: level flight is a taught mark
    // mid-dial, not the mechanical stop the needle homes against.
    CalibrationTable table = threePointTable();
    TEST_ASSERT_EQUAL_UINT16(900, calibrationStepFor(table, 0.));
}

void test_an_uncalibrated_table_holds_the_needle_on_the_home_stop(void) {
    CalibrationTable table;
    calibrationWipe(table);
    TEST_ASSERT_EQUAL_UINT16(0, calibrationStepFor(table, 0.));
    TEST_ASSERT_EQUAL_UINT16(0, calibrationStepFor(table, 1200.));
    TEST_ASSERT_EQUAL_UINT16(0, calibrationStepFor(table, -1200.));
}

void test_a_single_point_holds_the_needle_on_that_point(void) {
    CalibrationTable table;
    calibrationWipe(table);
    calibrationSet(table, 0, 900);
    TEST_ASSERT_EQUAL_UINT16(900, calibrationStepFor(table, -2000.));
    TEST_ASSERT_EQUAL_UINT16(900, calibrationStepFor(table, 0.));
    TEST_ASSERT_EQUAL_UINT16(900, calibrationStepFor(table, 2000.));
}

void test_the_calibrated_range_is_reported(void) {
    CalibrationTable table = threePointTable();
    TEST_ASSERT_EQUAL_INT16(-500, calibrationMinFpm(table));
    TEST_ASSERT_EQUAL_INT16(500, calibrationMaxFpm(table));
    calibrationWipe(table);
    TEST_ASSERT_EQUAL_INT16(0, calibrationMinFpm(table));
    TEST_ASSERT_EQUAL_INT16(0, calibrationMaxFpm(table));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_wipe_leaves_no_points_at_all);
    RUN_TEST(test_points_are_kept_sorted_regardless_of_entry_order);
    RUN_TEST(test_setting_a_known_fpm_value_replaces_its_step);
    RUN_TEST(test_a_full_table_rejects_further_points);
    RUN_TEST(test_a_full_table_still_accepts_a_replacement);
    RUN_TEST(test_calibrated_points_map_to_their_own_step);
    RUN_TEST(test_rates_between_two_points_interpolate_linearly);
    RUN_TEST(test_interpolation_works_across_the_sign_change);
    RUN_TEST(test_rates_below_the_lowest_point_park_on_that_point);
    RUN_TEST(test_rates_above_the_highest_point_park_on_that_point);
    RUN_TEST(test_zero_is_not_the_home_step);
    RUN_TEST(test_an_uncalibrated_table_holds_the_needle_on_the_home_stop);
    RUN_TEST(test_a_single_point_holds_the_needle_on_that_point);
    RUN_TEST(test_the_calibrated_range_is_reported);
    return UNITY_END();
}
