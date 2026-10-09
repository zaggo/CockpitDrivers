#include <unity.h>
#include "TurnCoordinatorCalibration.h"

void setUp(void) {}
void tearDown(void) {}

// Turn rate needle after a bench walkthrough: the left mark (pt-90), the
// level mark (pt0) and the right mark (pt90). The level mark sits mid-dial,
// nowhere near the mechanical home stop at step 0.
static CalibrationTable turnRateTable(void) {
    CalibrationTable table;
    calibrationWipe(table);
    calibrationSet(table, -90, 300);
    calibrationSet(table, 0, 1920);
    calibrationSet(table, 90, 3540);
    return table;
}

// Slip ball: the old AirManager LUA shows the dial is not symmetric about
// the centre (0.084 of travel on one side, 0.072 on the other), so the two
// halves get different step spans.
static CalibrationTable slipTable(void) {
    CalibrationTable table;
    calibrationWipe(table);
    calibrationSet(table, -8, 0);
    calibrationSet(table, 0, 950);
    calibrationSet(table, 8, 1764);
    return table;
}

void test_wipe_leaves_no_points_at_all(void) {
    CalibrationTable table;
    calibrationWipe(table);
    calibrationSet(table, 90, 3540); // garbage from a previous session
    calibrationWipe(table);
    TEST_ASSERT_EQUAL_UINT8(0, table.count);
}

void test_points_are_kept_sorted_regardless_of_entry_order(void) {
    CalibrationTable table;
    calibrationWipe(table);
    TEST_ASSERT_TRUE(calibrationSet(table, 90, 3540));
    TEST_ASSERT_TRUE(calibrationSet(table, -90, 300));
    TEST_ASSERT_TRUE(calibrationSet(table, 0, 1920));
    TEST_ASSERT_EQUAL_UINT8(3, table.count);
    TEST_ASSERT_EQUAL_INT16(-90, table.points[0].value);
    TEST_ASSERT_EQUAL_UINT16(300, table.points[0].step);
    TEST_ASSERT_EQUAL_INT16(0, table.points[1].value);
    TEST_ASSERT_EQUAL_UINT16(1920, table.points[1].step);
    TEST_ASSERT_EQUAL_INT16(90, table.points[2].value);
}

void test_setting_a_known_value_replaces_its_step(void) {
    CalibrationTable table = turnRateTable();
    TEST_ASSERT_TRUE(calibrationSet(table, -90, 320));
    TEST_ASSERT_EQUAL_UINT8(3, table.count);
    TEST_ASSERT_EQUAL_UINT16(320, table.points[0].step);
}

void test_a_full_table_rejects_further_points(void) {
    CalibrationTable table;
    calibrationWipe(table);
    for (int16_t i = 0; i < (int16_t)kMaxCalibrationPoints; i++) {
        TEST_ASSERT_TRUE(calibrationSet(table, (int16_t)(i * 20 - 120), (uint16_t)(i * 300)));
    }
    TEST_ASSERT_EQUAL_UINT8(kMaxCalibrationPoints, table.count);
    TEST_ASSERT_FALSE(calibrationSet(table, 300, 3800));
    TEST_ASSERT_EQUAL_UINT8(kMaxCalibrationPoints, table.count);
}

void test_a_full_table_still_accepts_a_replacement(void) {
    CalibrationTable table;
    calibrationWipe(table);
    for (int16_t i = 0; i < (int16_t)kMaxCalibrationPoints; i++) {
        calibrationSet(table, (int16_t)(i * 20 - 120), (uint16_t)(i * 300));
    }
    TEST_ASSERT_TRUE(calibrationSet(table, -120, 11));
    TEST_ASSERT_EQUAL_UINT16(11, table.points[0].step);
}

void test_min_and_max_report_the_outer_points(void) {
    CalibrationTable table = turnRateTable();
    TEST_ASSERT_EQUAL_INT16(-90, calibrationMinValue(table));
    TEST_ASSERT_EQUAL_INT16(90, calibrationMaxValue(table));
}

void test_min_and_max_of_an_empty_table_are_zero(void) {
    CalibrationTable table;
    calibrationWipe(table);
    TEST_ASSERT_EQUAL_INT16(0, calibrationMinValue(table));
    TEST_ASSERT_EQUAL_INT16(0, calibrationMaxValue(table));
}

void test_calibrated_points_map_to_their_own_step(void) {
    CalibrationTable table = turnRateTable();
    TEST_ASSERT_EQUAL_UINT16(300, calibrationStepFor(table, -90.));
    TEST_ASSERT_EQUAL_UINT16(1920, calibrationStepFor(table, 0.));
    TEST_ASSERT_EQUAL_UINT16(3540, calibrationStepFor(table, 90.));
}

void test_values_between_two_points_interpolate_linearly(void) {
    CalibrationTable table = turnRateTable();
    // -45 sits halfway from -90 to 0: 300 + 0.5 * 1620.
    TEST_ASSERT_EQUAL_UINT16(1110, calibrationStepFor(table, -45.));
    // 45 sits halfway from 0 to 90: 1920 + 0.5 * 1620.
    TEST_ASSERT_EQUAL_UINT16(2730, calibrationStepFor(table, 45.));
}

void test_interpolation_works_across_the_sign_change(void) {
    CalibrationTable table;
    calibrationWipe(table);
    calibrationSet(table, -90, 300);
    calibrationSet(table, 90, 3540);
    // No point at 0 - the segment spans the sign change: 300 + 0.5 * 3240.
    TEST_ASSERT_EQUAL_UINT16(1920, calibrationStepFor(table, 0.));
    TEST_ASSERT_EQUAL_UINT16(1110, calibrationStepFor(table, -45.));
}

void test_turn_rate_past_the_outer_mark_parks_on_it(void) {
    // X-Plane hands out well over 110 degrees in a spin; the dial has no marks
    // out there, so the needle stays on the last calibrated point.
    CalibrationTable table = turnRateTable();
    TEST_ASSERT_EQUAL_UINT16(3540, calibrationStepFor(table, 91.));
    TEST_ASSERT_EQUAL_UINT16(3540, calibrationStepFor(table, 200.));
    TEST_ASSERT_EQUAL_UINT16(300, calibrationStepFor(table, -91.));
    TEST_ASSERT_EQUAL_UINT16(300, calibrationStepFor(table, -200.));
}

void test_slip_table_interpolates_each_side_on_its_own_slope(void) {
    CalibrationTable table = slipTable();
    // Left half spans 950 steps over 8 degrees, right half 814. -4 and +4 must
    // land on their own half's slope, not on a mirrored one.
    TEST_ASSERT_EQUAL_UINT16(475, calibrationStepFor(table, -4.));   // 0 + 0.5 * 950
    TEST_ASSERT_EQUAL_UINT16(1357, calibrationStepFor(table, 4.));   // 950 + 0.5 * 814
    // Sub-degree input works too: 950 + (0.5 / 8) * 814 = 1000.875 -> 1001.
    TEST_ASSERT_EQUAL_UINT16(1001, calibrationStepFor(table, 0.5));
}

void test_zero_is_not_the_home_step(void) {
    CalibrationTable table = turnRateTable();
    TEST_ASSERT_EQUAL_UINT16(1920, calibrationStepFor(table, 0.));
}

void test_an_uncalibrated_table_holds_the_needle_on_the_home_stop(void) {
    CalibrationTable table;
    calibrationWipe(table);
    TEST_ASSERT_EQUAL_UINT16(0, calibrationStepFor(table, 0.));
    TEST_ASSERT_EQUAL_UINT16(0, calibrationStepFor(table, 90.));
    TEST_ASSERT_EQUAL_UINT16(0, calibrationStepFor(table, -90.));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_wipe_leaves_no_points_at_all);
    RUN_TEST(test_points_are_kept_sorted_regardless_of_entry_order);
    RUN_TEST(test_setting_a_known_value_replaces_its_step);
    RUN_TEST(test_a_full_table_rejects_further_points);
    RUN_TEST(test_a_full_table_still_accepts_a_replacement);
    RUN_TEST(test_min_and_max_report_the_outer_points);
    RUN_TEST(test_min_and_max_of_an_empty_table_are_zero);
    RUN_TEST(test_calibrated_points_map_to_their_own_step);
    RUN_TEST(test_values_between_two_points_interpolate_linearly);
    RUN_TEST(test_interpolation_works_across_the_sign_change);
    RUN_TEST(test_turn_rate_past_the_outer_mark_parks_on_it);
    RUN_TEST(test_slip_table_interpolates_each_side_on_its_own_slope);
    RUN_TEST(test_zero_is_not_the_home_step);
    RUN_TEST(test_an_uncalibrated_table_holds_the_needle_on_the_home_stop);
    return UNITY_END();
}
