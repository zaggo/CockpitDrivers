#include <unity.h>
#include "HSICalibration.h"

void setUp(void) {}
void tearDown(void) {}

// --- needle zero adjust -----------------------------------------------------

void test_small_values_pass_through_unchanged(void) {
    TEST_ASSERT_EQUAL_INT16(5, normalizeZeroAdjust(5));
    TEST_ASSERT_EQUAL_INT16(-5, normalizeZeroAdjust(-5));
}

void test_full_turns_collapse_to_zero(void) {
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(360));
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(-3600));
}

void test_values_fold_into_minus180_to_179(void) {
    TEST_ASSERT_EQUAL_INT16(-170, normalizeZeroAdjust(190));
    TEST_ASSERT_EQUAL_INT16(170, normalizeZeroAdjust(-190));
}

// After homing an axis sits at 0 with the stored offset applied, so a jog onto
// the true mark adds to the offset. Replacing it throws away every earlier pass
// and makes the needle wander a little further each time.
void test_a_jog_is_added_to_the_stored_offset(void) {
    TEST_ASSERT_EQUAL_INT16(8, accumulateZeroAdjust(5, 3));
    TEST_ASSERT_EQUAL_INT16(2, accumulateZeroAdjust(5, -3));
}

void test_the_shipped_hsi_defaults_survive_a_round_trip(void) {
    TEST_ASSERT_EQUAL_INT16(-90, normalizeZeroAdjust(-90));
    TEST_ASSERT_EQUAL_INT16(85, normalizeZeroAdjust(85));
    TEST_ASSERT_EQUAL_INT16(11, normalizeZeroAdjust(11));
}

void test_a_forward_jog_reads_as_positive_degrees(void) {
    TEST_ASSERT_EQUAL_INT32(10, jogDegreesFromPosition(114, 4096));
}

void test_a_backward_jog_reads_as_negative_degrees(void) {
    TEST_ASSERT_EQUAL_INT32(-10, jogDegreesFromPosition(4096 - 114, 4096));
}

// hd2 lands on 22 steps (22.75 truncated) = 1.93 degrees; that is a 2 degree
// jog, not 1.
void test_a_step_truncated_jog_rounds_to_the_commanded_degree(void) {
    TEST_ASSERT_EQUAL_INT32(2, jogDegreesFromPosition(22, 4096));
    TEST_ASSERT_EQUAL_INT32(-2, jogDegreesFromPosition(4096 - 22, 4096));
}

void test_a_zero_step_axis_cannot_divide_by_zero(void) {
    TEST_ASSERT_EQUAL_INT32(0, jogDegreesFromPosition(100, 0));
}

// --- servo travel -----------------------------------------------------------
//
// clamp, then scale by the linkage ratio, then add the mechanical offset -
// the order the AirManager firmware used, kept so the shipped defaults land
// the servos exactly where they land today.

void test_the_shipped_cdi_servo_mapping_is_preserved(void) {
    // cdiServo: ratio 2.0, offset 80 deg, travel +-40
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 80.0, servoTargetDegree(0.0, -40.0, 40.0, 2000, 8000));
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 100.0, servoTargetDegree(10.0, -40.0, 40.0, 2000, 8000));
}

void test_the_gs2_servo_negative_ratio_is_preserved(void) {
    // gs2Servo: ratio -10/7, offset 89 deg
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 89.0, servoTargetDegree(0.0, -40.0, 40.0, -1429, 8900));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 74.71, servoTargetDegree(10.0, -40.0, 40.0, -1429, 8900));
}

void test_travel_is_clamped_before_scaling_not_after(void) {
    // Clamping after scaling would let a ratio of 2 turn a +-40 limit into
    // +-80: at the limit this returns 40*2 + 80 = 160, where scale-first
    // would return 80 clamped to 40, then + 80 = 120.
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 160.0, servoTargetDegree(40.0, -40.0, 40.0, 2000, 8000));
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 160.0, servoTargetDegree(400.0, -40.0, 40.0, 2000, 8000));
}

void test_the_fromto_servo_passes_its_discrete_positions_through(void) {
    // fromToServo: ratio 1.0, offset 0, effectively unlimited travel
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.0, servoTargetDegree(0.0, -999.0, 999.0, 1000, 0));
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 90.0, servoTargetDegree(90.0, -999.0, 999.0, 1000, 0));
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 182.0, servoTargetDegree(182.0, -999.0, 999.0, 1000, 0));
}

// --- deviation dots ---------------------------------------------------------

void test_the_shipped_ten_degrees_per_dot_is_preserved(void) {
    // The LUA multiplied nav1_hdef_dot by 10 before handing it over.
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 10.0, dotsToServoDegree(1000, 1000));
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 25.0, dotsToServoDegree(2500, 1000));
}

void test_a_negative_deviation_keeps_its_sign(void) {
    TEST_ASSERT_DOUBLE_WITHIN(0.001, -10.0, dotsToServoDegree(-1000, 1000));
}

void test_a_recalibrated_factor_scales_the_whole_range(void) {
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 8.0, dotsToServoDegree(1000, 800));
}

void test_a_zero_factor_parks_the_needle(void) {
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.0, dotsToServoDegree(2500, 0));
}

// --- GS servo table ---------------------------------------------------------
//
// Each GS servo is driven from a taught table of (dots, percent) points:
// percent is the absolute servo deflection around the 50 % centre, so the
// sign and the linkage ratio live in the table, not in code.

void test_gs_percent_maps_minus100_to_100_onto_the_full_servo_range(void) {
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 90.0, gsPercentToServoDegree(0.0));
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.0, gsPercentToServoDegree(-100.0));
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 180.0, gsPercentToServoDegree(100.0));
}

void test_gs_table_interpolates_between_neighbouring_points(void) {
    const GsPoint pts[] = { {-2000, -60}, {0, 5}, {2000, 70} };
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 5.0, interpolateGsPercent(pts, 3, 0));
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 37.5, interpolateGsPercent(pts, 3, 1000));
    TEST_ASSERT_DOUBLE_WITHIN(0.001, -27.5, interpolateGsPercent(pts, 3, -1000));
}

void test_gs_table_outermost_points_are_soft_endstops(void) {
    const GsPoint pts[] = { {-2000, -60}, {0, 5}, {2000, 70} };
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 70.0, interpolateGsPercent(pts, 3, 3500));
    TEST_ASSERT_DOUBLE_WITHIN(0.001, -60.0, interpolateGsPercent(pts, 3, -9000));
}

void test_gs_table_with_no_points_centres_the_servo(void) {
    TEST_ASSERT_DOUBLE_WITHIN(0.001, 0.0, interpolateGsPercent(nullptr, 0, 1500));
}

void test_gs_table_with_one_point_holds_that_point(void) {
    const GsPoint pts[] = { {500, -20} };
    TEST_ASSERT_DOUBLE_WITHIN(0.001, -20.0, interpolateGsPercent(pts, 1, 1500));
    TEST_ASSERT_DOUBLE_WITHIN(0.001, -20.0, interpolateGsPercent(pts, 1, -1500));
}

void test_gs_points_are_inserted_sorted_by_dots(void) {
    GsPoint pts[4];
    uint8_t count = 0;
    TEST_ASSERT_TRUE(insertGsPoint(pts, &count, 4, 1000, 40));
    TEST_ASSERT_TRUE(insertGsPoint(pts, &count, 4, -1000, -40));
    TEST_ASSERT_TRUE(insertGsPoint(pts, &count, 4, 0, 0));
    TEST_ASSERT_EQUAL_UINT8(3, count);
    TEST_ASSERT_EQUAL_INT16(-1000, pts[0].dots1000);
    TEST_ASSERT_EQUAL_INT16(0, pts[1].dots1000);
    TEST_ASSERT_EQUAL_INT16(1000, pts[2].dots1000);
    TEST_ASSERT_EQUAL_INT8(40, pts[2].percent);
}

void test_a_gs_point_at_the_same_dots_replaces_the_old_one(void) {
    GsPoint pts[4];
    uint8_t count = 0;
    insertGsPoint(pts, &count, 4, 1000, 40);
    TEST_ASSERT_TRUE(insertGsPoint(pts, &count, 4, 1000, 45));
    TEST_ASSERT_EQUAL_UINT8(1, count);
    TEST_ASSERT_EQUAL_INT8(45, pts[0].percent);
}

void test_a_full_gs_table_refuses_a_new_point(void) {
    GsPoint pts[2];
    uint8_t count = 0;
    insertGsPoint(pts, &count, 2, 0, 0);
    insertGsPoint(pts, &count, 2, 1000, 10);
    TEST_ASSERT_FALSE(insertGsPoint(pts, &count, 2, 2000, 20));
    TEST_ASSERT_EQUAL_UINT8(2, count);
    // ... but still lets an existing dots value be retaught.
    TEST_ASSERT_TRUE(insertGsPoint(pts, &count, 2, 1000, 15));
    TEST_ASSERT_EQUAL_INT8(15, pts[1].percent);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_small_values_pass_through_unchanged);
    RUN_TEST(test_full_turns_collapse_to_zero);
    RUN_TEST(test_values_fold_into_minus180_to_179);
    RUN_TEST(test_a_jog_is_added_to_the_stored_offset);
    RUN_TEST(test_the_shipped_hsi_defaults_survive_a_round_trip);
    RUN_TEST(test_a_forward_jog_reads_as_positive_degrees);
    RUN_TEST(test_a_backward_jog_reads_as_negative_degrees);
    RUN_TEST(test_a_step_truncated_jog_rounds_to_the_commanded_degree);
    RUN_TEST(test_a_zero_step_axis_cannot_divide_by_zero);
    RUN_TEST(test_the_shipped_cdi_servo_mapping_is_preserved);
    RUN_TEST(test_the_gs2_servo_negative_ratio_is_preserved);
    RUN_TEST(test_travel_is_clamped_before_scaling_not_after);
    RUN_TEST(test_the_fromto_servo_passes_its_discrete_positions_through);
    RUN_TEST(test_the_shipped_ten_degrees_per_dot_is_preserved);
    RUN_TEST(test_a_negative_deviation_keeps_its_sign);
    RUN_TEST(test_a_recalibrated_factor_scales_the_whole_range);
    RUN_TEST(test_a_zero_factor_parks_the_needle);
    RUN_TEST(test_gs_percent_maps_minus100_to_100_onto_the_full_servo_range);
    RUN_TEST(test_gs_table_interpolates_between_neighbouring_points);
    RUN_TEST(test_gs_table_outermost_points_are_soft_endstops);
    RUN_TEST(test_gs_table_with_no_points_centres_the_servo);
    RUN_TEST(test_gs_table_with_one_point_holds_that_point);
    RUN_TEST(test_gs_points_are_inserted_sorted_by_dots);
    RUN_TEST(test_a_gs_point_at_the_same_dots_replaces_the_old_one);
    RUN_TEST(test_a_full_gs_table_refuses_a_new_point);
    return UNITY_END();
}
