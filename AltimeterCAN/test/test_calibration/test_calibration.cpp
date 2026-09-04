#include <unity.h>
#include "AltimeterCalibration.h"

void setUp(void) {}
void tearDown(void) {}

// --- zero adjust ------------------------------------------------------------

void test_small_values_pass_through_unchanged(void) {
    TEST_ASSERT_EQUAL_INT16(5, normalizeZeroAdjust(5));
    TEST_ASSERT_EQUAL_INT16(-5, normalizeZeroAdjust(-5));
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(0));
}

void test_full_turns_collapse_to_zero(void) {
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(360));
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(-360));
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(3600));
}

void test_values_fold_into_minus180_to_179(void) {
    TEST_ASSERT_EQUAL_INT16(-170, normalizeZeroAdjust(190));
    TEST_ASSERT_EQUAL_INT16(170, normalizeZeroAdjust(-190));
    TEST_ASSERT_EQUAL_INT16(-180, normalizeZeroAdjust(180));
    TEST_ASSERT_EQUAL_INT16(-180, normalizeZeroAdjust(-180));
}

// The point of the whole exercise: after homing the axis sits at 0 with the
// stored offset already applied, so a jog is added to it, not substituted.
void test_a_jog_is_added_to_the_stored_offset(void) {
    TEST_ASSERT_EQUAL_INT16(8, accumulateZeroAdjust(5, 3));
    TEST_ASSERT_EQUAL_INT16(2, accumulateZeroAdjust(5, -3));
}

void test_repeated_calibration_passes_converge_instead_of_drifting(void) {
    int16_t stored = 5;
    stored = accumulateZeroAdjust(stored, 3);   // 8
    stored = accumulateZeroAdjust(stored, -1);  // 7
    stored = accumulateZeroAdjust(stored, 0);   // 7
    TEST_ASSERT_EQUAL_INT16(7, stored);
}

void test_accumulated_offsets_never_exceed_half_a_turn(void) {
    int16_t stored = 170;
    stored = accumulateZeroAdjust(stored, 20);
    TEST_ASSERT_EQUAL_INT16(-170, stored);
}

// --- position to jog --------------------------------------------------------

void test_a_forward_jog_reads_as_positive_degrees(void) {
    // 4096 steps per turn, so a quarter turn is 1024 steps = 90 degrees.
    TEST_ASSERT_EQUAL_INT32(90, jogDegreesFromPosition(1024, 4096));
    TEST_ASSERT_EQUAL_INT32(0, jogDegreesFromPosition(0, 4096));
}

// getPosition() normalises into 0..total-1, so a small backwards jog comes back
// as almost a whole turn. Taking that at face value would store a ~350 degree
// offset for a nudge of ten.
void test_a_backward_jog_reads_as_negative_degrees(void) {
    TEST_ASSERT_EQUAL_INT32(-90, jogDegreesFromPosition(4096 - 1024, 4096));
    TEST_ASSERT_EQUAL_INT32(-1, jogDegreesFromPosition(4096 - 12, 4096));
}

void test_a_zero_step_axis_cannot_divide_by_zero(void) {
    TEST_ASSERT_EQUAL_INT32(0, jogDegreesFromPosition(100, 0));
}

// --- barometer --------------------------------------------------------------

static BaroCalibration defaultBaro(void) {
    BaroCalibration c;
    baroCalibrationDefaults(c, 0, 2810, 1023, 3100);
    return c;
}

void test_the_calibrated_endpoints_map_to_their_own_values(void) {
    BaroCalibration c = defaultBaro();
    TEST_ASSERT_EQUAL_UINT16(2810, baroInHg100(c, 0));
    TEST_ASSERT_EQUAL_UINT16(3100, baroInHg100(c, 1023));
}

void test_the_midpoint_interpolates_linearly(void) {
    BaroCalibration c;
    baroCalibrationDefaults(c, 0, 2800, 1000, 3000);
    TEST_ASSERT_EQUAL_UINT16(2900, baroInHg100(c, 500));
    TEST_ASSERT_EQUAL_UINT16(2850, baroInHg100(c, 250));
}

void test_raw_values_outside_the_endpoints_are_clamped(void) {
    BaroCalibration c;
    baroCalibrationDefaults(c, 100, 2800, 900, 3000);
    TEST_ASSERT_EQUAL_UINT16(2800, baroInHg100(c, 0));
    TEST_ASSERT_EQUAL_UINT16(2800, baroInHg100(c, 100));
    TEST_ASSERT_EQUAL_UINT16(3000, baroInHg100(c, 1023));
}

// A pot wired the other way round gives a falling raw value for rising
// pressure. Calibration should still work rather than clamp everything.
void test_a_reversed_pot_still_interpolates(void) {
    BaroCalibration c;
    baroCalibrationDefaults(c, 1000, 2800, 0, 3000);
    TEST_ASSERT_EQUAL_UINT16(2800, baroInHg100(c, 1000));
    TEST_ASSERT_EQUAL_UINT16(3000, baroInHg100(c, 0));
    TEST_ASSERT_EQUAL_UINT16(2900, baroInHg100(c, 500));
    TEST_ASSERT_EQUAL_UINT16(2800, baroInHg100(c, 1023));
}

// Both points taught at the same pot position would divide by zero.
void test_two_points_at_the_same_raw_value_do_not_divide_by_zero(void) {
    BaroCalibration c;
    baroCalibrationDefaults(c, 500, 2900, 500, 3100);
    TEST_ASSERT_EQUAL_UINT16(2900, baroInHg100(c, 0));
    TEST_ASSERT_EQUAL_UINT16(2900, baroInHg100(c, 500));
    TEST_ASSERT_EQUAL_UINT16(2900, baroInHg100(c, 1023));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_small_values_pass_through_unchanged);
    RUN_TEST(test_full_turns_collapse_to_zero);
    RUN_TEST(test_values_fold_into_minus180_to_179);
    RUN_TEST(test_a_jog_is_added_to_the_stored_offset);
    RUN_TEST(test_repeated_calibration_passes_converge_instead_of_drifting);
    RUN_TEST(test_accumulated_offsets_never_exceed_half_a_turn);
    RUN_TEST(test_a_forward_jog_reads_as_positive_degrees);
    RUN_TEST(test_a_backward_jog_reads_as_negative_degrees);
    RUN_TEST(test_a_zero_step_axis_cannot_divide_by_zero);
    RUN_TEST(test_the_calibrated_endpoints_map_to_their_own_values);
    RUN_TEST(test_the_midpoint_interpolates_linearly);
    RUN_TEST(test_raw_values_outside_the_endpoints_are_clamped);
    RUN_TEST(test_a_reversed_pot_still_interpolates);
    RUN_TEST(test_two_points_at_the_same_raw_value_do_not_divide_by_zero);
    return UNITY_END();
}
