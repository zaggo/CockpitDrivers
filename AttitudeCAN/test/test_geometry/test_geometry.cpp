#include <unity.h>
#include "AttitudeGeometry.h"

void setUp(void) {}
void tearDown(void) {}

// --- step position normalisation -------------------------------------------

void test_positions_inside_one_turn_pass_through(void) {
    TEST_ASSERT_EQUAL_UINT32(0, normalizeStepPosition(0, 4096));
    TEST_ASSERT_EQUAL_UINT32(1024, normalizeStepPosition(1024, 4096));
    TEST_ASSERT_EQUAL_UINT32(4095, normalizeStepPosition(4095, 4096));
}

void test_negative_positions_fold_forward(void) {
    TEST_ASSERT_EQUAL_UINT32(4095, normalizeStepPosition(-1, 4096));
    TEST_ASSERT_EQUAL_UINT32(3072, normalizeStepPosition(-1024, 4096));
}

void test_zero_total_steps_cannot_divide_by_zero(void) {
    TEST_ASSERT_EQUAL_UINT32(0, normalizeStepPosition(500, 0));
}

// --- shortest path ----------------------------------------------------------

void test_a_quarter_turn_goes_the_direct_way(void) {
    TEST_ASSERT_EQUAL_INT32(1024, shortestPathSteps(0, 1024, 4096));
    TEST_ASSERT_EQUAL_INT32(-1024, shortestPathSteps(1024, 0, 4096));
}

void test_crossing_zero_takes_the_short_way(void) {
    // 359 -> 1 degree is 23 steps forward, not 4073 backwards.
    TEST_ASSERT_EQUAL_INT32(23, shortestPathSteps(4085, 12, 4096));
    TEST_ASSERT_EQUAL_INT32(-23, shortestPathSteps(12, 4085, 4096));
}

void test_exactly_half_a_turn_resolves_clockwise(void) {
    TEST_ASSERT_EQUAL_INT32(2048, shortestPathSteps(0, 2048, 4096));
}

void test_no_move_needed_when_already_on_target(void) {
    TEST_ASSERT_EQUAL_INT32(0, shortestPathSteps(777, 777, 4096));
}

// --- homing window ----------------------------------------------------------

void test_a_normal_window_halves_to_its_centre(void) {
    int32_t centre = -1;
    TEST_ASSERT_TRUE(homingWindowCentre(60, 0, 4096, &centre));
    TEST_ASSERT_EQUAL_INT32(30, centre);
}

// The capture frame the state machine uses: end edge reset to 0, start edge
// captured as a negative (folded) position.
void test_a_window_ending_exactly_at_zero_is_handled(void) {
    int32_t centre = -1;
    TEST_ASSERT_TRUE(homingWindowCentre(0, 4066, 4096, &centre));
    TEST_ASSERT_EQUAL_INT32(15, centre);
}

void test_a_wrapped_difference_is_rejected_instead_of_becoming_180_degrees(void) {
    int32_t centre = 4242;
    TEST_ASSERT_FALSE(homingWindowCentre(0, 30, 4096, &centre));
    TEST_ASSERT_EQUAL_INT32(4242, centre);
}

void test_a_window_wider_than_an_eighth_of_a_turn_is_rejected(void) {
    int32_t centre = -1;
    TEST_ASSERT_FALSE(homingWindowCentre(1024, 0, 4096, &centre));
}

// --- zero adjust ------------------------------------------------------------

void test_offsets_fold_into_minus180_to_179(void) {
    TEST_ASSERT_EQUAL_INT16(-170, normalizeZeroAdjust(190));
    TEST_ASSERT_EQUAL_INT16(170, normalizeZeroAdjust(-190));
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(360));
}

void test_a_jog_is_added_to_the_stored_offset(void) {
    TEST_ASSERT_EQUAL_INT16(8, accumulateZeroAdjust(5, 3));
    TEST_ASSERT_EQUAL_INT16(2, accumulateZeroAdjust(5, -3));
}

void test_a_backward_jog_reads_as_negative_degrees(void) {
    TEST_ASSERT_EQUAL_INT32(90, jogDegreesFromPosition(1024, 4096));
    TEST_ASSERT_EQUAL_INT32(-90, jogDegreesFromPosition(4096 - 1024, 4096));
    TEST_ASSERT_EQUAL_INT32(0, jogDegreesFromPosition(100, 0));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_positions_inside_one_turn_pass_through);
    RUN_TEST(test_negative_positions_fold_forward);
    RUN_TEST(test_zero_total_steps_cannot_divide_by_zero);
    RUN_TEST(test_a_quarter_turn_goes_the_direct_way);
    RUN_TEST(test_crossing_zero_takes_the_short_way);
    RUN_TEST(test_exactly_half_a_turn_resolves_clockwise);
    RUN_TEST(test_no_move_needed_when_already_on_target);
    RUN_TEST(test_a_normal_window_halves_to_its_centre);
    RUN_TEST(test_a_window_ending_exactly_at_zero_is_handled);
    RUN_TEST(test_a_wrapped_difference_is_rejected_instead_of_becoming_180_degrees);
    RUN_TEST(test_a_window_wider_than_an_eighth_of_a_turn_is_rejected);
    RUN_TEST(test_offsets_fold_into_minus180_to_179);
    RUN_TEST(test_a_jog_is_added_to_the_stored_offset);
    RUN_TEST(test_a_backward_jog_reads_as_negative_degrees);
    return UNITY_END();
}
