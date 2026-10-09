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

// --- signed degrees to steps -----------------------------------------------

void test_signed_degrees_round_to_the_nearest_step(void) {
    TEST_ASSERT_EQUAL_INT32(0, signedDegreesToSteps(0.0, 4096));
    TEST_ASSERT_EQUAL_INT32(1024, signedDegreesToSteps(90.0, 4096));
    TEST_ASSERT_EQUAL_INT32(-1024, signedDegreesToSteps(-90.0, 4096));
    // 0.05 degrees is 0.57 steps: rounds to 1, and symmetrically to -1.
    TEST_ASSERT_EQUAL_INT32(1, signedDegreesToSteps(0.05, 4096));
    TEST_ASSERT_EQUAL_INT32(-1, signedDegreesToSteps(-0.05, 4096));
    TEST_ASSERT_EQUAL_INT32(0, signedDegreesToSteps(90.0, 0));
}

// --- pitch window and roll sense -------------------------------------------

// X-Plane hands out +-90 while a failed vacuum gyro tumbles. The mechanism
// stops at +-30, so the clamp has to happen BEFORE the scale factor.
void test_pitch_is_clamped_before_scaling(void) {
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 30.0 * 0.76923077, pitchDisplayDegree(90.0, 30.0, 0.76923077));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -30.0 * 0.76923077, pitchDisplayDegree(-90.0, 30.0, 0.76923077));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 10.0 * 0.76923077, pitchDisplayDegree(10.0, 30.0, 0.76923077));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, pitchDisplayDegree(0.0, 30.0, 0.76923077));
}

void test_roll_is_inverted_only_when_asked(void) {
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -20.0, rollDisplayDegree(20.0, true));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 20.0, rollDisplayDegree(20.0, false));
}

// --- nested axes ------------------------------------------------------------

// Roll at zero: the pitch motor IS the pitch display.
void test_pitch_motor_equals_display_when_roll_is_zero(void) {
    TEST_ASSERT_EQUAL_UINT32(100, pitchMotorTarget(100, 0, 4096));
    TEST_ASSERT_EQUAL_UINT32(4096 - 100, pitchMotorTarget(-100, 0, 4096));
}

// The pitch motor rides on the roll frame, so holding the display still over
// a full roll turn means the pitch motor counter-rotates exactly one turn.
void test_holding_pitch_over_a_full_roll_counter_rotates_the_pitch_motor(void) {
    int32_t pitchMotorTravel = 0;
    uint32_t motorPos = pitchMotorTarget(0, 0, 4096);
    for (uint32_t rollPos = 64; rollPos <= 4096; rollPos += 64) {
        const uint32_t next = pitchMotorTarget(0, rollPos % 4096, 4096);
        pitchMotorTravel += shortestPathSteps(motorPos, next, 4096);
        motorPos = next;
    }
    TEST_ASSERT_EQUAL_INT32(-4096, pitchMotorTravel);
}

// Roll crossing zero (359 -> 1 degree) with pitch held must not make the pitch
// motor jump most of a turn.
void test_roll_crossing_zero_keeps_the_pitch_motor_delta_small(void) {
    const uint32_t before = pitchMotorTarget(200, 4085, 4096);
    const uint32_t after = pitchMotorTarget(200, 12, 4096);
    TEST_ASSERT_EQUAL_INT32(-23, shortestPathSteps(before, after, 4096));
}

// getPosition() can be negative; a negative display request too.
void test_pitch_motor_target_folds_negative_display_steps(void) {
    TEST_ASSERT_EQUAL_UINT32(4096 - 300, pitchMotorTarget(-100, 200, 4096));
    TEST_ASSERT_EQUAL_UINT32(0, pitchMotorTarget(-100, 200, 0));
}

void test_display_steps_are_recovered_from_motor_and_roll(void) {
    TEST_ASSERT_EQUAL_INT32(100, pitchDisplaySteps(100, 0, 4096));
    TEST_ASSERT_EQUAL_INT32(100, pitchDisplaySteps(4096 - 100, 200, 4096));
    TEST_ASSERT_EQUAL_INT32(-100, pitchDisplaySteps(-300, 200, 4096));
    TEST_ASSERT_EQUAL_INT32(0, pitchDisplaySteps(50, 0, 0));
}

// --- coordinated delays -----------------------------------------------------

void test_equal_travel_keeps_both_axes_at_their_max_rate(void) {
    uint32_t dr = 0, dp = 0;
    coordinatedDelays(100, 100, 1000, 1200, &dr, &dp);
    TEST_ASSERT_EQUAL_UINT32(1000, dr);
    TEST_ASSERT_EQUAL_UINT32(1200, dp);
}

// The axis with more steps sets the pace; the other is slowed so that both
// arrive together. Sign of the travel is irrelevant.
void test_the_shorter_axis_is_slowed_proportionally(void) {
    uint32_t dr = 0, dp = 0;
    coordinatedDelays(200, -100, 1000, 1200, &dr, &dp);
    TEST_ASSERT_EQUAL_UINT32(1000, dr);
    TEST_ASSERT_EQUAL_UINT32(2000, dp);

    coordinatedDelays(-50, 150, 1000, 1200, &dr, &dp);
    TEST_ASSERT_EQUAL_UINT32(3600, dr);
    TEST_ASSERT_EQUAL_UINT32(1200, dp);
}

void test_an_idle_axis_does_not_divide_by_zero(void) {
    uint32_t dr = 0, dp = 0;
    coordinatedDelays(0, 150, 1000, 1200, &dr, &dp);
    TEST_ASSERT_EQUAL_UINT32(1000, dr);
    TEST_ASSERT_EQUAL_UINT32(1200, dp);
    coordinatedDelays(150, 0, 1000, 1200, &dr, &dp);
    TEST_ASSERT_EQUAL_UINT32(1000, dr);
    TEST_ASSERT_EQUAL_UINT32(1200, dp);
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
    RUN_TEST(test_signed_degrees_round_to_the_nearest_step);
    RUN_TEST(test_pitch_is_clamped_before_scaling);
    RUN_TEST(test_roll_is_inverted_only_when_asked);
    RUN_TEST(test_pitch_motor_equals_display_when_roll_is_zero);
    RUN_TEST(test_holding_pitch_over_a_full_roll_counter_rotates_the_pitch_motor);
    RUN_TEST(test_roll_crossing_zero_keeps_the_pitch_motor_delta_small);
    RUN_TEST(test_pitch_motor_target_folds_negative_display_steps);
    RUN_TEST(test_display_steps_are_recovered_from_motor_and_roll);
    RUN_TEST(test_equal_travel_keeps_both_axes_at_their_max_rate);
    RUN_TEST(test_the_shorter_axis_is_slowed_proportionally);
    RUN_TEST(test_an_idle_axis_does_not_divide_by_zero);
    return UNITY_END();
}
