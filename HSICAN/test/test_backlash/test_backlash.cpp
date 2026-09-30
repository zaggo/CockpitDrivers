#include <unity.h>
#include "Backlash.h"

void setUp(void) {}
void tearDown(void) {}

// --- reversal bookkeeping ---------------------------------------------------
//
// unusedBacklashSteps counts how much slack is still to be crossed before the
// output moves again. On a reversal, the slack already travelled in the old
// direction is exactly the slack that does not need crossing in the new one.

void test_a_reversal_from_rest_must_cross_the_whole_slack(void) {
    TEST_ASSERT_EQUAL_UINT32(75, backlashOnReversal(0, 75));
}

void test_a_reversal_part_way_through_the_slack_only_crosses_the_rest(void) {
    // 30 still pending means 45 were travelled, so going back costs 45.
    TEST_ASSERT_EQUAL_UINT32(45, backlashOnReversal(30, 75));
}

void test_reversing_twice_returns_to_where_it_started(void) {
    uint32_t unused = backlashOnReversal(30, 75);   // 45
    unused = backlashOnReversal(unused, 75);        // 30
    TEST_ASSERT_EQUAL_UINT32(30, unused);
}

void test_a_reversal_with_the_full_slack_pending_costs_nothing(void) {
    TEST_ASSERT_EQUAL_UINT32(0, backlashOnReversal(75, 75));
}

void test_an_axis_without_slack_never_accrues_any(void) {
    TEST_ASSERT_EQUAL_UINT32(0, backlashOnReversal(0, 0));
}

// Defensive: a stored value larger than the configured slack (a shrunk EEPROM
// setting, say) must not underflow into a near-billion step move.
void test_a_pending_count_above_the_total_clamps_to_zero(void) {
    TEST_ASSERT_EQUAL_UINT32(0, backlashOnReversal(100, 75));
}

// --- step consumption -------------------------------------------------------

void test_a_step_inside_the_slack_is_eaten_and_the_output_stays_put(void) {
    uint32_t unused = 3;
    TEST_ASSERT_TRUE(consumeBacklashStep(&unused));
    TEST_ASSERT_EQUAL_UINT32(2, unused);
}

void test_a_step_with_no_slack_left_moves_the_output(void) {
    uint32_t unused = 0;
    TEST_ASSERT_FALSE(consumeBacklashStep(&unused));
    TEST_ASSERT_EQUAL_UINT32(0, unused);
}

void test_the_slack_drains_in_exactly_as_many_steps_as_configured(void) {
    uint32_t unused = 4;
    int eaten = 0;
    for (int i = 0; i < 10; i++) {
        if (consumeBacklashStep(&unused)) eaten++;
    }
    TEST_ASSERT_EQUAL_INT(4, eaten);
}

// --- deadband ---------------------------------------------------------------
//
// 0x104 arrives at 50 Hz in hundredths of a degree, so the integer step target
// jitters by +-1 continuously. Without a deadband every flip appends a full
// slack crossing and a standing pointer buzzes back and forth forever.

void test_a_real_move_in_the_same_direction_goes_through(void) {
    TEST_ASSERT_TRUE(moveIsWorthMaking(40, true, 2, 25));
}

void test_a_one_step_jitter_in_the_same_direction_is_suppressed(void) {
    TEST_ASSERT_FALSE(moveIsWorthMaking(1, true, 2, 25));
}

void test_a_one_step_jitter_that_reverses_is_suppressed(void) {
    TEST_ASSERT_FALSE(moveIsWorthMaking(-1, true, 2, 25));
}

void test_a_reversal_must_clear_the_larger_threshold(void) {
    // Ten steps the other way is real movement by the move threshold, but not
    // enough to be worth paying a slack crossing for.
    TEST_ASSERT_FALSE(moveIsWorthMaking(-10, true, 2, 25));
    TEST_ASSERT_TRUE(moveIsWorthMaking(-30, true, 2, 25));
}

void test_a_zero_reversal_deadband_disables_the_behaviour(void) {
    TEST_ASSERT_TRUE(moveIsWorthMaking(-3, true, 2, 0));
}

void test_a_move_of_exactly_the_threshold_is_made(void) {
    TEST_ASSERT_TRUE(moveIsWorthMaking(2, true, 2, 25));
    TEST_ASSERT_TRUE(moveIsWorthMaking(-25, true, 2, 25));
}

void test_a_zero_step_move_is_never_worth_making(void) {
    TEST_ASSERT_FALSE(moveIsWorthMaking(0, true, 0, 0));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_a_reversal_from_rest_must_cross_the_whole_slack);
    RUN_TEST(test_a_reversal_part_way_through_the_slack_only_crosses_the_rest);
    RUN_TEST(test_reversing_twice_returns_to_where_it_started);
    RUN_TEST(test_a_reversal_with_the_full_slack_pending_costs_nothing);
    RUN_TEST(test_an_axis_without_slack_never_accrues_any);
    RUN_TEST(test_a_pending_count_above_the_total_clamps_to_zero);
    RUN_TEST(test_a_step_inside_the_slack_is_eaten_and_the_output_stays_put);
    RUN_TEST(test_a_step_with_no_slack_left_moves_the_output);
    RUN_TEST(test_the_slack_drains_in_exactly_as_many_steps_as_configured);
    RUN_TEST(test_a_real_move_in_the_same_direction_goes_through);
    RUN_TEST(test_a_one_step_jitter_in_the_same_direction_is_suppressed);
    RUN_TEST(test_a_one_step_jitter_that_reverses_is_suppressed);
    RUN_TEST(test_a_reversal_must_clear_the_larger_threshold);
    RUN_TEST(test_a_zero_reversal_deadband_disables_the_behaviour);
    RUN_TEST(test_a_move_of_exactly_the_threshold_is_made);
    RUN_TEST(test_a_zero_step_move_is_never_worth_making);
    return UNITY_END();
}
