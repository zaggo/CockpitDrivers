#include <unity.h>
#include "CompassGeometry.h"

void setUp(void) {}
void tearDown(void) {}

// --- step position normalisation -------------------------------------------

void test_positions_inside_one_turn_pass_through(void) {
    TEST_ASSERT_EQUAL_UINT32(0, normalizeStepPosition(0, 4096));
    TEST_ASSERT_EQUAL_UINT32(1024, normalizeStepPosition(1024, 4096));
    TEST_ASSERT_EQUAL_UINT32(4095, normalizeStepPosition(4095, 4096));
}

void test_a_full_turn_folds_onto_zero(void) {
    TEST_ASSERT_EQUAL_UINT32(0, normalizeStepPosition(4096, 4096));
    TEST_ASSERT_EQUAL_UINT32(904, normalizeStepPosition(5000, 4096));
}

// getPosition() is int32_t and can go negative after a counter-clockwise move.
// Taking that at face value would index off the far end of the card.
void test_negative_positions_fold_forward(void) {
    TEST_ASSERT_EQUAL_UINT32(4095, normalizeStepPosition(-1, 4096));
    TEST_ASSERT_EQUAL_UINT32(3072, normalizeStepPosition(-1024, 4096));
}

void test_zero_total_steps_cannot_divide_by_zero(void) {
    TEST_ASSERT_EQUAL_UINT32(0, normalizeStepPosition(500, 0));
}

// --- degrees to steps -------------------------------------------------------

void test_cardinal_headings_map_to_quarter_turns(void) {
    TEST_ASSERT_EQUAL_UINT32(0, degreesToSteps(0.0, 4096));
    TEST_ASSERT_EQUAL_UINT32(1024, degreesToSteps(90.0, 4096));
    TEST_ASSERT_EQUAL_UINT32(2048, degreesToSteps(180.0, 4096));
    TEST_ASSERT_EQUAL_UINT32(3072, degreesToSteps(270.0, 4096));
}

void test_a_full_turn_of_heading_is_zero(void) {
    TEST_ASSERT_EQUAL_UINT32(0, degreesToSteps(360.0, 4096));
}

// compass_heading_deg_mag can come through slightly out of range; an unwrapped
// negative would otherwise cast into a position near the far end of the card.
void test_out_of_range_headings_wrap(void) {
    TEST_ASSERT_EQUAL_UINT32(3072, degreesToSteps(-90.0, 4096));
    TEST_ASSERT_EQUAL_UINT32(1024, degreesToSteps(450.0, 4096));
    TEST_ASSERT_EQUAL_UINT32(1024, degreesToSteps(-270.0, 4096));
}

// Rounding is to the nearest step, so a heading within half a step of due north
// lands on 0 rather than on totalSteps.
void test_headings_just_short_of_north_round_onto_zero(void) {
    TEST_ASSERT_EQUAL_UINT32(4095, degreesToSteps(359.9, 4096));
    TEST_ASSERT_EQUAL_UINT32(0, degreesToSteps(359.99, 4096));
}

void test_a_geared_card_uses_its_own_step_count(void) {
    TEST_ASSERT_EQUAL_UINT32(512, degreesToSteps(90.0, 2048));
    TEST_ASSERT_EQUAL_UINT32(0, degreesToSteps(90.0, 0));
}

// --- shortest path ----------------------------------------------------------

void test_a_quarter_turn_goes_the_direct_way(void) {
    TEST_ASSERT_EQUAL_INT32(1024, shortestPathSteps(0, 1024, 4096));
    TEST_ASSERT_EQUAL_INT32(-1024, shortestPathSteps(1024, 0, 4096));
}

// The whole reason this function exists: a card at 359 degrees moving to 1
// degree must travel two degrees, not 358.
void test_crossing_north_takes_the_short_way(void) {
    TEST_ASSERT_EQUAL_INT32(32, shortestPathSteps(4080, 16, 4096));
    TEST_ASSERT_EQUAL_INT32(-32, shortestPathSteps(16, 4080, 4096));
}

void test_three_quarters_clockwise_becomes_a_quarter_counterclockwise(void) {
    TEST_ASSERT_EQUAL_INT32(-1024, shortestPathSteps(0, 3072, 4096));
}

// Exactly half a turn is equidistant either way. The tie is broken clockwise;
// pinned here so a refactor cannot silently flip it.
void test_exactly_half_a_turn_resolves_clockwise(void) {
    TEST_ASSERT_EQUAL_INT32(2048, shortestPathSteps(0, 2048, 4096));
}

void test_no_move_needed_when_already_on_target(void) {
    TEST_ASSERT_EQUAL_INT32(0, shortestPathSteps(1234, 1234, 4096));
}

void test_unnormalized_inputs_are_folded_first(void) {
    TEST_ASSERT_EQUAL_INT32(10, shortestPathSteps(4106, 20, 4096));
}

void test_shortest_path_on_a_zero_step_card_is_no_move(void) {
    TEST_ASSERT_EQUAL_INT32(0, shortestPathSteps(0, 100, 0));
}

// --- zero adjust ------------------------------------------------------------

void test_small_offsets_pass_through_unchanged(void) {
    TEST_ASSERT_EQUAL_INT16(5, normalizeZeroAdjust(5));
    TEST_ASSERT_EQUAL_INT16(-5, normalizeZeroAdjust(-5));
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(0));
}

void test_full_turns_collapse_to_zero(void) {
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(360));
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(-360));
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(3600));
}

void test_offsets_fold_into_minus180_to_179(void) {
    TEST_ASSERT_EQUAL_INT16(-170, normalizeZeroAdjust(190));
    TEST_ASSERT_EQUAL_INT16(170, normalizeZeroAdjust(-190));
    TEST_ASSERT_EQUAL_INT16(-180, normalizeZeroAdjust(180));
    TEST_ASSERT_EQUAL_INT16(-180, normalizeZeroAdjust(-180));
}

// After homing the card sits at 0 with the stored offset already applied, so a
// calibration jog is added to it, never substituted.
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
    TEST_ASSERT_EQUAL_INT32(90, jogDegreesFromPosition(1024, 4096));
    TEST_ASSERT_EQUAL_INT32(0, jogDegreesFromPosition(0, 4096));
}

// A small backwards nudge comes back from getPosition() as almost a whole turn.
// Storing that at face value would record a ~350 degree offset for a nudge of ten.
void test_a_backward_jog_reads_as_negative_degrees(void) {
    TEST_ASSERT_EQUAL_INT32(-10, jogDegreesFromPosition(4096 - 114, 4096));
}

void test_a_zero_step_card_cannot_divide_by_zero(void) {
    TEST_ASSERT_EQUAL_INT32(0, jogDegreesFromPosition(1024, 0));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_positions_inside_one_turn_pass_through);
    RUN_TEST(test_a_full_turn_folds_onto_zero);
    RUN_TEST(test_negative_positions_fold_forward);
    RUN_TEST(test_zero_total_steps_cannot_divide_by_zero);
    RUN_TEST(test_cardinal_headings_map_to_quarter_turns);
    RUN_TEST(test_a_full_turn_of_heading_is_zero);
    RUN_TEST(test_out_of_range_headings_wrap);
    RUN_TEST(test_headings_just_short_of_north_round_onto_zero);
    RUN_TEST(test_a_geared_card_uses_its_own_step_count);
    RUN_TEST(test_a_quarter_turn_goes_the_direct_way);
    RUN_TEST(test_crossing_north_takes_the_short_way);
    RUN_TEST(test_three_quarters_clockwise_becomes_a_quarter_counterclockwise);
    RUN_TEST(test_exactly_half_a_turn_resolves_clockwise);
    RUN_TEST(test_no_move_needed_when_already_on_target);
    RUN_TEST(test_unnormalized_inputs_are_folded_first);
    RUN_TEST(test_shortest_path_on_a_zero_step_card_is_no_move);
    RUN_TEST(test_small_offsets_pass_through_unchanged);
    RUN_TEST(test_full_turns_collapse_to_zero);
    RUN_TEST(test_offsets_fold_into_minus180_to_179);
    RUN_TEST(test_a_jog_is_added_to_the_stored_offset);
    RUN_TEST(test_repeated_calibration_passes_converge_instead_of_drifting);
    RUN_TEST(test_accumulated_offsets_never_exceed_half_a_turn);
    RUN_TEST(test_a_forward_jog_reads_as_positive_degrees);
    RUN_TEST(test_a_backward_jog_reads_as_negative_degrees);
    RUN_TEST(test_a_zero_step_card_cannot_divide_by_zero);
    return UNITY_END();
}
