#include <unity.h>
#include "HSIGeometry.h"

void setUp(void) {}
void tearDown(void) {}

// --- axis targets -----------------------------------------------------------
//
// The course pointer and the heading bug ride on the rotating card, so their
// axes hold a difference. Only the card itself is mirrored. Degrees * 100.

void test_north_up_with_everything_aligned_parks_all_axes_at_zero(void) {
    AxisTargets t = axisTargets(0, 0, 0);
    TEST_ASSERT_EQUAL_UINT16(0, t.compassAxis100);
    TEST_ASSERT_EQUAL_UINT16(0, t.obsAxis100);
    TEST_ASSERT_EQUAL_UINT16(0, t.hdgAxis100);
}

void test_the_card_is_mirrored(void) {
    AxisTargets t = axisTargets(9000, 9000, 9000);
    TEST_ASSERT_EQUAL_UINT16(27000, t.compassAxis100);
}

void test_pointers_hold_their_difference_from_the_heading(void) {
    // heading 090, OBS 120, bug 060
    AxisTargets t = axisTargets(9000, 12000, 6000);
    TEST_ASSERT_EQUAL_UINT16(3000, t.obsAxis100);    // 120 - 90
    TEST_ASSERT_EQUAL_UINT16(33000, t.hdgAxis100);   // 60 - 90 -> 330
}

void test_a_pointer_behind_the_heading_wraps_the_long_way_round(void) {
    AxisTargets t = axisTargets(35900, 100, 35900);
    TEST_ASSERT_EQUAL_UINT16(200, t.obsAxis100);     // 1 - 359 -> 2
    TEST_ASSERT_EQUAL_UINT16(0, t.hdgAxis100);
}

void test_the_wrap_at_the_top_of_the_scale(void) {
    AxisTargets t = axisTargets(35999, 0, 0);
    TEST_ASSERT_EQUAL_UINT16(1, t.compassAxis100);   // -(359.99) -> 0.01
    TEST_ASSERT_EQUAL_UINT16(1, t.obsAxis100);
    TEST_ASSERT_EQUAL_UINT16(1, t.hdgAxis100);
}

void test_every_output_stays_inside_the_scale(void) {
    for (uint16_t h = 0; h < 36000; h += 997) {
        for (uint16_t o = 0; o < 36000; o += 1499) {
            AxisTargets t = axisTargets(h, o, o);
            TEST_ASSERT_LESS_THAN_UINT16(36000, t.compassAxis100);
            TEST_ASSERT_LESS_THAN_UINT16(36000, t.obsAxis100);
            TEST_ASSERT_LESS_THAN_UINT16(36000, t.hdgAxis100);
        }
    }
}

// --- homing window centre ---------------------------------------------------
//
// The two window edges are captured in separate phases with a resetPosition()
// in between, so an unsigned subtraction is the natural expression - and the
// trap. A wrapped difference halves to totalSteps/2, i.e. 180 degrees off,
// while still reporting success. WhiskeyCompassCAN hit this twice.

void test_a_normal_window_halves_to_its_centre(void) {
    int32_t centre = -1;
    TEST_ASSERT_TRUE(homingWindowCentre(60, 0, 4096, &centre));
    TEST_ASSERT_EQUAL_INT32(30, centre);
}

void test_a_window_ending_exactly_at_zero_is_handled(void) {
    int32_t centre = -1;
    TEST_ASSERT_TRUE(homingWindowCentre(0, 4066, 4096, &centre));
    TEST_ASSERT_EQUAL_INT32(15, centre);
}

void test_a_window_starting_exactly_at_zero_is_handled(void) {
    int32_t centre = -1;
    TEST_ASSERT_TRUE(homingWindowCentre(30, 0, 4096, &centre));
    TEST_ASSERT_EQUAL_INT32(15, centre);
}

// The one that matters: a reversed pair must be rejected, not silently
// wrapped into a half-turn.
void test_a_wrapped_difference_is_rejected_instead_of_becoming_180_degrees(void) {
    int32_t centre = -1;
    TEST_ASSERT_FALSE(homingWindowCentre(0, 30, 4096, &centre));
}

void test_a_window_wider_than_an_eighth_of_a_turn_is_rejected(void) {
    int32_t centre = -1;
    TEST_ASSERT_FALSE(homingWindowCentre(1024, 0, 4096, &centre));
}

void test_a_window_exactly_at_the_bound_is_still_accepted(void) {
    int32_t centre = -1;
    TEST_ASSERT_TRUE(homingWindowCentre(512, 0, 4096, &centre));
    TEST_ASSERT_EQUAL_INT32(256, centre);
}

void test_a_zero_step_axis_cannot_divide_by_zero(void) {
    int32_t centre = -1;
    TEST_ASSERT_FALSE(homingWindowCentre(10, 0, 0, &centre));
}

void test_a_rejected_window_leaves_the_output_untouched(void) {
    int32_t centre = 4242;
    TEST_ASSERT_FALSE(homingWindowCentre(0, 30, 4096, &centre));
    TEST_ASSERT_EQUAL_INT32(4242, centre);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_north_up_with_everything_aligned_parks_all_axes_at_zero);
    RUN_TEST(test_the_card_is_mirrored);
    RUN_TEST(test_pointers_hold_their_difference_from_the_heading);
    RUN_TEST(test_a_pointer_behind_the_heading_wraps_the_long_way_round);
    RUN_TEST(test_the_wrap_at_the_top_of_the_scale);
    RUN_TEST(test_every_output_stays_inside_the_scale);
    RUN_TEST(test_a_normal_window_halves_to_its_centre);
    RUN_TEST(test_a_window_ending_exactly_at_zero_is_handled);
    RUN_TEST(test_a_window_starting_exactly_at_zero_is_handled);
    RUN_TEST(test_a_wrapped_difference_is_rejected_instead_of_becoming_180_degrees);
    RUN_TEST(test_a_window_wider_than_an_eighth_of_a_turn_is_rejected);
    RUN_TEST(test_a_window_exactly_at_the_bound_is_still_accepted);
    RUN_TEST(test_a_zero_step_axis_cannot_divide_by_zero);
    RUN_TEST(test_a_rejected_window_leaves_the_output_untouched);
    return UNITY_END();
}
