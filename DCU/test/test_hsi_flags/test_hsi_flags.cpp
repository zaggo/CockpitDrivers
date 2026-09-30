#include <unity.h>
#include "HsiWire.h"

void setUp(void) {}
void tearDown(void) {}

// --- flags byte -------------------------------------------------------------
// The plugin calls the same hsiFlagsByte() on its side of the link, so these
// cases pin both ends of the chain.

void test_no_nav_clears_both_direction_bits(void) {
    TEST_ASSERT_EQUAL_UINT8(0x00, hsiFlagsByte(0, 0));
}

void test_to_sets_only_bit0(void) {
    TEST_ASSERT_EQUAL_UINT8(0x01, hsiFlagsByte(1, 0));
}

void test_from_sets_only_bit1(void) {
    TEST_ASSERT_EQUAL_UINT8(0x02, hsiFlagsByte(2, 0));
}

// The old AirManager firmware compared nav1_fromto against 2.4 and so never
// matched FROM at all. Bit 1 must actually appear for the value 2.
void test_from_is_reachable_at_all(void) {
    TEST_ASSERT_TRUE((hsiFlagsByte(2, 0) & 0x02) != 0);
}

void test_glideslope_sets_bit2_independently(void) {
    TEST_ASSERT_EQUAL_UINT8(0x04, hsiFlagsByte(0, 1));
    TEST_ASSERT_EQUAL_UINT8(0x05, hsiFlagsByte(1, 1));
    TEST_ASSERT_EQUAL_UINT8(0x06, hsiFlagsByte(2, 1));
}

void test_any_nonzero_glideslope_value_counts_as_valid(void) {
    TEST_ASSERT_EQUAL_UINT8(0x04, hsiFlagsByte(0, 7));
}

void test_unexpected_fromto_values_are_treated_as_no_nav(void) {
    TEST_ASSERT_EQUAL_UINT8(0x00, hsiFlagsByte(3, 0));
    TEST_ASSERT_EQUAL_UINT8(0x00, hsiFlagsByte(-1, 0));
}

void test_reserved_bits_stay_clear(void) {
    TEST_ASSERT_EQUAL_UINT8(0, hsiFlagsByte(2, 1) & 0xF8);
}

// --- degree wrapping --------------------------------------------------------

void test_plain_degrees_scale_by_one_hundred(void) {
    TEST_ASSERT_EQUAL_UINT16(0, hsiWrapDeg100(0.0f));
    TEST_ASSERT_EQUAL_UINT16(9000, hsiWrapDeg100(90.0f));
    TEST_ASSERT_EQUAL_UINT16(35999, hsiWrapDeg100(359.99f));
}

// X-Plane hands out slightly negative headings and values past 360. An
// unwrapped negative cast to uint16 sends an axis on a full spurious turn.
void test_negative_degrees_wrap_forward(void) {
    TEST_ASSERT_EQUAL_UINT16(35900, hsiWrapDeg100(-1.0f));
    TEST_ASSERT_EQUAL_UINT16(27000, hsiWrapDeg100(-90.0f));
}

void test_degrees_past_a_full_turn_wrap_back(void) {
    TEST_ASSERT_EQUAL_UINT16(1000, hsiWrapDeg100(370.0f));
    TEST_ASSERT_EQUAL_UINT16(0, hsiWrapDeg100(360.0f));
}

void test_a_value_rounding_up_onto_36000_becomes_zero(void) {
    TEST_ASSERT_EQUAL_UINT16(0, hsiWrapDeg100(359.999f));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_no_nav_clears_both_direction_bits);
    RUN_TEST(test_to_sets_only_bit0);
    RUN_TEST(test_from_sets_only_bit1);
    RUN_TEST(test_from_is_reachable_at_all);
    RUN_TEST(test_glideslope_sets_bit2_independently);
    RUN_TEST(test_any_nonzero_glideslope_value_counts_as_valid);
    RUN_TEST(test_unexpected_fromto_values_are_treated_as_no_nav);
    RUN_TEST(test_reserved_bits_stay_clear);
    RUN_TEST(test_plain_degrees_scale_by_one_hundred);
    RUN_TEST(test_negative_degrees_wrap_forward);
    RUN_TEST(test_degrees_past_a_full_turn_wrap_back);
    RUN_TEST(test_a_value_rounding_up_onto_36000_becomes_zero);
    return UNITY_END();
}
