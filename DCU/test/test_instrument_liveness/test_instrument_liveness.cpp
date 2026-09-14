#include <unity.h>
#include "InstrumentLiveness.h"

void setUp(void) {}
void tearDown(void) {}

void test_a_node_that_never_reported_is_not_silent(void) {
    uint16_t seen = 0;
    uint16_t alive = 0;
    TEST_ASSERT_FALSE(anyKnownInstrumentSilent(seen, alive));
    TEST_ASSERT_EQUAL_UINT16(0, silentInstruments(seen, alive));
}

void test_a_node_that_reported_and_is_alive_is_not_silent(void) {
    uint16_t seen = 0;
    uint16_t alive = 0;
    instrumentMarkSeen(seen, 7);
    instrumentSetAlive(alive, 7, true);
    TEST_ASSERT_FALSE(anyKnownInstrumentSilent(seen, alive));
}

void test_a_node_that_reported_and_went_quiet_is_silent(void) {
    uint16_t seen = 0;
    uint16_t alive = 0;
    instrumentMarkSeen(seen, 7);
    instrumentSetAlive(alive, 7, true);
    instrumentSetAlive(alive, 7, false);
    TEST_ASSERT_TRUE(anyKnownInstrumentSilent(seen, alive));
    TEST_ASSERT_EQUAL_UINT16(1u << 7, silentInstruments(seen, alive));
}

void test_one_silent_node_does_not_hide_the_others(void) {
    uint16_t seen = 0;
    uint16_t alive = 0;
    instrumentMarkSeen(seen, 2);
    instrumentMarkSeen(seen, 9);
    instrumentSetAlive(alive, 2, true);
    instrumentSetAlive(alive, 9, false);
    TEST_ASSERT_EQUAL_UINT16(1u << 9, silentInstruments(seen, alive));
}

void test_alive_can_be_asserted_and_cleared_repeatedly(void) {
    uint16_t alive = 0;
    instrumentSetAlive(alive, 3, true);
    TEST_ASSERT_TRUE(instrumentIsAlive(alive, 3));
    instrumentSetAlive(alive, 3, false);
    TEST_ASSERT_FALSE(instrumentIsAlive(alive, 3));
    instrumentSetAlive(alive, 3, true);
    TEST_ASSERT_TRUE(instrumentIsAlive(alive, 3));
}

// A malformed heartbeat could carry any nodeId byte. It must not shift past
// the width of the mask, which is undefined behaviour.
void test_node_ids_past_the_mask_width_are_ignored(void) {
    uint16_t seen = 0;
    uint16_t alive = 0;
    instrumentMarkSeen(seen, 16);
    instrumentMarkSeen(seen, 200);
    instrumentSetAlive(alive, 16, true);
    TEST_ASSERT_EQUAL_UINT16(0, seen);
    TEST_ASSERT_EQUAL_UINT16(0, alive);
    TEST_ASSERT_FALSE(instrumentIsAlive(alive, 16));
    TEST_ASSERT_EQUAL_UINT16(0, nodeBit(16));
}

void test_every_valid_node_gets_its_own_bit(void) {
    for (uint8_t nodeId = 0; nodeId < kLivenessMaxNodes; nodeId++) {
        TEST_ASSERT_EQUAL_UINT16((uint16_t)(1u << nodeId), nodeBit(nodeId));
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_a_node_that_never_reported_is_not_silent);
    RUN_TEST(test_a_node_that_reported_and_is_alive_is_not_silent);
    RUN_TEST(test_a_node_that_reported_and_went_quiet_is_silent);
    RUN_TEST(test_one_silent_node_does_not_hide_the_others);
    RUN_TEST(test_alive_can_be_asserted_and_cleared_repeatedly);
    RUN_TEST(test_node_ids_past_the_mask_width_are_ignored);
    RUN_TEST(test_every_valid_node_gets_its_own_bit);
    return UNITY_END();
}
