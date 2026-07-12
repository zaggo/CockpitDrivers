#include <unity.h>
#include "Heartbeat.h"

void setUp(void) {}
void tearDown(void) {}

void test_heartbeatAlive_false_when_never_seen(void) {
    TEST_ASSERT_FALSE(heartbeatAlive(0, 1000, 1500));
}

void test_heartbeatAlive_true_within_timeout(void) {
    TEST_ASSERT_TRUE(heartbeatAlive(1000, 2000, 1500));
}

void test_heartbeatAlive_false_after_timeout(void) {
    TEST_ASSERT_FALSE(heartbeatAlive(1000, 3000, 1500));
}

void test_heartbeatAlive_true_across_millis_rollover(void) {
    // lastSeen just before rollover, now just after: unsigned wraparound
    // must still read as a small, in-timeout gap.
    uint32_t lastSeen = 0xFFFFFFF0u;
    uint32_t now = 10u; // wraps to a gap of 26ms
    TEST_ASSERT_TRUE(heartbeatAlive(lastSeen, now, 1500));
}

void test_isStale_false_before_first_send(void) {
    TEST_ASSERT_FALSE(isStale(0, 10000, 5000));
}

void test_isStale_false_when_recent(void) {
    TEST_ASSERT_FALSE(isStale(1000, 2000, 5000));
}

void test_isStale_true_when_overdue(void) {
    TEST_ASSERT_TRUE(isStale(1000, 6000, 5000));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_heartbeatAlive_false_when_never_seen);
    RUN_TEST(test_heartbeatAlive_true_within_timeout);
    RUN_TEST(test_heartbeatAlive_false_after_timeout);
    RUN_TEST(test_heartbeatAlive_true_across_millis_rollover);
    RUN_TEST(test_isStale_false_before_first_send);
    RUN_TEST(test_isStale_false_when_recent);
    RUN_TEST(test_isStale_true_when_overdue);
    return UNITY_END();
}
