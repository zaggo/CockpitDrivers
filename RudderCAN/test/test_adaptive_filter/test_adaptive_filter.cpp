#include <unity.h>
#include "AdaptiveFilter.h"

void setUp(void) {}
void tearDown(void) {}

// One raw ADC count in Q6.
static const int32_t kQ6 = 64;

void test_reset_seeds_exactly(void) {
    AdaptiveFilter f;
    f.reset(500);
    TEST_ASSERT_TRUE(f.seeded());
    TEST_ASSERT_EQUAL_INT32(500 * kQ6, f.valueQ6());
}

void test_first_update_seeds_without_ramp(void) {
    AdaptiveFilter f;
    TEST_ASSERT_FALSE(f.seeded());
    f.update(500);
    TEST_ASSERT_TRUE(f.seeded());
    TEST_ASSERT_EQUAL_INT32(500 * kQ6, f.valueQ6());
}

void test_small_step_converges_without_overshoot(void) {
    // Deliberately a 5 LSB step, small enough that alpha stays off its ceiling
    // and the convergence behaviour is actually exercised.
    AdaptiveFilter f;
    f.reset(500);
    int32_t previous = f.valueQ6();
    for (int i = 0; i < 200; i++) {
        f.update(505);
        // Monotonic rise, and never past the target.
        TEST_ASSERT_TRUE(f.valueQ6() >= previous);
        TEST_ASSERT_TRUE(f.valueQ6() <= 505 * kQ6);
        previous = f.valueQ6();
    }
    // Settled to within one LSB of the target.
    TEST_ASSERT_TRUE((505 * kQ6) - f.valueQ6() < kQ6);
}

void test_alpha_saturates_on_full_scale_jump(void) {
    // A 0 -> 1023 step drives alpha to its 256/256 ceiling, so the filter is
    // transparent: the estimate must land exactly on the input, with no overflow.
    AdaptiveFilter f;
    f.reset(0);
    f.update(1023);
    TEST_ASSERT_EQUAL_INT32(1023 * kQ6, f.valueQ6());
}

void test_noise_is_attenuated_below_one_lsb(void) {
    AdaptiveFilter f;
    f.reset(512);
    for (int i = 0; i < 100; i++) {
        f.update((i % 2) ? 514 : 510);
    }
    int32_t lo = f.valueQ6();
    int32_t hi = f.valueQ6();
    for (int i = 0; i < 100; i++) {
        f.update((i % 2) ? 514 : 510);
        if (f.valueQ6() < lo) lo = f.valueQ6();
        if (f.valueQ6() > hi) hi = f.valueQ6();
    }
    // +-2 LSB of input dither must collapse to under 1 LSB of output span.
    TEST_ASSERT_TRUE((hi - lo) < kQ6);
}

void test_ramp_tracking_error_stays_small(void) {
    // 4 LSB per sample at 500 Hz is a full-travel sweep in about half a second.
    AdaptiveFilter f;
    f.reset(0);
    uint16_t raw = 0;
    for (int i = 0; i < 50; i++) {
        raw = (uint16_t)(raw + 4);
        f.update(raw);
    }
    const int32_t errorLsb = ((int32_t)raw * kQ6 - f.valueQ6()) / kQ6;
    TEST_ASSERT_TRUE(errorLsb >= 0);
    TEST_ASSERT_TRUE(errorLsb < 12);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_reset_seeds_exactly);
    RUN_TEST(test_first_update_seeds_without_ramp);
    RUN_TEST(test_small_step_converges_without_overshoot);
    RUN_TEST(test_alpha_saturates_on_full_scale_jump);
    RUN_TEST(test_noise_is_attenuated_below_one_lsb);
    RUN_TEST(test_ramp_tracking_error_stays_small);
    return UNITY_END();
}
