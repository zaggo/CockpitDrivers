#include "AxisCurve.h"
#include <cmath>
#include <cstdio>
#include <limits>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w) { ++g_checks; if (!c) { ++g_failures; std::printf("  FAIL: %s\n", w); } }
static bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) <= eps; }

// Sweep -1..1 and verify output never decreases and is always finite and in range.
static bool monotonicAndFinite(const AxisCurve& c) {
    float prev = -2.f;
    for (int i = 0; i <= 400; ++i) {
        float x = -1.f + 2.f * i / 400.f;
        float y = c.apply(x);
        if (std::isnan(y) || y < -1.f || y > 1.f) return false;
        if (y < prev - 1e-6f) return false;
        prev = y;
    }
    return true;
}

int main() {
    // Defaults are the identity.
    {
        AxisCurve c;
        check(near(c.apply(0.f), 0.f), "identity: 0 -> 0");
        check(near(c.apply(0.5f), 0.5f), "identity: 0.5 -> 0.5");
        check(near(c.apply(-0.25f), -0.25f), "identity: -0.25 -> -0.25");
        check(near(c.apply(1.f), 1.f) && near(c.apply(-1.f), -1.f), "identity: endpoints");
    }

    // Out-of-range and NaN input is clamped (Review Focus 1).
    {
        AxisCurve c;
        check(near(c.apply(3.f), 1.f), "input 3 clamps to 1");
        check(near(c.apply(-7.f), -1.f), "input -7 clamps to -1");
        check(near(c.apply(std::numeric_limits<float>::quiet_NaN()), 0.f), "NaN input -> 0");
    }

    // Sensitivity -1 is cubic (flat centre), +1 is cube root (steep centre).
    {
        AxisCurve c; c.sensPos = -1.f; c.sensNeg = -1.f;
        check(near(c.apply(0.5f), 0.125f), "sens -1: 0.5 -> 0.125 (x^3)");
        check(near(c.apply(-0.5f), -0.125f), "sens -1: -0.5 -> -0.125");
        check(near(c.apply(1.f), 1.f), "sens -1 still reaches 1");
        c.sensPos = 1.f; c.sensNeg = 1.f;
        check(near(c.apply(0.5f), std::cbrt(0.5f)), "sens +1: 0.5 -> cbrt(0.5)");
        check(near(c.apply(-0.125f), -0.5f), "sens +1: -0.125 -> -0.5");
    }

    // Sensitivity halves are independent.
    {
        AxisCurve c; c.sensPos = -1.f; c.sensNeg = 0.f;
        check(near(c.apply(0.5f), 0.125f), "sensPos only affects positive half");
        check(near(c.apply(-0.5f), -0.5f), "negative half stays linear");
    }

    // Dead zone: zero inside, continuous at the edge, still reaches 1.
    {
        AxisCurve c; c.deadZone = 0.2f;
        check(near(c.apply(0.1f), 0.f), "inside dead zone -> 0");
        check(near(c.apply(-0.2f), 0.f), "dead zone edge -> 0");
        check(near(c.apply(0.21f), 0.0125f, 1e-3f), "just past dead zone is tiny, not a jump");
        check(near(c.apply(0.6f), 0.5f), "dead zone rescales: 0.6 -> 0.5");
        check(near(c.apply(1.f), 1.f), "dead zone still reaches 1");
    }

    // Extremity dead zone: saturates early, continuous.
    {
        AxisCurve c; c.extremityDeadZone = 0.2f;
        check(near(c.apply(0.8f), 1.f), "extremity edge -> 1");
        check(near(c.apply(0.9f), 1.f), "inside extremity -> 1");
        check(near(c.apply(-0.95f), -1.f), "negative extremity -> -1");
        check(near(c.apply(0.4f), 0.5f), "extremity rescales: 0.4 -> 0.5");
    }

    // Neutral shifts the zero point; stops still reach +-1.
    {
        AxisCurve c; c.neutral = 0.2f;
        check(near(c.apply(0.2f), 0.f), "neutral 0.2: raw 0.2 -> 0");
        check(near(c.apply(1.f), 1.f), "neutral 0.2: raw 1 -> 1");
        check(near(c.apply(-1.f), -1.f), "neutral 0.2: raw -1 -> -1");
        check(near(c.apply(0.6f), 0.5f), "neutral 0.2: raw 0.6 -> 0.5 (half of remaining span)");
        check(near(c.apply(-0.4f), -0.5f), "neutral 0.2: raw -0.4 -> -0.5");
    }

    // Neutral at the stop must not divide by zero (Review Focus 3).
    {
        AxisCurve c; c.neutral = 1.f;
        check(monotonicAndFinite(c), "neutral = 1: finite and monotonic");
        c.neutral = -1.f;
        check(monotonicAndFinite(c), "neutral = -1: finite and monotonic");
    }

    // Overlapping dead zones (Review Focus 4).
    {
        AxisCurve c; c.deadZone = 0.5f; c.extremityDeadZone = 0.5f;
        check(monotonicAndFinite(c), "dz 0.5 + ext 0.5: finite and monotonic");
        check(near(c.apply(0.f), 0.f) && near(c.apply(1.f), 1.f), "dz 0.5 + ext 0.5: 0 -> 0, 1 -> 1");
        c.deadZone = 1.f; c.extremityDeadZone = 1.f;
        check(monotonicAndFinite(c), "dz 1 + ext 1: still finite");
    }

    // Mixed parameters: monotonic end to end.
    {
        AxisCurve c; c.sensPos = -0.35f; c.sensNeg = 0.6f; c.neutral = -0.1f; c.deadZone = 0.05f; c.extremityDeadZone = 0.1f;
        check(monotonicAndFinite(c), "mixed params: finite and monotonic");
        check(c.apply(-1.f) == -1.f && c.apply(1.f) == 1.f, "mixed params: stops reach +-1 exactly");
    }

    // clampToRange() sanitises out-of-range and NaN values (Review Focus 2).
    {
        AxisCurve c;
        c.sensPos = 5.f; c.sensNeg = -5.f; c.neutral = std::numeric_limits<float>::quiet_NaN();
        c.deadZone = -1.f; c.extremityDeadZone = 2.f;
        c.clampToRange();
        check(c.sensPos == 1.f && c.sensNeg == -1.f, "clampToRange: sensitivities clamp to +-1");
        check(c.neutral == 0.f, "clampToRange: NaN -> 0");
        check(c.deadZone == 0.f && c.extremityDeadZone == 1.f, "clampToRange: dead zones clamp to 0..1");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
