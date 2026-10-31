#include "BrakeEqualizer.h"
#include <cmath>
#include <cstdio>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w) { ++g_checks; if (!c) { ++g_failures; std::printf("  FAIL: %s\n", w); } }
static bool near(float a, float b, float eps = 1e-5f) { return std::fabs(a - b) < eps; }

int main() {
    // dz = 0: exact passthrough, differential braking unaffected by default.
    {
        BrakeEqualizer e;
        float l = 0.3f, r = 0.7f;
        e.apply(l, r);
        check(near(l, 0.3f) && near(r, 0.7f), "dz=0: passthrough");
    }

    // Difference inside the dead zone: both sides output the average.
    {
        BrakeEqualizer e; e.diffDeadZone = 0.2f;
        float l = 0.5f, r = 0.6f;   // d = 0.1 < dz
        e.apply(l, r);
        check(near(l, 0.55f) && near(r, 0.55f), "inside dz: both sides get the average");
    }

    // Exactly at the edge: still equalized (continuity).
    {
        BrakeEqualizer e; e.diffDeadZone = 0.2f;
        float l = 0.4f, r = 0.6f;   // d == dz
        e.apply(l, r);
        check(near(l, 0.5f) && near(r, 0.5f), "at dz edge: still equalized");
    }

    // Just past the edge: differential fades in from zero, no jump.
    {
        BrakeEqualizer e; e.diffDeadZone = 0.2f;
        float l = 0.4f, r = 0.62f;  // d = 0.22 -> eff = 0.025
        e.apply(l, r);
        check(r > l, "past edge: direction of difference kept");
        check(r - l < 0.1f, "past edge: fades in, does not jump to 0.22");
        check(near(0.5f * (l + r), 0.51f), "past edge: average preserved");
    }

    // Full pedal separation still yields the full difference.
    {
        BrakeEqualizer e; e.diffDeadZone = 0.2f;
        float l = 0.f, r = 1.f;     // d = 1 -> eff = 1
        e.apply(l, r);
        check(near(l, 0.f) && near(r, 1.f), "d=1: full differential braking");
    }

    // Symmetric: swapping sides swaps the outputs.
    {
        BrakeEqualizer e; e.diffDeadZone = 0.2f;
        float l = 0.7f, r = 0.5f;
        e.apply(l, r);
        float l2 = 0.5f, r2 = 0.7f;
        e.apply(l2, r2);
        check(near(l, r2) && near(r, l2), "left/right symmetric");
    }

    // The differential output grows monotonically from zero as d crosses dz.
    {
        BrakeEqualizer e; e.diffDeadZone = 0.3f;
        bool monotone = true;
        float prev = -1.f;
        for (int i = 0; i <= 100; ++i) {
            float l = 0.f, r = i / 100.f;   // d = 0..1
            e.apply(l, r);
            const float diff = r - l;
            if (diff < prev - 1e-6f) monotone = false;
            prev = diff;
        }
        check(monotone, "differential fades in monotonically over d");
    }

    // Outputs stay inside 0..1 for every input pair.
    {
        BrakeEqualizer e; e.diffDeadZone = 0.1f;
        bool inRange = true;
        for (int i = 0; i <= 100 && inRange; ++i)
            for (int j = 0; j <= 100; ++j) {
                float l = i / 100.f, r = j / 100.f;
                e.apply(l, r);
                if (l < -1e-6f || l > 1.f + 1e-6f || r < -1e-6f || r > 1.f + 1e-6f) inRange = false;
            }
        check(inRange, "output always within 0..1");
    }

    // Config hardening: out-of-range clamp, NaN safety.
    {
        BrakeEqualizer e; e.diffDeadZone = 5.f;
        e.clampToRange();
        check(near(e.diffDeadZone, 1.f), "diffDeadZone clamped to 1");
        e.diffDeadZone = std::nanf("");
        e.clampToRange();
        check(near(e.diffDeadZone, 0.f), "NaN diffDeadZone -> 0");

        e.diffDeadZone = 0.f;
        float l = std::nanf(""), r = 0.5f;
        e.apply(l, r);
        check(near(l, 0.f) && near(r, 0.5f), "NaN input -> 0");

        // dz = 1 must not divide by zero (the effective dead zone saturates
        // just below 1 so full separation still passes its full difference).
        e.diffDeadZone = 1.f;
        l = 0.f; r = 0.9f;
        e.apply(l, r);
        check(std::isfinite(l) && std::isfinite(r), "dz=1 stays finite");
        check(near(l, r), "dz=1: d below the saturated dead zone equalizes");
        l = 0.f; r = 1.f;
        e.apply(l, r);
        check(near(l, 0.f) && near(r, 1.f), "dz=1: full separation still reaches full difference");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
