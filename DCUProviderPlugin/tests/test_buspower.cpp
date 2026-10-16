#include "BusPower.h"
#include <cmath>
#include <cstdio>
#include <limits>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w) { ++g_checks; if (!c) { ++g_failures; std::printf("  FAIL: %s\n", w); } }
static bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) <= eps; }

int main() {
    // A fresh BusPower assumes a healthy bus: pre-brownout behaviour until the
    // first update() proves otherwise.
    {
        BusPower bp;
        check(near(bp.lightFactor(), 1.f), "fresh: lights full");
        check(bp.instrumentsPowered(), "fresh: instruments powered");
    }

    // Full lighting at and above kLightsFullVolts, dark at and below
    // kLightsOutVolts, linear in between.
    {
        BusPower bp;
        bp.update(14.2f);
        check(near(bp.lightFactor(), 1.f), "14.2V (engine running): lights full");
        bp.update(BusPower::kLightsFullVolts);
        check(near(bp.lightFactor(), 1.f), "kLightsFullVolts: lights full");
        bp.update(12.1f);  // measured battery-only
        check(bp.lightFactor() > 0.7f && bp.lightFactor() < 0.8f,
              "12.1V (battery only): lights dimmed to ~75%");
        bp.update(10.75f); // fade midpoint
        check(near(bp.lightFactor(), 0.5f), "10.75V: lights half");
        bp.update(BusPower::kLightsOutVolts);
        check(near(bp.lightFactor(), 0.f), "kLightsOutVolts: lights out");
        bp.update(3.5f);
        check(near(bp.lightFactor(), 0.f), "3.5V: lights out");
    }

    // Instruments stay on through a mild sag and die below kInstrumentsOffVolts.
    {
        BusPower bp;
        bp.update(12.1f);
        check(bp.instrumentsPowered(), "12.1V (battery only): instruments alive");
        bp.update(11.0f);
        check(bp.instrumentsPowered(), "11V: still powered");
        bp.update(9.9f);
        check(!bp.instrumentsPowered(), "9.9V: instruments dead");
        // ...while the lights are still glowing at ~a third.
        check(bp.lightFactor() > 0.3f && bp.lightFactor() < 0.4f,
              "9.9V: lights still glowing after instruments died");
    }

    // Hysteresis: between off and on thresholds the state must not flap.
    {
        BusPower bp;
        bp.update(14.0f);
        bp.update(9.0f);  // dead
        bp.update(10.2f); // above off-threshold, below on-threshold
        check(!bp.instrumentsPowered(), "hysteresis: 10.2V does not revive");
        bp.update(BusPower::kInstrumentsOnVolts);
        check(bp.instrumentsPowered(), "hysteresis: on-threshold revives");
        bp.update(10.2f);
        check(bp.instrumentsPowered(), "hysteresis: 10.2V does not kill");
        bp.update(BusPower::kInstrumentsOffVolts);
        check(!bp.instrumentsPowered(), "hysteresis: off-threshold kills");
    }

    // Power-up restores full lighting together with the instruments.
    {
        BusPower bp;
        bp.update(0.0f);
        check(near(bp.lightFactor(), 0.f) && !bp.instrumentsPowered(), "0V: everything dark and dead");
        bp.update(14.0f);
        check(near(bp.lightFactor(), 1.f) && bp.instrumentsPowered(), "14V after 0V: everything back");
    }

    // Non-finite input keeps the previous state instead of latching anything.
    {
        BusPower bp;
        bp.update(14.0f);
        bp.update(std::numeric_limits<float>::quiet_NaN());
        check(near(bp.lightFactor(), 1.f) && bp.instrumentsPowered(), "NaN keeps healthy state");
        bp.update(0.0f);
        bp.update(-std::numeric_limits<float>::infinity());
        check(near(bp.lightFactor(), 0.f) && !bp.instrumentsPowered(), "-inf keeps dead state");
    }

    std::printf("%s (%d checks, %d failures)\n",
                g_failures ? "FAIL" : "OK", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
