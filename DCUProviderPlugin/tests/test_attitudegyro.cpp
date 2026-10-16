#include "AttitudeGyro.h"
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w) { ++g_checks; if (!c) { ++g_failures; std::printf("  FAIL: %s\n", w); } }
static bool near(float a, float b, float eps = 1e-3f) { return std::fabs(a - b) <= eps; }

// Run the model for `seconds` at 50 Hz, like the downlink tick does.
static void run(AttitudeGyro& g, float vacuumInHg, float seconds, float dt = 0.02f) {
    for (float t = 0.f; t < seconds; t += dt)
        g.update(vacuumInHg, dt);
}

int main() {
    // A fresh AttitudeGyro assumes a spinning rotor: fail-live default, same
    // idea as BusPower, so plugin start on a running engine doesn't tumble.
    {
        AttitudeGyro g;
        check(near(g.rotorSpeed(), 1.f), "fresh: rotor at full speed");
        float p = 10.f, r = -5.f;
        g.apply(p, r);
        check(near(p, 10.f) && near(r, -5.f), "fresh: apply is a no-op");
    }

    // Healthy suction keeps the indication exact indefinitely.
    {
        AttitudeGyro g;
        run(g, AttitudeGyro::kNominalSuctionInHg, 120.f);
        float p = -3.f, r = 25.f;
        g.apply(p, r);
        check(near(p, -3.f) && near(r, 25.f), "healthy: apply is a no-op after 2 min");
    }

    // Rotor physics: first-order lag with the documented time constants.
    {
        AttitudeGyro g;
        run(g, 0.f, 75.f);
        check(near(g.rotorSpeed(), std::exp(-1.f), 0.02f), "spin-down: r ~= e^-1 after one tau (75s)");
        run(g, 0.f, 75.f);
        check(near(g.rotorSpeed(), std::exp(-2.f), 0.02f), "spin-down: r ~= e^-2 after two tau");

        AttitudeGyro g2;
        run(g2, 0.f, 600.f); // fully dead
        run(g2, AttitudeGyro::kNominalSuctionInHg, 30.f);
        check(near(g2.rotorSpeed(), 1.f - std::exp(-1.f), 0.02f), "spin-up: r ~= 1-e^-1 after one tau (30s)");
    }

    // Phase 1 & 2: shortly after suction loss the indication has moved off
    // truth (wander), but is still bounded and nowhere near the rest pose.
    {
        AttitudeGyro g;
        run(g, 0.f, 45.f); // r ~= e^-0.6 ~= 0.55
        bool moved = false, bounded = true;
        float maxAbsP = 0.f, maxAbsR = 0.f;
        for (int i = 0; i < 500; ++i) {
            g.update(0.f, 0.02f);
            float p = 0.f, r = 0.f;
            g.apply(p, r);
            if (std::fabs(p) > 0.05f || std::fabs(r) > 0.05f) moved = true;
            maxAbsP = std::fmax(maxAbsP, std::fabs(p));
            maxAbsR = std::fmax(maxAbsR, std::fabs(r));
        }
        const float limit = AttitudeGyro::kWanderPitchDeg + AttitudeGyro::kWobbleDeg +
                            AttitudeGyro::kWanderRollDeg + 1.f;
        bounded = maxAbsP < limit && maxAbsR < limit;
        check(moved, "spin-down: indication wanders off truth");
        check(bounded, "spin-down: errors stay bounded, no rest pose yet");
    }

    // Phase 4: fully dead, the horizon hangs at the documented rest pose no
    // matter what the aircraft does.
    {
        AttitudeGyro g;
        run(g, 0.f, 600.f);
        check(g.rotorSpeed() < AttitudeGyro::kRestFullSpeed, "dead: rotor below rest threshold");
        for (float probe : {-20.f, 0.f, 15.f}) {
            float p = probe, r = probe;
            g.apply(p, r);
            check(near(p, AttitudeGyro::kRestPitchDeg, 0.5f), "dead: pitch pinned to rest pose");
            check(near(r, AttitudeGyro::kRestRollDeg, 0.5f), "dead: roll pinned to rest pose");
        }
    }

    // Recovery: suction back, the horizon re-erects onto truth.
    {
        AttitudeGyro g;
        run(g, 0.f, 600.f);
        run(g, AttitudeGyro::kNominalSuctionInHg, 180.f); // six spin-up tau
        float p = -7.f, r = 12.f;
        g.apply(p, r);
        check(near(p, -7.f, 0.5f) && near(r, 12.f, 0.5f), "spin-up: indication re-erected onto truth");
    }

    // Idle taxi: measured ~3.41 inHg sits below nominal, so the gyro droops
    // slowly - degraded but far from tumbling after a minute.
    {
        AttitudeGyro g;
        run(g, 3.41f, 60.f);
        // r settles toward 3.41/4.9 ~= 0.696 with tau 75s from 1.0:
        // r = 0.696 + 0.304*e^(-0.8) ~= 0.833
        check(near(g.rotorSpeed(), 0.833f, 0.02f), "idle: gyro droops to ~0.83 after 60s");
        float p = 0.f, r = 0.f;
        g.apply(p, r);
        check(std::fabs(p) < 10.f && std::fabs(r) < 15.f, "idle: degraded but not tumbled");
    }

    // Robustness: junk input changes nothing, huge dt is clamped to 0.25s.
    {
        AttitudeGyro g;
        g.update(std::numeric_limits<float>::quiet_NaN(), 0.02f);
        g.update(4.9f, std::numeric_limits<float>::infinity());
        check(near(g.rotorSpeed(), 1.f), "non-finite input ignored");
        g.update(-3.f, 0.02f); // negative suction clamps to target 0
        check(g.rotorSpeed() < 1.f, "negative suction treated as no suction");

        AttitudeGyro g2;
        g2.update(0.f, 30.f); // one hitchy frame must not spin the rotor down
        check(near(g2.rotorSpeed(), std::exp(-0.25f / 75.f), 0.01f), "dt clamped to 0.25s");
    }

    std::printf("%s (%d checks, %d failures)\n",
                g_failures ? "FAIL" : "OK", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
