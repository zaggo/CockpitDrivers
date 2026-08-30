#include "EffectsLayer.h"
#include "EffectsConfig.h"
#include "MotionCues.h"
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <utility>
#include <vector>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }

int main() {
    const double dt = 1.0/60.0;

    // Airborne, no events -> zero offset.
    {
        EffectsLayer e(EffectsConfig::defaults());
        MotionCues air; air.onGround = false; air.groundspeed = 60.0f;
        double maxAbs = 0.0;
        for (int i=0;i<120;i++){ Pose p = e.update(air, dt); maxAbs = std::max(maxAbs, std::fabs((double)p.heave)); }
        check(maxAbs < 1e-6, "airborne -> no effect");
    }

    // Touchdown edge -> bump fires then decays toward zero.
    {
        EffectsLayer e(EffectsConfig::defaults());
        MotionCues air; air.onGround = false;
        e.update(air, dt);
        MotionCues gnd; gnd.onGround = true; gnd.groundspeed = 0.0f;
        double early = 0.0;
        for (int i=0;i<20;i++){ Pose p = e.update(gnd, dt); early = std::max(early, std::fabs((double)p.heave)); }
        double late = 0.0;
        for (int i=0;i<120;i++){ Pose p = e.update(gnd, dt); late = std::max(late, std::fabs((double)p.heave)); }
        check(early > 1.0, "touchdown produces a bump");
        check(late < early, "touchdown bump decays");
    }

    // Rolling on ground -> rumble present; stationary -> none.
    {
        EffectsLayer e(EffectsConfig::defaults());
        MotionCues roll; roll.onGround = true; roll.groundspeed = 40.0f;
        // skip initial touchdown-edge transient by pre-grounding
        MotionCues gndStill; gndStill.onGround = true; gndStill.groundspeed = 0.0f;
        for (int i=0;i<200;i++) e.update(gndStill, dt);      // let any bump decay
        double rMax=0.0; for (int i=0;i<120;i++){ Pose p=e.update(roll,dt); rMax=std::max(rMax,std::fabs((double)p.heave)); }
        double sMax=0.0; for (int i=0;i<120;i++){ Pose p=e.update(gndStill,dt); sMax=std::max(sMax,std::fabs((double)p.heave)); }
        check(rMax > 0.5, "rolling -> rumble");
        check(sMax < 1e-6, "stationary -> no rumble");
    }

    // state()/setState() round-trip: a layer seeded from another's captured
    // state must produce IDENTICAL output from then on. This is the exact
    // mechanism washout_replay's --cues loader relies on to seed a replay
    // from a recording's eff_* columns (Change 2) -- if this drifts, seeding
    // silently reproduces the wrong thing instead of failing loudly.
    {
        EffectsLayer source(EffectsConfig::defaults());
        MotionCues roll; roll.onGround = true; roll.groundspeed = 45.0f;
        // Advance the source through a touchdown edge and partway through a
        // rumble so all four state members end up non-default before capture.
        MotionCues air; air.onGround = false;
        source.update(air, dt);
        for (int i = 0; i < 37; ++i) source.update(roll, dt);   // mid-rumble, non-trivial phase

        const EffectsLayer::State captured = source.state();
        check(captured.prevOnGround, "captured state has prevOnGround true (sanity)");
        check(captured.rumblePhase > 1e-6, "captured state has a non-zero rumble phase (sanity)");

        EffectsLayer seeded(EffectsConfig::defaults());
        seeded.setState(captured);

        bool identical = true;
        for (int i = 0; i < 60; ++i) {
            const Pose ps = source.update(roll, dt);
            const Pose pd = seeded.update(roll, dt);
            if (ps.heave != pd.heave || ps.pitch != pd.pitch) { identical = false; break; }
        }
        check(identical, "a layer seeded via setState(state()) tracks the source exactly");

        // A layer that ISN'T seeded (starts from a fresh/zero state) must
        // diverge from the same point -- otherwise the round-trip check
        // above would pass for a trivial reason (e.g. rumble output being
        // phase-insensitive by accident).
        EffectsLayer unseeded(EffectsConfig::defaults());
        bool diverged = false;
        for (int i = 0; i < 60; ++i) {
            const Pose ps = source.update(roll, dt);
            const Pose pu = unseeded.update(roll, dt);
            if (ps.heave != pu.heave) { diverged = true; break; }
        }
        check(diverged, "an unseeded layer does NOT track the source -- the seed matters");
    }

    // --- Runway slab joints -------------------------------------------------
    // The whole point of the effect is that it stays inside the platform's
    // acceleration budget, so that is what these check -- not "does it move".

    auto slabCfg = [](double spacing, double step, double budget) {
        EffectsConfig c = EffectsConfig::defaults();
        c.rumbleGain = 0.0; c.touchdownGain = 0.0;   // isolate the slab effect
        c.slabSpacingM = spacing; c.slabStepMm = step; c.slabAccelMmS2 = budget;
        return c;
    };

    // Off by default -- an existing config must not suddenly grow an effect.
    {
        check(EffectsConfig::defaults().slabSpacingM == 0.0, "slab joints off by default");
        EffectsLayer e(EffectsConfig::defaults());
        MotionCues roll; roll.onGround = true; roll.groundspeed = 20.0f;
        EffectsConfig noRumble = EffectsConfig::defaults();
        noRumble.rumbleGain = 0.0; noRumble.touchdownGain = 0.0;
        EffectsLayer q(noRumble);
        q.update(MotionCues{}, dt);   // consume the touchdown edge
        double maxAbs = 0.0;
        for (int i = 0; i < 600; ++i) maxAbs = std::max(maxAbs, std::fabs((double)q.update(roll, dt).heave));
        check(maxAbs < 1e-6, "spacing 0 -> no slab output at all");
    }

    // Sample the effect densely enough to differentiate it twice, and check the
    // peak acceleration it demands against the configured budget.
    auto peakAccel = [&](double spacing, double step, double budget, double gs, double seconds) {
        EffectsLayer e(slabCfg(spacing, step, budget));
        MotionCues roll; roll.onGround = true; roll.groundspeed = (float)gs;
        e.update(roll, dt);                    // consume the touchdown edge
        const double h = 1.0 / 240.0;          // finer than the flight loop
        std::vector<double> x;
        for (int i = 0; i < (int)(seconds / h); ++i) x.push_back((double)e.update(roll, h).heave);
        double worst = 0.0, peak = 0.0;
        for (size_t i = 2; i < x.size(); ++i) {
            worst = std::max(worst, std::fabs((x[i] - 2*x[i-1] + x[i-2]) / (h*h)));
            peak  = std::max(peak, std::fabs(x[i]));
        }
        return std::pair<double,double>(worst, peak);
    };

    {
        // Taxi: the full step is rendered, well inside the budget.
        auto r = peakAccel(10.0, 1.0, 300.0, 10.0, 6.0);
        check(r.first  <= 300.0 * 1.05, "taxi: peak acceleration stays within the budget");
        check(r.second >= 0.9 && r.second <= 1.1, "taxi: the full 1 mm step is rendered");
    }
    {
        // Takeoff roll: joints arrive far faster; the step must shrink rather
        // than the budget be exceeded. This is the case the old rumble failed.
        auto r = peakAccel(5.0, 1.0, 300.0, 60.0, 6.0);
        check(r.first  <= 300.0 * 1.05, "fast roll: peak acceleration STILL within the budget");
        check(r.second <  1.0,          "fast roll: the step shrank to fit");
        check(r.second >  0.0,          "fast roll: something is still rendered");
    }
    {
        // Joints must not be dropped while accelerating. The gap to the next
        // joint is estimated from the current speed, so on a takeoff roll the
        // next joint arrives sooner than estimated; without a margin the move
        // is still running and the joint is skipped, which stutters the rhythm.
        EffectsLayer e(slabCfg(10.0, 2.0, 200.0));
        MotionCues roll; roll.onGround = true; roll.groundspeed = 5.0f;
        e.update(roll, dt);
        double dist = 0.0, prevTo = 0.0;
        int joints = 0;
        for (int i = 0; i < 3000; ++i) {
            roll.groundspeed = (float)std::min(60.0, 5.0 + 0.02 * i);   // ~1 m/s^2
            dist += (double)roll.groundspeed * dt;
            e.update(roll, dt);
            const double to = e.state().slabTo;
            if (to != prevTo) { ++joints; prevTo = to; }
        }
        const double expected = dist / 10.0;
        check(joints >= 0.95 * expected,
              "accelerating roll fires essentially every joint, none skipped");
    }
    {
        // Alternation: the offset must visit both signs, or the platform would
        // walk away from level one joint at a time.
        EffectsLayer e(slabCfg(10.0, 1.0, 300.0));
        MotionCues roll; roll.onGround = true; roll.groundspeed = 10.0f;
        e.update(roll, dt);
        double lo = 0.0, hi = 0.0;
        for (int i = 0; i < 600; ++i) {
            const double v = (double)e.update(roll, dt).heave;
            lo = std::min(lo, v); hi = std::max(hi, v);
        }
        check(hi > 0.5 && lo < -0.5, "joints alternate up and down");
    }
    {
        // Leaving the ground must return to level, not strand an offset.
        EffectsLayer e(slabCfg(10.0, 1.0, 300.0));
        MotionCues roll; roll.onGround = true; roll.groundspeed = 10.0f;
        e.update(roll, dt);
        for (int i = 0; i < 200; ++i) e.update(roll, dt);
        MotionCues air; air.onGround = false; air.groundspeed = 60.0f;
        double last = 0.0;
        for (int i = 0; i < 200; ++i) last = (double)e.update(air, dt).heave;
        check(std::fabs(last) < 1e-6, "airborne again -> slab offset returns to level");
    }
    {
        // State round-trip, same contract as the rumble phase above: the
        // odometer and the held offset do not decay, so a replay that does not
        // seed them cannot be bit-exact.
        EffectsLayer source(slabCfg(10.0, 1.0, 300.0));
        MotionCues roll; roll.onGround = true; roll.groundspeed = 10.0f;
        source.update(roll, dt);
        for (int i = 0; i < 137; ++i) source.update(roll, dt);
        EffectsLayer seeded(slabCfg(10.0, 1.0, 300.0));
        seeded.setState(source.state());
        bool identical = true;
        for (int i = 0; i < 300; ++i) {
            if (source.update(roll, dt).heave != seeded.update(roll, dt).heave) { identical = false; break; }
        }
        check(identical, "slab state survives setState(state())");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
