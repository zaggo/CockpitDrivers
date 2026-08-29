#include "WashoutFilter.h"
#include "WashoutConfig.h"
#include "MotionCues.h"
#include <cstdio>
#include <cmath>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }

static MotionCues level() { MotionCues c; c.heaveG = 1.0f; return c; } // at-rest

static Pose run(WashoutFilter& f, MotionCues c, int ticks, double dt=1.0/60.0) {
    Pose p;
    for (int i=0;i<ticks;i++) p = f.update(c, dt);
    return p;
}

int main() {
    const double dt = 1.0/60.0;

    // At rest -> pose stays ~home.
    {
        WashoutFilter f(WashoutConfig::defaults());
        Pose p = run(f, level(), 600);
        check(std::fabs(p.heave) < 0.5, "rest heave ~0");
        check(std::fabs(p.pitch) < 0.1, "rest pitch ~0");
        check(std::fabs(p.roll)  < 0.1, "rest roll ~0");
    }

    // Sustained heave accel washes out: big early, ~0 after long time.
    {
        WashoutFilter f(WashoutConfig::defaults());
        MotionCues up = level(); up.heaveG = 1.6f;   // sustained +0.6g
        Pose early = run(f, up, 10);
        Pose late  = run(f, up, 1200);
        check(std::fabs(early.heave) > std::fabs(late.heave), "heave washes out over time");
        check(std::fabs(late.heave) < 3.0, "sustained heave settles near 0");
    }

    // Sustained surge -> tilt-coordination reaches a steady non-zero pitch (held).
    {
        WashoutFilter f(WashoutConfig::defaults());
        MotionCues fwd = level(); fwd.surgeG = 0.2f;
        Pose late = run(f, fwd, 2000);
        check(std::fabs(late.pitch) > 0.5, "sustained surge -> steady tilt pitch");
        check(std::fabs(late.pitch) <= WashoutConfig::defaults().tiltLimitDeg + 1e-6,
              "tilt within limit");
    }

    // Sustained roll rate washes out (rotational high-pass + leak).
    {
        WashoutFilter f(WashoutConfig::defaults());
        MotionCues rr = level(); rr.rollRate = 10.0f; // deg/s sustained
        Pose early = run(f, rr, 15);
        Pose late  = run(f, rr, 1500);
        check(std::fabs(early.roll) > std::fabs(late.roll), "roll-rate cue washes out");
        check(std::fabs(late.roll) < 1.0, "sustained roll rate settles near 0");
    }

    // Limits: absurd input stays clamped and finite.
    {
        WashoutFilter f(WashoutConfig::defaults());
        MotionCues big = level(); big.heaveG = 20.0f; big.surgeG = 5.0f; big.rollRate = 500.0f;
        Pose p = run(f, big, 500);
        const WashoutConfig d = WashoutConfig::defaults();
        check(std::fabs(p.heave) <= d.heaveLimitMm + 1e-6, "heave clamped");
        check(std::fabs(p.pitch) <= d.tiltLimitDeg + d.rotLimitDeg + 1e-6, "pitch bounded");
        check(std::isfinite(p.heave) && std::isfinite(p.pitch) && std::isfinite(p.roll), "finite");
    }

    // reset() returns to zero-state (same output as a fresh filter at rest).
    {
        WashoutFilter f(WashoutConfig::defaults());
        MotionCues up = level(); up.heaveG = 1.6f;
        run(f, up, 100);
        f.reset();
        Pose p = f.update(level(), dt);
        check(std::fabs(p.heave) < 0.5 && std::fabs(p.pitch) < 0.1, "reset clears state");
    }

    // Trace: an ordinary manoeuvre saturates heave at the shipped settings.
    // 0.3 g through |G|max ~ 0.91 s^2 predicts a ~400 mm excursion against a
    // 30 mm limit -- this is the campaign's core claim, kept as a guard.
    {
        WashoutFilter f(WashoutConfig::defaults());
        MotionCues up = level(); up.heaveG = 1.3f;
        run(f, up, 30);
        check(f.trace().heaveClamped, "0.3g saturates heave at default settings");
        check(std::fabs(f.trace().heavePosRaw) > WashoutConfig::defaults().heaveLimitMm,
              "pre-clamp heave exceeds the limit");
    }

    // Trace: nothing clamps at rest.
    {
        WashoutFilter f(WashoutConfig::defaults());
        run(f, level(), 300);
        check(!f.trace().heaveClamped, "rest does not clamp heave");
        check(!f.trace().rotRollClamped && !f.trace().rotPitchClamped &&
              !f.trace().rotYawClamped, "rest does not clamp rotations");
    }

    // Trace: reset() clears it.
    {
        WashoutFilter f(WashoutConfig::defaults());
        MotionCues up = level(); up.heaveG = 1.6f;
        run(f, up, 60);
        f.reset();
        check(f.trace().heavePosRaw == 0.0 && !f.trace().heaveClamped,
              "reset clears the trace");
    }

    // Shortening the washout constants shrinks the excursion for the same input.
    //
    // The heave limit is disabled here on purpose. heavePos_ is a CLAMPED state:
    // heavePosRaw is read before this tick's clamp, but the value it is built from
    // was already clipped last tick, so with the shipped 30 mm limit the "raw"
    // peak can never exceed the limit by more than one integration step (~42 mm
    // measured). The comparison would then measure the clamp, not the filter.
    // With the limit inert, the sustained-0.3g step response peaks at ~286 mm for
    // tau 2.0 (t ~ 3.2 s) and ~46 mm for tau 0.3 (t ~ 1.0 s) - a ratio of ~0.16,
    // so the 0.25 threshold holds with margin.
    {
        auto peakRaw = [](double tau) {
            WashoutConfig cfg = WashoutConfig::defaults();
            cfg.heaveVelWashoutTau = tau;
            cfg.heavePosWashoutTau = tau;
            cfg.heaveLimitMm       = 1.0e9;   // clamp inert: measure the filter, not the limit
            WashoutFilter f(cfg);
            MotionCues up = level(); up.heaveG = 1.3f;
            double peak = 0.0;
            for (int i = 0; i < 900; ++i) {
                f.update(up, 1.0 / 60.0);
                const double v = std::fabs(f.trace().heavePosRaw);
                if (v > peak) peak = v;
            }
            return peak;
        };
        check(peakRaw(0.3) < peakRaw(2.0) * 0.25,
              "tau 2.0 -> 0.3 cuts the raw heave excursion by more than 4x (clamp disabled)");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
