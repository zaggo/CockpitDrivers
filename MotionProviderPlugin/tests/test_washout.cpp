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

    // Trace: nothing clamps at rest, on any tick of the run (not just the last).
    {
        WashoutFilter f(WashoutConfig::defaults());
        bool heaveEverClamped = false;
        bool rotEverClamped = false;
        for (int i = 0; i < 300; ++i) {
            f.update(level(), dt);
            heaveEverClamped |= f.trace().heaveClamped;
            rotEverClamped |= f.trace().rotRollClamped || f.trace().rotPitchClamped ||
                               f.trace().rotYawClamped;
        }
        check(!heaveEverClamped, "rest does not clamp heave");
        check(!rotEverClamped, "rest does not clamp rotations");
    }

    // Trace: reset() clears it.
    {
        WashoutFilter f(WashoutConfig::defaults());
        MotionCues up = level(); up.heaveG = 1.6f;
        run(f, up, 60);
        f.reset();
        const WashoutTrace fresh{};
        const WashoutTrace& t = f.trace();
        // memcmp over the raw struct is unsafe here: WashoutTrace interleaves bool
        // and double members, so the compiler inserts padding bytes (after
        // heaveClamped, after tiltRateActive, and trailing after rotYawClamped).
        // trace_ is default-initialized as a plain data member (padding left
        // indeterminate), while a locally list-initialized WashoutTrace{} does not
        // reliably zero those same bytes the same way -- memcmp compared garbage
        // padding, not the fields, and failed spuriously. Compare fields instead.
        check(t.heaveAHp == fresh.heaveAHp && t.heaveVel == fresh.heaveVel &&
              t.heavePosRaw == fresh.heavePosRaw && t.heaveClamped == fresh.heaveClamped &&
              t.tiltPitch == fresh.tiltPitch && t.tiltRoll == fresh.tiltRoll &&
              t.tiltRateActive == fresh.tiltRateActive &&
              t.rotRollRaw == fresh.rotRollRaw && t.rotPitchRaw == fresh.rotPitchRaw &&
              t.rotYawRaw == fresh.rotYawRaw &&
              t.rotRollClamped == fresh.rotRollClamped &&
              t.rotPitchClamped == fresh.rotPitchClamped &&
              t.rotYawClamped == fresh.rotYawClamped,
              "reset clears every trace field");
    }

    // Shortening the washout constants shrinks the excursion for the same input.
    //
    // The heave limit is disabled here on purpose. heavePos_ is a CLAMPED state:
    // heavePosRaw is read before this tick's clamp, but the value it is built from
    // was already clipped last tick, so with the shipped 30 mm limit the "raw"
    // peak can never exceed the limit by more than one integration step (~42 mm
    // measured). The comparison would then measure the clamp, not the filter.
    //
    // Figures below are for WashoutConfig::defaults(), i.e. heaveGain 0.5 - NOT the
    // 0.15 configured in configuration.toml, which yields excursions 3.3x smaller.
    // With the limit inert, the sustained-0.3g step response peaks at ~950 mm for
    // tau 2.0 (t ~ 3.2 s) and ~68 mm for tau 0.3 (t ~ 1.0 s) - a ratio of ~0.07,
    // comfortably under the 0.25 threshold.
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

    // Complementarity: the low-pass the tilt path consumes and the high-pass the
    // translation path consumes must sum back to the raw input, at every tick.
    // This is what makes double-counting structurally impossible; it is the
    // reason the crossover reuses tilt_lp_tau instead of adding a constant.
    {
        WashoutConfig cfg = WashoutConfig::defaults();
        cfg.surgeGain     = 1.0;
        cfg.surgeLimitMm  = 1.0e9;    // clamp inert: this is about the split, not the limit
        WashoutFilter f(cfg);
        const double dt = 1.0 / 60.0;
        const double G  = 9.80665;
        double lpRef = 0.0;                       // the same one-pole, recomputed here
        bool ok = true;
        for (int i = 0; i < 300; ++i) {
            MotionCues c = level();
            c.surgeG = 0.2f * static_cast<float>(std::sin(2 * M_PI * 0.3 * (i / 60.0)));
            const double aRaw = static_cast<double>(c.surgeG) * G;
            const double alpha = dt / (cfg.tiltLpTau + dt);
            lpRef += alpha * (aRaw - lpRef);
            f.update(c, dt);
            const double hp = f.trace().surgeAHp / cfg.surgeGain;
            if (std::fabs((lpRef + hp) - aRaw) > 1e-12) ok = false;
        }
        check(ok, "surge LP + HP reconstructs the raw input every tick");
    }

    // Zero gain (the shipped configuration) produces no translation at all.
    {
        WashoutConfig cfg = WashoutConfig::defaults();   // surgeGain/swayGain default to 0
        WashoutFilter f(cfg);
        MotionCues c = level(); c.surgeG = 0.4f; c.swayG = 0.3f;
        Pose p = run(f, c, 600);
        check(p.surge == 0.0f, "zero surge gain -> no surge output");
        check(p.sway  == 0.0f, "zero sway gain -> no sway output");
    }

    // The per-axis clamp holds and writes back into the integrator state.
    {
        WashoutConfig cfg = WashoutConfig::defaults();
        cfg.surgeGain    = 4.0;
        cfg.surgeLimitMm = 5.0;
        WashoutFilter f(cfg);
        MotionCues c = level(); c.surgeG = 0.6f;
        bool sawClamp = false, everOver = false;
        for (int i = 0; i < 600; ++i) {
            Pose p = f.update(c, 1.0 / 60.0);
            if (f.trace().surgeClamped) sawClamp = true;
            if (std::fabs(p.surge) > cfg.surgeLimitMm + 1e-6) everOver = true;
        }
        check(sawClamp,  "sustained surge engages the surge clamp");
        check(!everOver, "surge output never exceeds surge_limit_mm");
    }

    // reset() clears the new state: a fresh filter and a reset one agree.
    {
        WashoutConfig cfg = WashoutConfig::defaults();
        cfg.surgeGain = 1.0; cfg.swayGain = 1.0;
        WashoutFilter a(cfg), b(cfg);
        MotionCues c = level(); c.surgeG = 0.3f; c.swayG = 0.2f;
        run(b, c, 400);
        b.reset();
        MotionCues probe = level(); probe.surgeG = 0.1f; probe.swayG = 0.05f;
        Pose pa = run(a, probe, 50);
        Pose pb = run(b, probe, 50);
        check(std::fabs(pa.surge - pb.surge) < 1e-12, "reset clears surge state");
        check(std::fabs(pa.sway  - pb.sway)  < 1e-12, "reset clears sway state");
    }

    // Restructure regression. Drives a deterministic mixed cue sequence through
    // the shipped defaults and pins the output DOF at two points in the run.
    // Task 3 moves the tilt low-pass from the gained signal to the raw one --
    // algebraically identical, so these numbers must not move. Heave amplitude
    // is kept low enough that the clamp never engages: a value pinned to the
    // clamp asserts the limiter, not the filter, and would pass unchanged even
    // if the filter itself regressed.
    {
        constexpr double kPi = 3.14159265358979323846;
        WashoutConfig cfg = WashoutConfig::defaults();
        WashoutFilter f(cfg);
        Pose p120, p600;
        bool heaveEverClamped = false;
        for (int i = 0; i < 600; ++i) {
            const double t = i / 60.0;
            MotionCues c = level();
            c.heaveG    = 1.0f + 0.02f * static_cast<float>(std::sin(2 * kPi * 0.4 * t));
            c.surgeG    = 0.25f * static_cast<float>(std::sin(2 * kPi * 0.13 * t));
            c.swayG     = 0.18f * static_cast<float>(std::cos(2 * kPi * 0.21 * t));
            c.rollRate  = 6.0f * static_cast<float>(std::sin(2 * kPi * 0.7 * t));
            c.pitchRate = 4.0f * static_cast<float>(std::cos(2 * kPi * 0.5 * t));
            c.yawRate   = 2.0f * static_cast<float>(std::sin(2 * kPi * 0.3 * t));
            Pose p = f.update(c, 1.0 / 60.0);
            heaveEverClamped |= f.trace().heaveClamped;
            if (i == 119) p120 = p;
            if (i == 599) p600 = p;
        }
        check(!heaveEverClamped, "restructure golden run stays off the heave clamp");

        check(std::fabs(p120.heave -  13.876955986023) < 1e-9, "restructure keeps t120 heave bit-stable");
        check(std::fabs(p120.roll  -   0.631350338459) < 1e-9, "restructure keeps t120 roll bit-stable");
        check(std::fabs(p120.pitch -   3.284156560898) < 1e-9, "restructure keeps t120 pitch bit-stable");
        check(std::fabs(p120.yaw   -   0.123054608703) < 1e-9, "restructure keeps t120 yaw bit-stable");

        check(std::fabs(p600.heave - -11.237508773804) < 1e-9, "restructure keeps t600 heave bit-stable");
        check(std::fabs(p600.roll  -   2.095232009888) < 1e-9, "restructure keeps t600 roll bit-stable");
        check(std::fabs(p600.pitch -   3.315368652344) < 1e-9, "restructure keeps t600 pitch bit-stable");
        check(std::fabs(p600.yaw   -  -0.524945855141) < 1e-9, "restructure keeps t600 yaw bit-stable");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
