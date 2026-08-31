#include "WashoutFilter.h"
#include "WashoutConfig.h"
#include "MotionCues.h"
#include <cstdio>
#include <cmath>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }

// M_PI is not standard C++ and MSVC won't define it without _USE_MATH_DEFINES;
// use this everywhere in the file instead.
constexpr double kPi = 3.14159265358979323846;

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
              t.surgeAHp == fresh.surgeAHp && t.surgeVel == fresh.surgeVel &&
              t.surgePosRaw == fresh.surgePosRaw && t.surgeClamped == fresh.surgeClamped &&
              t.swayAHp == fresh.swayAHp && t.swayVel == fresh.swayVel &&
              t.swayPosRaw == fresh.swayPosRaw && t.swayClamped == fresh.swayClamped &&
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
    //
    // The two gains per axis (surgeGain/tiltSurgeGain, swayGain/tiltSwayGain)
    // are deliberately distinct and non-unity. At gain 1.0 every gain factor
    // cancels in the check below, so a bug that applies the translation gain
    // twice (`gain*(aRaw - gain*aLp)`), or one that never moved the low-pass
    // off the *gained* signal in the first place (pre-restructure shape:
    // LP(tiltGain*aRaw) instead of LP(aRaw)), would pass unchanged. Distinct
    // gains make both of those show up as a nonzero reconstruction error.
    {
        WashoutConfig cfg = WashoutConfig::defaults();
        cfg.surgeGain     = 2.5;
        cfg.tiltSurgeGain = 1.7;
        cfg.surgeLimitMm  = 1.0e9;    // clamp inert: this is about the split, not the limit
        cfg.swayGain      = 1.4;
        cfg.tiltSwayGain  = 0.6;
        cfg.swayLimitMm   = 1.0e9;
        WashoutFilter f(cfg);
        const double dt = 1.0 / 60.0;
        const double G  = 9.80665;
        double lpRefSurge = 0.0, lpRefSway = 0.0;   // the same one-pole, recomputed here
        bool okSurge = true, okSway = true;
        for (int i = 0; i < 300; ++i) {
            MotionCues c = level();
            c.surgeG = 0.2f  * static_cast<float>(std::sin(2 * kPi * 0.3  * (i / 60.0)));
            c.swayG  = 0.15f * static_cast<float>(std::cos(2 * kPi * 0.37 * (i / 60.0)));
            const double aRawSurge = static_cast<double>(c.surgeG) * G;
            const double aRawSway  = static_cast<double>(c.swayG)  * G;
            const double alpha = dt / (cfg.tiltLpTau + dt);
            lpRefSurge += alpha * (aRawSurge - lpRefSurge);
            lpRefSway  += alpha * (aRawSway  - lpRefSway);
            f.update(c, dt);
            const double hpSurge = f.trace().surgeAHp / cfg.surgeGain;
            const double hpSway  = f.trace().swayAHp  / cfg.swayGain;
            if (std::fabs((lpRefSurge + hpSurge) - aRawSurge) > 1e-12) okSurge = false;
            if (std::fabs((lpRefSway  + hpSway)  - aRawSway)  > 1e-12) okSway  = false;
        }
        check(okSurge, "surge LP + HP reconstructs the raw input every tick (distinct gains)");
        check(okSway,  "sway LP + HP reconstructs the raw input every tick (distinct gains)");
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

    // Same, but with output smoothing enabled. configuration.toml ships
    // smooth_tau = 0.01, so the live rig always runs the smoothing branch
    // whose array indices this task shifted from 0..3 to 2..5 -- exercise
    // that branch rather than only the smoothTau == 0 bypass above.
    {
        WashoutConfig cfg = WashoutConfig::defaults();   // surgeGain/swayGain default to 0
        cfg.smoothTau = 0.01;
        WashoutFilter f(cfg);
        MotionCues c = level(); c.surgeG = 0.4f; c.swayG = 0.3f;
        Pose p = run(f, c, 600);
        check(p.surge == 0.0f, "zero surge gain -> no surge output (smoothing on)");
        check(p.sway  == 0.0f, "zero sway gain -> no sway output (smoothing on)");
    }

    // The per-axis clamp holds and writes back into the integrator state:
    // trace().surgePosRaw (read BEFORE this tick's clamp) must stay near
    // surgeLimitMm under sustained saturation, not run away underneath a
    // clamp that only ever touched the output.
    {
        WashoutConfig cfg = WashoutConfig::defaults();
        cfg.surgeGain    = 4.0;
        cfg.surgeLimitMm = 5.0;
        WashoutFilter f(cfg);
        MotionCues c = level(); c.surgeG = 0.6f;
        const double dt = 1.0 / 60.0;
        const double G  = 9.80665;

        // Bound derived from the recurrence itself, not chosen to pass:
        // `pos = pos*leak(posTau) + vel*dt*1000` runs every tick off of
        // last tick's post-clamp `pos` (the clamp writes back
        // unconditionally: `pos = limited;`), so that term alone can never
        // exceed surgeLimitMm (leak is in (0,1]). The only way this tick's
        // pre-clamp value can exceed the limit is through `vel*dt*1000`.
        // For this sustained, same-sign, constant input, the tilt low-pass
        // rises monotonically from 0 toward aRaw, so
        // aHp = surgeGain*(aRaw - aLp) is bounded in [0, surgeGain*aRaw] at
        // every tick. Feeding that ceiling into the (also never clamped)
        // velocity leaky-integrator forever gives its steady-state ceiling,
        // which no finite run starting from vel=0 can exceed:
        //   velBound = aHpMax * dt / (1 - leak(velTau))
        // and one tick of that velocity can add at most velBound*dt*1000 mm
        // to the pre-clamp position.
        const double aRaw     = static_cast<double>(c.surgeG) * G;
        const double aHpMax   = cfg.surgeGain * aRaw;
        const double leakVel  = std::exp(-dt / cfg.transVelWashoutTau);
        const double velBound = aHpMax * dt / (1.0 - leakVel);
        const double rawBound = cfg.surgeLimitMm + velBound * dt * 1000.0;

        bool sawClamp = false, everOver = false, rawEverFar = false;
        for (int i = 0; i < 600; ++i) {
            Pose p = f.update(c, dt);
            if (f.trace().surgeClamped) sawClamp = true;
            if (std::fabs(p.surge) > cfg.surgeLimitMm + 1e-6) everOver = true;
            if (std::fabs(f.trace().surgePosRaw) > rawBound) rawEverFar = true;
        }
        check(sawClamp,  "sustained surge engages the surge clamp");
        check(!everOver, "surge output never exceeds surge_limit_mm");
        check(!rawEverFar,
              "clamp writes back into the integrator: pre-clamp position never runs away");
    }

    // Axis identity: nothing else in the suite would catch a surge<->sway
    // swap. The golden case pins the other four axes with surge/sway both
    // zero; the clamp case above drives surge alone but never checks sway;
    // reset() compares two instances that would swap identically. Drive each
    // axis alone with both gains non-zero and check the cue lands on the
    // axis it was raised on.
    {
        WashoutConfig cfg = WashoutConfig::defaults();
        cfg.surgeGain = 2.0; cfg.swayGain = 2.0;
        {
            WashoutFilter f(cfg);
            MotionCues c = level(); c.surgeG = 0.3f; c.swayG = 0.0f;
            Pose p = run(f, c, 300);
            check(p.sway   == 0.0f, "pure surge cue -> no sway output");
            check(p.surge  != 0.0f, "pure surge cue -> nonzero surge output");
        }
        {
            WashoutFilter f(cfg);
            MotionCues c = level(); c.surgeG = 0.0f; c.swayG = 0.3f;
            Pose p = run(f, c, 300);
            check(p.surge  == 0.0f, "pure sway cue -> no surge output");
            check(p.sway   != 0.0f, "pure sway cue -> nonzero sway output");
        }
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

    // Same golden run as above, but with the SHIPPED tilt gains (configuration.toml
    // ships tilt_surge_gain = tilt_sway_gain = 0.4, not the WashoutConfig::defaults()
    // 1.0 the block above uses). At gain 1.0 the tilt gain factor is invisible: it
    // multiplies by exactly 1, so deleting `cfg_.tiltSurgeGain` from the tgtPitch
    // computation in WashoutFilter.cpp leaves every literal above passing unchanged.
    // This block exists so a regression in how the tilt gain is applied has
    // somewhere to show up.
    //
    // tiltLimitDeg is held inert here (the shipped/default 3 deg limit is left
    // alone in the block above). At that limit, gain 1.0 and gain 0.4 both drive
    // tgtPitch past the clamp and saturate at the *same* +/-3 deg ceiling at t120
    // and t600, so the two configs land on identical pitch values regardless of
    // the gain -- blind to the gain in the same way as the block above, just via
    // clamp saturation instead of gain=1 identity. Freeing the clamp here is what
    // makes this block's pitch checks actually depend on tiltSurgeGain.
    {
        WashoutConfig cfg = WashoutConfig::defaults();
        cfg.tiltSurgeGain = 0.4;
        cfg.tiltSwayGain  = 0.4;
        cfg.tiltLimitDeg  = 1.0e9;   // clamp inert: see comment above
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
        check(!heaveEverClamped, "restructure golden run (tilt gain 0.4) stays off the heave clamp");

        check(std::fabs(p120.heave -  13.876955986023) < 1e-9, "restructure keeps t120 heave bit-stable (tilt gain 0.4)");
        check(std::fabs(p120.roll  -   0.673518538475) < 1e-9, "restructure keeps t120 roll bit-stable (tilt gain 0.4)");
        check(std::fabs(p120.pitch -   3.463330030441) < 1e-9, "restructure keeps t120 pitch bit-stable (tilt gain 0.4)");
        check(std::fabs(p120.yaw   -   0.123054608703) < 1e-9, "restructure keeps t120 yaw bit-stable (tilt gain 0.4)");

        check(std::fabs(p600.heave - -11.237508773804) < 1e-9, "restructure keeps t600 heave bit-stable (tilt gain 0.4)");
        check(std::fabs(p600.roll  -   0.730485379696) < 1e-9, "restructure keeps t600 roll bit-stable (tilt gain 0.4)");
        check(std::fabs(p600.pitch -   3.337597608566) < 1e-9, "restructure keeps t600 pitch bit-stable (tilt gain 0.4)");
        check(std::fabs(p600.yaw   -  -0.524945855141) < 1e-9, "restructure keeps t600 yaw bit-stable (tilt gain 0.4)");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
