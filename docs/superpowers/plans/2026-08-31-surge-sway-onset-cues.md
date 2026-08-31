# Surge/Sway Onset Cues Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the motion platform a translational onset cue on surge and sway — the first fraction of a second of a takeoff push, a braking application or a crosswind gust — while tilt coordination keeps rendering the sustained part.

**Architecture:** The tilt channel already low-passes horizontal specific force. The onset channel takes exactly the complement of that low-pass (`aHp = aRaw − aLp`), leaky-double-integrates it to millimetres like the heave channel, and clamps it to a per-axis limit measured from the geometry. One crossover constant (`tilt_lp_tau`) serves both halves, so `LP + HP = 1` and no acceleration is counted twice. New gains ship at 0, so the flying configuration is untouched until a rig verdict adopts values.

**Tech Stack:** C++17, X-Plane 12 SDK plugin (CMake, not PlatformIO). Host test suites are plain `main()` executables with a hand-rolled `check()` helper, run under CTest. Offline harness: `tools/washout_replay` (C++) and `tools/washout_metrics.py` (numpy, in `tools/.venv`).

**Spec:** `docs/superpowers/specs/2026-08-31-surge-sway-onset-cues-design.md`

## Global Constraints

- Work inside `MotionProviderPlugin/`. Paths in this plan are relative to that directory unless they start with `docs/`.
- Build and run the full test suite for every task: `cmake -S tests -B tests/build && cmake --build tests/build && ctest --test-dir tests/build --output-on-failure`. **All eleven suites must be green at the end of every task** — `kinematics`, `config`, `washout`, `effects`, `bff`, `safety`, `monitor`, `heartbeat`, `armramp`, `armgate`, `telemetry`.
- Python is the venv only: `tools/.venv/bin/python`. Never `python3` — the system interpreter has no numpy.
- `configuration.toml` must stay **numerically identical** to the 2026-08-30 acceptance flight through Tasks 1–7. New keys are added with `surge_gain = 0` / `sway_gain = 0`; no existing value changes. Adopted values are written only in Task 10.
- New telemetry columns are **appended** to `Telemetry::header()`, never inserted. `docs/motion-tuning/README.md` §2 exports cues with a position-based `cut -d, -f2,4-16,58-61`; inserting breaks it silently.
- Crossover constant is `tilt_lp_tau`. Do **not** add a separate high-pass constant — that is the whole point of the chosen approach.
- Acceleration budget: ≈363 mm/s² (120 000 counts/s² at 330.7 counts/mm), shared with everything else the chain emits. At the 1.5 s crossover a 20 mm cue peaks at 8.9 mm/s², so the budget does not bind — but it does the moment an effective time constant drops below ≈0.24 s.
- Commit after every task. Conventional Commits, subject ≤ 50 chars.

---

### Task 1: Envelope probe tool

Measures how much surge and sway the geometry actually has, both bare and with heave and tilt already spending their share. Nothing else in this plan starts before its numbers exist: `clampToReachable` scales all six DOF by one factor, so an unreachable surge demand shrinks heave and tilt too.

**Files:**
- Create: `tools/envelope_probe.cpp`
- Modify: `tools/CMakeLists.txt`
- Modify: `docs/motion-tuning/baseline-metrics.md`

**Interfaces:**
- Consumes: `MotionConfig::loadGeometry(path)`, `StewartKinematics::solve(Pose)` → `SolveResult{ .allReachable }`.
- Produces: the executable `tools/build/envelope_probe`, and two numbers later tasks use as `surge_limit_mm` and `sway_limit_mm`.

- [ ] **Step 1: Write the probe**

Create `tools/envelope_probe.cpp`:

```cpp
// Measures the platform's reachable surge/sway travel, bare and in the corner
// of the envelope the other channels already occupy. Links the real
// StewartKinematics -- there is deliberately no second implementation of the
// geometry that could drift from the plugin's.
#include "MotionConfig.h"
#include "StewartKinematics.h"

#include <cstdio>
#include <cstring>
#include <string>

namespace {

bool reachable(const StewartKinematics& k, const Pose& p) {
    return k.solve(p).allReachable;
}

// Largest travel along `axis` in direction `dir` that still solves, searched in
// [0, hi] by bisection. Assumes `base` is reachable and that reachability is
// monotone along the axis -- true here: the legs run out of travel as the pose
// moves away from home, they do not come back.
double maxTravel(const StewartKinematics& k, const Pose& base,
                 float Pose::*axis, double dir, double hi) {
    Pose p = base;
    p.*axis = static_cast<float>(base.*axis + dir * hi);
    if (reachable(k, p)) return hi;      // never ran out inside the search range
    double lo = 0.0;
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        p = base;
        p.*axis = static_cast<float>(base.*axis + dir * mid);
        if (reachable(k, p)) lo = mid; else hi = mid;
    }
    return lo;
}

void report(const StewartKinematics& k, const char* label, const Pose& base) {
    if (!reachable(k, base)) {
        std::printf("%-28s BASE POSE UNREACHABLE\n", label);
        return;
    }
    const double sPos = maxTravel(k, base, &Pose::surge, +1.0, 500.0);
    const double sNeg = maxTravel(k, base, &Pose::surge, -1.0, 500.0);
    const double yPos = maxTravel(k, base, &Pose::sway,  +1.0, 500.0);
    const double yNeg = maxTravel(k, base, &Pose::sway,  -1.0, 500.0);
    std::printf("%-28s surge +%7.2f / -%7.2f    sway +%7.2f / -%7.2f  (mm)\n",
                label, sPos, sNeg, yPos, yNeg);
}

}  // namespace

int main(int argc, char** argv) {
    std::string cfgPath = "configuration.toml";
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) cfgPath = argv[++i];
    }

    bool loaded = false;
    const StewartGeometry geo = MotionConfig::loadGeometry(cfgPath, &loaded);
    std::printf("config: %s (%s)\n", cfgPath.c_str(), loaded ? "loaded" : "DEFAULTS -- file not read");
    StewartKinematics kin(geo);
    std::printf("home height: %.2f mm\n\n", kin.homeHeight());

    // Bare: an upper bound, never available in flight.
    report(kin, "home pose", Pose{});

    // In the corner: heave at its limit and tilt+rotational at their combined
    // per-axis limit, i.e. what the existing channels are already allowed to
    // occupy at the same time. This is the number the per-axis limits come from.
    for (double hs : {+1.0, -1.0}) {
        for (double as : {+1.0, -1.0}) {
            Pose c;
            c.heave = static_cast<float>(hs * 30.0);
            c.roll  = static_cast<float>(as * 14.0);
            c.pitch = static_cast<float>(as * 14.0);
            c.yaw   = static_cast<float>(as * 7.0);
            char label[64];
            std::snprintf(label, sizeof(label), "corner h%+.0f r/p%+.0f y%+.0f",
                          c.heave, c.roll, c.yaw);
            report(kin, label, c);
        }
    }
    return 0;
}
```

- [ ] **Step 2: Add it to the tools build**

In `tools/CMakeLists.txt`, after the `washout_replay` target:

```cmake
# Reachable-envelope measurement. Same rule as washout_replay: links the real
# StewartKinematics, no second copy of the geometry.
add_executable(envelope_probe
    envelope_probe.cpp
    ../src/StewartKinematics.cpp
    ../src/MotionConfig.cpp)
target_include_directories(envelope_probe PRIVATE ../src ../third_party/tomlplusplus)
```

- [ ] **Step 3: Build and run it**

```bash
cd MotionProviderPlugin
cmake -S tools -B tools/build && cmake --build tools/build
./tools/build/envelope_probe --config configuration.toml
```

Expected: `config: configuration.toml (loaded)`, a home height near the value `StewartKinematics::homeHeight()` reports for this geometry, one `home pose` row and four `corner` rows, all with finite travel numbers.

- [ ] **Step 4: Sanity-check the output before trusting it**

Three things must hold, or the probe is measuring itself rather than the platform:

1. No row says `BASE POSE UNREACHABLE`. If a corner is unreachable, the existing channels can already demand an unreachable pose — stop and report that, it is a finding about the shipped config, not about this feature.
2. Every corner row's travel is **smaller** than the `home pose` row's.
3. No number equals exactly `500.00` — that is the search ceiling, meaning the axis never ran out and the result is not a measurement. If it happens, raise the `500.0` ceiling and rerun.

- [ ] **Step 5: Record the numbers and choose the limits**

Append a section to `docs/motion-tuning/baseline-metrics.md`:

```markdown
## Reachable surge/sway envelope (2026-08-31, `tools/envelope_probe`)

Measured on the shipped `[geometry]`, with the real `StewartKinematics::solve`.

| Base pose | surge + | surge − | sway + | sway − |
|---|---|---|---|---|
| home | ... | ... | ... | ... |
| corner heave +30, roll/pitch +14, yaw +7 | ... | ... | ... | ... |
| corner heave +30, roll/pitch −14, yaw −7 | ... | ... | ... | ... |
| corner heave −30, roll/pitch +14, yaw +7 | ... | ... | ... | ... |
| corner heave −30, roll/pitch −14, yaw −7 | ... | ... | ... | ... |

The corner rows are what matters: `clampToReachable` scales all six DOF by one
bisection factor, so surge that does not fit shrinks heave and tilt with it.

**Chosen limits:** `surge_limit_mm` and `sway_limit_mm` = 70 % of the smallest
corner value on that axis, rounded down to a whole millimetre. The 30 % margin
covers the effects layer and the fact that the corner grid is coarse.
```

Fill the `...` cells with the measured numbers, then state the two chosen limits explicitly under the table (e.g. "surge_limit_mm = 12, sway_limit_mm = 9"). Those two numbers are inputs to Tasks 2 and 3.

- [ ] **Step 6: Commit**

```bash
git add MotionProviderPlugin/tools/envelope_probe.cpp MotionProviderPlugin/tools/CMakeLists.txt docs/motion-tuning/baseline-metrics.md
git commit -m "feat(motion): measure reachable surge/sway envelope"
```

---

### Task 2: Capture the pre-restructure golden sequence

The next task moves the tilt low-pass from the gained signal to the raw signal. That is algebraically identical (`LP(k·x) ≡ k·LP(x)`) but not bit-identical in floating point, and the tilt channel is signed-off flying behaviour. This task records what the filter does **today** so the next one can prove it still does it.

**Files:**
- Modify: `tests/test_washout.cpp`

**Interfaces:**
- Produces: a test case named "restructure regression" holding four hard-coded golden values, used unchanged by Task 3.

- [ ] **Step 1: Add a printing capture case**

Add to `tests/test_washout.cpp`, immediately before the final `std::printf("\n%d checks...` line:

```cpp
    // Restructure regression. Drives a deterministic mixed cue sequence through
    // the shipped defaults and pins the four output DOF. Task 3 moves the tilt
    // low-pass from the gained signal to the raw one -- algebraically identical,
    // so these numbers must not move.
    {
        WashoutConfig cfg = WashoutConfig::defaults();
        WashoutFilter f(cfg);
        Pose p;
        for (int i = 0; i < 600; ++i) {
            const double t = i / 60.0;
            MotionCues c = level();
            c.heaveG    = 1.0f + 0.3f * static_cast<float>(std::sin(2 * M_PI * 0.4 * t));
            c.surgeG    = 0.25f * static_cast<float>(std::sin(2 * M_PI * 0.13 * t));
            c.swayG     = 0.18f * static_cast<float>(std::cos(2 * M_PI * 0.21 * t));
            c.rollRate  = 6.0f * static_cast<float>(std::sin(2 * M_PI * 0.7 * t));
            c.pitchRate = 4.0f * static_cast<float>(std::cos(2 * M_PI * 0.5 * t));
            c.yawRate   = 2.0f * static_cast<float>(std::sin(2 * M_PI * 0.3 * t));
            p = f.update(c, 1.0 / 60.0);
        }
        std::printf("GOLDEN heave=%.12f roll=%.12f pitch=%.12f yaw=%.12f\n",
                    p.heave, p.roll, p.pitch, p.yaw);
    }
```

- [ ] **Step 2: Build and run it to read the numbers**

```bash
cd MotionProviderPlugin
cmake -S tests -B tests/build && cmake --build tests/build
./tests/build/test_washout
```

Expected: one `GOLDEN heave=... roll=... pitch=... yaw=...` line, and the suite still reporting 0 failures.

- [ ] **Step 3: Turn the printout into assertions**

Replace the `std::printf("GOLDEN ...` call with the four checks, pasting the measured numbers:

```cpp
        check(std::fabs(p.heave - GOLDEN_HEAVE) < 1e-9, "restructure keeps heave bit-stable");
        check(std::fabs(p.roll  - GOLDEN_ROLL)  < 1e-9, "restructure keeps roll bit-stable");
        check(std::fabs(p.pitch - GOLDEN_PITCH) < 1e-9, "restructure keeps pitch bit-stable");
        check(std::fabs(p.yaw   - GOLDEN_YAW)   < 1e-9, "restructure keeps yaw bit-stable");
```

Substitute the literal values for `GOLDEN_HEAVE` etc. — write them inline as `-1.234567890123` style literals, not named constants, so the diff shows exactly what was measured. Keep the 1e-9 absolute tolerance: the expected float difference from the restructure is ~1e-16 relative, so 1e-9 catches a real change while tolerating rounding.

- [ ] **Step 4: Run to verify it passes**

Run: `cmake --build tests/build && ./tests/build/test_washout`
Expected: PASS, 0 failures. (It must pass here — the restructure has not happened yet.)

- [ ] **Step 5: Commit**

```bash
git add MotionProviderPlugin/tests/test_washout.cpp
git commit -m "test(motion): pin washout output before restructure"
```

---

### Task 3: The onset channel in WashoutFilter

**Files:**
- Modify: `src/WashoutConfig.h`
- Modify: `src/WashoutFilter.h`
- Modify: `src/WashoutFilter.cpp`
- Test: `tests/test_washout.cpp`

**Interfaces:**
- Consumes: `surge_limit_mm` / `sway_limit_mm` chosen in Task 1.
- Produces:
  - `WashoutConfig` fields `double surgeGain`, `swayGain`, `transVelWashoutTau`, `transPosWashoutTau`, `surgeLimitMm`, `swayLimitMm`.
  - `WashoutTrace` fields `double surgeAHp, surgeVel, surgePosRaw; bool surgeClamped;` and the same four for sway.
  - `Pose::surge` / `Pose::sway` now carry a non-zero value whenever the gains are non-zero.

- [ ] **Step 1: Write the failing tests**

Add these four cases to `tests/test_washout.cpp`, before the golden-sequence case:

```cpp
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
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd MotionProviderPlugin && cmake --build tests/build`
Expected: compile error — `WashoutConfig` has no member `surgeGain`, `WashoutTrace` has no member `surgeAHp`. That is the failing state for this task.

- [ ] **Step 3: Add the config fields**

In `src/WashoutConfig.h`, after the tilt block:

```cpp
    // Translational onset cue: the complement of the tilt low-pass, leaky
    // double-integrated to mm. Renders the first fraction of a second of a
    // longitudinal/lateral acceleration, which tilt coordination cannot --
    // it is low-passed and rate-limited by design. The crossover constant is
    // tiltLpTau above, shared by both halves, so LP + HP = 1 and the same
    // acceleration is never counted twice.
    // Gains ship at 0: the channel is off until a rig verdict adopts values.
    double surgeGain           = 0.0;
    double swayGain            = 0.0;
    double transVelWashoutTau  = 0.25;   // shared by surge and sway
    double transPosWashoutTau  = 0.25;
    double surgeLimitMm        = 10.0;   // replace with the Task 1 measurement
    double swayLimitMm         = 10.0;   // replace with the Task 1 measurement
```

Set the two limits to the values chosen in Task 1 rather than leaving 10.0.

- [ ] **Step 4: Add the trace fields and the state**

In `src/WashoutFilter.h`, add to `WashoutTrace` after the heave fields:

```cpp
    double surgeAHp       = 0.0;   // high-passed longitudinal specific force, m/s^2 (post-gain)
    double surgeVel       = 0.0;
    double surgePosRaw    = 0.0;   // position BEFORE the +/-surgeLimitMm clamp, mm
    bool   surgeClamped   = false;
    double swayAHp        = 0.0;
    double swayVel        = 0.0;
    double swayPosRaw     = 0.0;
    bool   swayClamped    = false;
```

In the private section, replace the two tilt low-pass members and widen the smoothing arrays:

```cpp
    // Horizontal specific force, low-passed on the RAW signal. The tilt path
    // applies its gain outside the filter (LP is linear, so LP(k*x) == k*LP(x));
    // the onset path takes the complement. One low-pass, two consumers.
    double surgeLpRaw_ = 0.0;
    double swayLpRaw_  = 0.0;
    double tiltPitch_ = 0.0;
    double tiltRoll_ = 0.0;

    // Translational onset state
    double surgeVel_ = 0.0, surgePos_ = 0.0;
    double swayVel_  = 0.0, swayPos_  = 0.0;
```

and change the smoothing state to six wide:

```cpp
    // Output smoothing state (two cascaded one-pole LPs per DOF, ordered
    // surge, sway, heave, roll, pitch, yaw). Active when cfg_.smoothTau > 0.
    double sm1_[6] = {0, 0, 0, 0, 0, 0};
    double sm2_[6] = {0, 0, 0, 0, 0, 0};
```

- [ ] **Step 5: Implement the channel**

In `src/WashoutFilter.cpp`, `reset()`:

```cpp
    surgeLpRaw_ = swayLpRaw_ = tiltPitch_ = tiltRoll_ = 0.0;
    surgeVel_ = surgePos_ = swayVel_ = swayPos_ = 0.0;
```
and change the smoothing clear loop to `for (int i = 0; i < 6; ++i)`.

Replace the tilt-coordination block in `update()` with:

```cpp
    // --- Horizontal specific force: one low-pass, two consumers ---
    // Tilt coordination takes the low-passed (sustained) part; the onset
    // channel takes the complement. The gains sit outside the filter so both
    // can be scaled independently without the split stopping being a split.
    const double aRawX = static_cast<double>(c.surgeG) * G;
    const double aRawY = static_cast<double>(c.swayG)  * G;
    surgeLpRaw_ += lpAlpha(dt, cfg_.tiltLpTau) * (aRawX - surgeLpRaw_);
    swayLpRaw_  += lpAlpha(dt, cfg_.tiltLpTau) * (aRawY - swayLpRaw_);

    double tgtPitch = std::asin(clampd(cfg_.tiltSurgeGain * surgeLpRaw_ / G, -1.0, 1.0)) * kRad2Deg;
    double tgtRoll  = std::asin(clampd(cfg_.tiltSwayGain  * swayLpRaw_  / G, -1.0, 1.0)) * kRad2Deg;
    tgtPitch = clampd(tgtPitch, -cfg_.tiltLimitDeg, cfg_.tiltLimitDeg);
    tgtRoll  = clampd(tgtRoll,  -cfg_.tiltLimitDeg, cfg_.tiltLimitDeg);
    bool tiltLimited = false;
    tiltPitch_ = rateLimit(tiltPitch_, tgtPitch, cfg_.tiltRateLimitDps, dt, tiltLimited);
    tiltRoll_  = rateLimit(tiltRoll_,  tgtRoll,  cfg_.tiltRateLimitDps, dt, tiltLimited);
    trace_.tiltPitch      = tiltPitch_;
    trace_.tiltRoll       = tiltRoll_;
    trace_.tiltRateActive = tiltLimited;

    // --- Translational onset: HP(accel) -> leaky double-integrate -> mm ---
    // Same shape as the heave channel, including the clamp writing back into
    // the integrator state (windup with no anti-windup -- consistent with
    // heave, deliberately).
    auto transChan = [&](double gain, double aLp, double aRaw, double limitMm,
                         double& vel, double& pos,
                         double& aHpOut, double& velOut, double& rawOut, bool& clampedOut) {
        const double aHp = gain * (aRaw - aLp);
        vel = vel * leak(dt, cfg_.transVelWashoutTau) + aHp * dt;
        pos = pos * leak(dt, cfg_.transPosWashoutTau) + vel * dt * 1000.0;
        aHpOut = aHp;
        velOut = vel;
        rawOut = pos;
        const double limited = clampd(pos, -limitMm, limitMm);
        clampedOut = (limited != pos);
        pos = limited;
    };
    transChan(cfg_.surgeGain, surgeLpRaw_, aRawX, cfg_.surgeLimitMm,
              surgeVel_, surgePos_,
              trace_.surgeAHp, trace_.surgeVel, trace_.surgePosRaw, trace_.surgeClamped);
    transChan(cfg_.swayGain, swayLpRaw_, aRawY, cfg_.swayLimitMm,
              swayVel_, swayPos_,
              trace_.swayAHp, trace_.swayVel, trace_.swayPosRaw, trace_.swayClamped);
```

Then widen the smoothing and the output:

```cpp
    double out[6] = { surgePos_,
                      swayPos_,
                      heavePos_,
                      tiltRoll_  + rollAngle_,
                      tiltPitch_ + pitchAngle_,
                      yawAngle_ };
    if (cfg_.smoothTau > 0.0) {
        const double a = lpAlpha(dt, cfg_.smoothTau);
        for (int i = 0; i < 6; ++i) {
            sm1_[i] += a * (out[i] - sm1_[i]);
            sm2_[i] += a * (sm1_[i] - sm2_[i]);
            out[i] = sm2_[i];
        }
    } else {
        for (int i = 0; i < 6; ++i) { sm1_[i] = out[i]; sm2_[i] = out[i]; }
    }

    Pose p;
    // Sign convention: +X forward, so a forward specific force commands a
    // forward platform translation. Confirmed against the tilt axis by bench
    // jog in Task 8 -- do not flip it on reasoning alone.
    p.surge = static_cast<float>(out[0]);
    p.sway  = static_cast<float>(out[1]);
    p.heave = static_cast<float>(out[2]);
    p.roll  = static_cast<float>(out[3]);
    p.pitch = static_cast<float>(out[4]);
    p.yaw   = static_cast<float>(out[5]);
    return p;
```

- [ ] **Step 6: Run the suite**

Run: `cd MotionProviderPlugin && cmake --build tests/build && ctest --test-dir tests/build --output-on-failure`
Expected: all eleven suites PASS, including the Task 2 golden case.

If the golden case fails at the 1e-9 tolerance, the `k·LP(x)` move is not the cause — the difference it produces is ~1e-16 relative. Look for a real change: a mis-ordered `out[]` index, or the tilt clamp applied to the wrong quantity. Do not widen the tolerance to make it pass.

- [ ] **Step 7: Commit**

```bash
git add MotionProviderPlugin/src/WashoutConfig.h MotionProviderPlugin/src/WashoutFilter.h MotionProviderPlugin/src/WashoutFilter.cpp MotionProviderPlugin/tests/test_washout.cpp
git commit -m "feat(motion): add surge/sway onset channel"
```

---

### Task 4: Config parsing and writeback

Without writeback the new keys vanish the next time the file is seeded, and a rig operator has no visible knob.

**Files:**
- Modify: `src/MotionConfig.cpp:101-120` (parse), `src/MotionConfig.cpp:230-250` (writeback)
- Test: `tests/test_config.cpp`

**Interfaces:**
- Consumes: the `WashoutConfig` fields from Task 3.
- Produces: TOML keys `surge_gain`, `sway_gain`, `trans_vel_washout_tau`, `trans_pos_washout_tau`, `surge_limit_mm`, `sway_limit_mm` under `[washout]`.

- [ ] **Step 1: Write the failing test**

Add to `tests/test_config.cpp`, inside `main()`:

```cpp
    // The onset-channel keys parse, and absent keys keep their defaults.
    {
        std::string tmp = tmpPath();
        { std::ofstream f(tmp);
          f << "[washout]\n"
               "surge_gain = 0.6\n"
               "sway_gain = 0.45\n"
               "trans_vel_washout_tau = 0.4\n"
               "surge_limit_mm = 12\n"; }
        WashoutConfig w = MotionConfig::loadWashout(tmp);
        WashoutConfig d = WashoutConfig::defaults();
        near(w.surgeGain, 0.6, "surge_gain parsed");
        near(w.swayGain, 0.45, "sway_gain parsed");
        near(w.transVelWashoutTau, 0.4, "trans_vel_washout_tau parsed");
        near(w.surgeLimitMm, 12.0, "surge_limit_mm parsed (int literal)");
        near(w.transPosWashoutTau, d.transPosWashoutTau, "absent trans_pos tau keeps default");
        near(w.swayLimitMm, d.swayLimitMm, "absent sway_limit_mm keeps default");
    }

    // Onset gains are off in a freshly seeded file, and every key round-trips.
    {
        std::string tmp = tmpPath();
        check(MotionConfig::writeDefaults(tmp), "writeDefaults succeeds");
        WashoutConfig w = MotionConfig::loadWashout(tmp);
        near(w.surgeGain, 0.0, "seeded surge_gain is 0");
        near(w.swayGain, 0.0, "seeded sway_gain is 0");
        WashoutConfig d = WashoutConfig::defaults();
        near(w.transVelWashoutTau, d.transVelWashoutTau, "trans_vel tau round-trips");
        near(w.transPosWashoutTau, d.transPosWashoutTau, "trans_pos tau round-trips");
        near(w.surgeLimitMm, d.surgeLimitMm, "surge_limit_mm round-trips");
        near(w.swayLimitMm, d.swayLimitMm, "sway_limit_mm round-trips");
    }
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd MotionProviderPlugin && cmake --build tests/build && ./tests/build/test_config`
Expected: FAIL — `surge_gain parsed (0.000000 vs 0.600000)` and the round-trip checks failing, because nothing reads or writes those keys yet.

- [ ] **Step 3: Add the parse lines**

In `src/MotionConfig.cpp`, in the `washout` table block after `tilt_rate_limit_dps`:

```cpp
        getDouble(*t, "surge_gain", w.surgeGain);
        getDouble(*t, "sway_gain", w.swayGain);
        getDouble(*t, "trans_vel_washout_tau", w.transVelWashoutTau);
        getDouble(*t, "trans_pos_washout_tau", w.transPosWashoutTau);
        getDouble(*t, "surge_limit_mm", w.surgeLimitMm);
        getDouble(*t, "sway_limit_mm", w.swayLimitMm);
```

- [ ] **Step 4: Add the writeback lines**

In the `[washout]` writeback block, after `tilt_rate_limit_dps`:

```cpp
    f << "surge_gain = "             << w.surgeGain          << "\n";
    f << "sway_gain = "              << w.swayGain           << "\n";
    f << "trans_vel_washout_tau = "  << w.transVelWashoutTau << "\n";
    f << "trans_pos_washout_tau = "  << w.transPosWashoutTau << "\n";
    f << "surge_limit_mm = "         << w.surgeLimitMm       << "\n";
    f << "sway_limit_mm = "          << w.swayLimitMm        << "\n";
```

- [ ] **Step 5: Run to verify it passes**

Run: `cd MotionProviderPlugin && cmake --build tests/build && ctest --test-dir tests/build --output-on-failure`
Expected: all eleven suites PASS.

- [ ] **Step 6: Add the keys to the live configuration.toml, off**

Append to the `[washout]` section of `MotionProviderPlugin/configuration.toml`, changing nothing else in the file:

```toml
# Translational onset cue (2026-08-31). Complement of the tilt low-pass:
# tilt renders the sustained part, this renders the onset, crossover at
# tilt_lp_tau. OFF until a rig verdict adopts values -- see
# docs/superpowers/specs/2026-08-31-surge-sway-onset-cues-design.md.
surge_gain = 0
sway_gain = 0
trans_vel_washout_tau = 0.25
trans_pos_washout_tau = 0.25
surge_limit_mm = 12    # from tools/envelope_probe, see docs/motion-tuning/baseline-metrics.md
sway_limit_mm = 9      # from tools/envelope_probe
```

Use the actual Task 1 numbers for the two limits.

- [ ] **Step 7: Verify the flying config did not move**

Run: `git diff MotionProviderPlugin/configuration.toml`
Expected: additions only. No existing line changed. If any existing value differs, revert it — the file must stay numerically identical to the acceptance flight.

- [ ] **Step 8: Commit**

```bash
git add MotionProviderPlugin/src/MotionConfig.cpp MotionProviderPlugin/tests/test_config.cpp MotionProviderPlugin/configuration.toml
git commit -m "feat(motion): config keys for the onset channel"
```

---

### Task 5: Telemetry columns

**Files:**
- Modify: `src/Telemetry.cpp:37-54` (header), `src/Telemetry.cpp:84-110` (write)
- Test: `tests/test_telemetry.cpp:59-71`

**Interfaces:**
- Consumes: the `WashoutTrace` fields from Task 3.
- Produces: twelve new CSV columns, appended in this exact order —
  `surge_a_hp, surge_vel, surge_pos_raw, surge_clamped, sway_a_hp, sway_vel, sway_pos_raw, sway_clamped, live_surge, live_sway, cmd_surge, cmd_sway`. Total column count 78.

- [ ] **Step 1: Write the failing test**

In `tests/test_telemetry.cpp`, update the schema assertion (currently `check(hc == 66, ...)`) and add the new-column checks next to the existing `eff_*` ones:

```cpp
        // 66 columns through the slab-joint work; +12 for the surge/sway onset
        // channel (8 trace, live_surge/live_sway, cmd_surge/cmd_sway) = 78.
        // Appended, never inserted: docs/motion-tuning/README.md section 2
        // exports cues with a position-based cut, which inserting breaks
        // silently.
        check(hc == 78, "header has exactly the documented 78-column schema");
        check(headerLine.find(",surge_a_hp,surge_vel,surge_pos_raw,surge_clamped,"
                              "sway_a_hp,sway_vel,sway_pos_raw,sway_clamped")
                  != std::string::npos, "onset trace columns present, in order");
        check(headerLine.find(",live_surge,live_sway,cmd_surge,cmd_sway")
                  != std::string::npos, "onset pose columns present, in order");
        check(headerLine.find(",eff_slab_dur,surge_a_hp") != std::string::npos,
              "new columns are appended after the effects state block");
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd MotionProviderPlugin && cmake --build tests/build && ./tests/build/test_telemetry`
Expected: FAIL — `header has exactly the documented 78-column schema` (it is 66).

- [ ] **Step 3: Extend the header**

In `Telemetry::header()`, append to the end of the returned string literal, after `eff_slab_dur`:

```cpp
           ",surge_a_hp,surge_vel,surge_pos_raw,surge_clamped"
           ",sway_a_hp,sway_vel,sway_pos_raw,sway_clamped"
           ",live_surge,live_sway,cmd_surge,cmd_sway";
```

(The existing final literal chunk ends without a trailing comma, so each new chunk starts with one.)

- [ ] **Step 4: Extend write()**

At the end of `Telemetry::write()`, after the last `eff_*` put:

```cpp
    putD(out_, r.trace.surgeAHp); putD(out_, r.trace.surgeVel);
    putD(out_, r.trace.surgePosRaw); putI(out_, r.trace.surgeClamped ? 1 : 0);
    putD(out_, r.trace.swayAHp);  putD(out_, r.trace.swayVel);
    putD(out_, r.trace.swayPosRaw);  putI(out_, r.trace.swayClamped ? 1 : 0);
    putF(out_, r.live.surge);      putF(out_, r.live.sway);
    putF(out_, r.commanded.surge); putF(out_, r.commanded.sway);
```

`eff_surge`/`eff_sway` are deliberately absent: `EffectsLayer` produces no horizontal component. Add a one-line comment saying so, so the asymmetry with `live_*`/`cmd_*` reads as a decision rather than an omission.

- [ ] **Step 5: Run to verify it passes**

Run: `cd MotionProviderPlugin && cmake --build tests/build && ctest --test-dir tests/build --output-on-failure`
Expected: all eleven suites PASS. The `telemetry` suite's own "header and data column counts agree" check covers the case where Step 3 and Step 4 disagree.

- [ ] **Step 6: Commit**

```bash
git add MotionProviderPlugin/src/Telemetry.cpp MotionProviderPlugin/tests/test_telemetry.cpp
git commit -m "feat(motion): record the onset channel in telemetry"
```

---

### Task 6: Replay — sweepable keys and six-DOF verify

Without the verify extension, `--verify` does not look at the new channel at all and still reports PASS. That is the most dangerous outcome this tool can produce, so it is part of the same task as the keys.

**Files:**
- Modify: `tools/washout_replay.cpp:51-60` (`CueSample`), `:168-180` (recorded-column read), `:210-228` (`kWashoutKeys`), `:440-450` (comparison)

**Interfaces:**
- Consumes: the config fields from Task 3 and the CSV columns from Task 5.
- Produces: `--set washout.surge_gain=...` (and the five siblings) accepted; `--verify` comparing all six live DOF.

- [ ] **Step 1: Add the keys**

In `kWashoutKeys`, after `washout.tilt_rate_limit_dps`:

```cpp
    {"washout.surge_gain",              &WashoutConfig::surgeGain},
    {"washout.sway_gain",               &WashoutConfig::swayGain},
    {"washout.trans_vel_washout_tau",   &WashoutConfig::transVelWashoutTau},
    {"washout.trans_pos_washout_tau",   &WashoutConfig::transPosWashoutTau},
    {"washout.surge_limit_mm",          &WashoutConfig::surgeLimitMm},
    {"washout.sway_limit_mm",           &WashoutConfig::swayLimitMm},
```

- [ ] **Step 2: Carry the recorded columns**

In `struct CueSample`, extend the recorded-output group:

```cpp
    // Recorded outputs, kept for --verify.
    float recLiveHeave = 0.0f, recLiveRoll = 0.0f, recLivePitch = 0.0f, recLiveYaw = 0.0f;
    // Recorded by the 78-column schema onward. A recording that predates those
    // columns reads them as 0, which matches what the filter produced then:
    // the channel did not exist, so live surge/sway were 0.
    float recLiveSurge = 0.0f, recLiveSway = 0.0f;
```

In the loader, inside the `if (haveRec) { ... }` block:

```cpp
            s.recLiveSurge = static_cast<float>(col(f, idx, "live_surge", 0.0));
            s.recLiveSway  = static_cast<float>(col(f, idx, "live_sway",  0.0));
```

Check `col()`'s signature at the top of the file before writing this: if it has no default-value overload, add one that returns the supplied default when the name is absent from `idx`, rather than making the two columns mandatory. Older recordings must stay replayable.

- [ ] **Step 3: Compare six DOF**

In the `if (s.haveRecorded)` block, widen the difference array:

```cpp
            const double d[6] = {
                std::fabs(static_cast<double>(live.heave) - s.recLiveHeave),
                std::fabs(static_cast<double>(live.roll)  - s.recLiveRoll),
                std::fabs(static_cast<double>(live.pitch) - s.recLivePitch),
                std::fabs(static_cast<double>(live.yaw)   - s.recLiveYaw),
                std::fabs(static_cast<double>(live.surge) - s.recLiveSurge),
                std::fabs(static_cast<double>(live.sway)  - s.recLiveSway)};
```

The NaN-latching loop below it already iterates `for (double v : d)`, so it needs no change.

- [ ] **Step 4: Build and check the keys are accepted**

```bash
cd MotionProviderPlugin
cmake --build tools/build
gunzip -c reference/cruise_calm.csv.gz > /tmp/cruise_calm.cues.csv
./tools/build/washout_replay --cues /tmp/cruise_calm.cues.csv --config configuration.toml \
    --set washout.surge_gain=0.5 --out /tmp/surge_probe.csv
```

Expected: runs to completion, exit 0. A key typo shows as exit code 2 and a refusal message, not a silent no-op.

- [ ] **Step 5: Check the regression path is still bit-clean**

```bash
./tools/build/washout_replay --cues /tmp/cruise_calm.cues.csv --config configuration.toml \
    --out /tmp/cruise_calm.csv
tools/.venv/bin/python tools/washout_metrics.py /tmp/cruise_calm.csv
```

Expected: `sat_envelope` still `0.00`, and `peak_out_mm`, `wrms`, `band_ratio`, `lag_ms` matching the values `docs/motion-tuning/tuning-log.md` records for the shipped config. The onset gains are 0 here, so any movement in those numbers is a defect from Task 3, not a tuning effect — stop and find it.

- [ ] **Step 6: Commit**

```bash
git add MotionProviderPlugin/tools/washout_replay.cpp
git commit -m "feat(motion): replay sweeps and verifies surge/sway"
```

---

### Task 7: Metrics and documentation

**Files:**
- Modify: `tools/washout_metrics.py` (`sat` dict, return dict, `HEADERS`, module docstring)
- Modify: `docs/motion-tuning/README.md` (§2 column note, §7 metric table)
- Modify: `MotionProviderPlugin/CLAUDE.md` (washout section)

**Interfaces:**
- Consumes: the `surge_clamped` / `sway_clamped` columns from Task 5.
- Produces: `sat_surge` and `sat_sway` in both the table and `--csv` output.

- [ ] **Step 1: Add the two metrics**

In `washout_metrics.py`, in the `sat = {...}` dict:

```python
        "surge": sat_pct(column(cols, arr, "surge_clamped", path=path)),
        "sway": sat_pct(column(cols, arr, "sway_clamped", path=path)),
```

No `default=` argument: like `heave_clamped`, a missing column could manufacture a favourable 0 %, which is exactly the value the campaign is trying to reach. Add that reason as a comment.

In the returned dict, next to `sat_heave`:

```python
        "sat_surge": sat["surge"],
        "sat_sway": sat["sway"],
```

In `HEADERS`, after `("sat_heave", ">", "{:.2f}")`:

```python
    ("sat_surge", ">", "{:.2f}"),
    ("sat_sway", ">", "{:.2f}"),
```

- [ ] **Step 2: Run it against a replay with the channel on**

```bash
cd MotionProviderPlugin
./tools/build/washout_replay --cues /tmp/cruise_calm.cues.csv --config configuration.toml \
    --set washout.surge_gain=0.5 --set washout.sway_gain=0.5 --out /tmp/surge_probe.csv
tools/.venv/bin/python tools/washout_metrics.py /tmp/surge_probe.csv
```

Expected: the table gains `sat_surge` and `sat_sway` columns with numeric values, and `sat_envelope` is printed as before. Then confirm the guard works:

```bash
# Drop everything from surge_clamped (column 70) onward -- the first of the two
# columns the new metrics require by name.
cut -d, -f1-69 /tmp/surge_probe.csv > /tmp/no_clamp.csv
tools/.venv/bin/python tools/washout_metrics.py /tmp/no_clamp.csv
```

Expected: aborts with `missing required column 'surge_clamped'` — not a table with `sat_surge  0.00` in it. Column order from Task 5: 67 `surge_a_hp`, 68 `surge_vel`, 69 `surge_pos_raw`, 70 `surge_clamped`, 71–74 the sway four, 75–78 `live_surge`, `live_sway`, `cmd_surge`, `cmd_sway`.

- [ ] **Step 3: Update the harness manual**

In `docs/motion-tuning/README.md`:

- §2: after the `cut` command, note that the schema is now 78 columns and that the surge/sway columns were appended precisely so this position-based cut stays valid — and that `washout_replay` will replay a cue file exported before the change, since the two `live_surge`/`live_sway` columns default to 0 when absent.
- §7 metric table: two rows, in the style of the existing ones.

```markdown
| `sat_surge`, `sat_sway` | % of unpaused ticks with the translational onset clamp engaged | Same reading as `sat_heave`. A high value means the onset channel is running into its per-axis limit and the cue is being flattened rather than shaped — lower the gain before raising the limit, because the limit is what keeps `sat_envelope` at zero. |
```

Also add, under the `sat_envelope` discussion: with the onset channel enabled, `sat_envelope` becoming non-zero means surge and sway are stealing from heave and tilt. It is the one failure mode of this feature that no per-channel metric can show.

- [ ] **Step 4: Update the plugin's CLAUDE.md**

In the "Washout filter" section, add the onset channel to the bullet list:

```markdown
- **Translational onset** — the complement of the tilt-coordination low-pass, leaky
  double-integrated to mm and clamped per axis. Renders the onset of a longitudinal or
  lateral acceleration, which tilt coordination cannot: it is low-passed and rate-limited
  by design. Crossover constant is `tilt_lp_tau`, shared with the tilt half, so the two
  cannot double-count. Off by default (`surge_gain`/`sway_gain` = 0). Design:
  `../docs/superpowers/specs/2026-08-31-surge-sway-onset-cues-design.md`.
```

Amend the tilt bullet to say the low-pass now runs on the raw signal with the gain applied outside it, and note in the structural paragraph that `surge_limit_mm`/`sway_limit_mm` exist to keep `clampToReachable` from engaging at all.

- [ ] **Step 5: Run the full suite once more**

Run: `cd MotionProviderPlugin && ctest --test-dir tests/build --output-on-failure`
Expected: eleven suites PASS.

- [ ] **Step 6: Commit**

```bash
git add MotionProviderPlugin/tools/washout_metrics.py docs/motion-tuning/README.md MotionProviderPlugin/CLAUDE.md
git commit -m "feat(motion): sat_surge/sat_sway metrics and docs"
```

---

### Task 8: Settle the sign, and verify on the rig hardware

The first task that needs the platform. Everything before it is host-side.

**Files:**
- Possibly modify: `src/WashoutFilter.cpp` (the `p.surge` / `p.sway` sign, one character each)
- Modify: `docs/motion-tuning/tuning-log.md`

**Interfaces:**
- Consumes: a built and installed plugin from Tasks 3–7.
- Produces: a confirmed sign convention, and a `--verify` PASS on a fresh recording with the channel enabled.

- [ ] **Step 1: Build and install the plugin**

```bash
cd MotionProviderPlugin
./build-macos.sh          # or ./build-windows.sh on the Sim-PC
```

Copy the built `.xpl` into the X-Plane plugin directory as usual, start X-Plane, and check the Status window loads with no error in `Log.txt`.

- [ ] **Step 2: Establish the reference direction by jog**

Arm the platform, switch to manual mode, and jog surge positive (`MotionProvider`'s manual axis 0, `kTransStep`). Note which way the cockpit physically moves. Write it down — this is ground truth, not something to derive from `base_angle_deg`.

- [ ] **Step 3: Check the cue sign against it**

Set `surge_gain = 0.5` in `configuration.toml`, click **Reload config**, and fly a takeoff push. The platform must translate in the same direction a forward acceleration would push the cockpit — the same direction the tilt channel already leans for (nose-up on forward acceleration presses the pilot back).

If it moves the wrong way, negate `p.surge` in `WashoutFilter.cpp` (`p.surge = static_cast<float>(-out[0]);`), rebuild, and re-check. Repeat for sway with a crosswind or a rudder input.

- [ ] **Step 4: Record and verify**

Press **Record** in the Status window *after* arming, fly two minutes including one takeoff push and one braking application, press **Stop Rec**. Then:

```bash
cd MotionProviderPlugin
./tools/build/washout_replay --cues motion-YYYYMMDD-HHMMSS.csv --config configuration.toml --verify
```

Expected: `verify: PASS`. This is the gate that proves the new filter state is fully captured by the recorded columns. A FAIL here means telemetry is missing something the filter depends on — fix that, do not proceed to sweeps.

Use the same `configuration.toml` that was flown. If any value was changed after the recording, replay with `--set` to reconstruct the flown config.

- [ ] **Step 5: Export a reference cue file**

```bash
cut -d, -f2,4-16,58-61 motion-YYYYMMDD-HHMMSS.csv | gzip > reference/onset_takeoff.csv.gz
```

Confirm the field numbers still match the current header before trusting the cut — it is position-based:

```bash
head -1 motion-YYYYMMDD-HHMMSS.csv | tr ',' '\n' | nl | sed -n '1,20p;55,80p'
```

- [ ] **Step 6: Log it and commit**

Add a dated entry to `docs/motion-tuning/tuning-log.md`: the sign as confirmed (and whether it had to be flipped), the recording name, the `--verify` result.

```bash
git add MotionProviderPlugin/reference/onset_takeoff.csv.gz docs/motion-tuning/tuning-log.md MotionProviderPlugin/src/WashoutFilter.cpp
git commit -m "test(motion): confirm onset sign, verify on the rig"
```

---

### Task 9: Offline sweeps and candidate selection

**Files:**
- Modify: `docs/motion-tuning/tuning-log.md`

**Interfaces:**
- Consumes: `reference/onset_takeoff.csv.gz` from Task 8 plus the existing `cruise_calm`, `ground_takeoff`, `approach_landing` references.
- Produces: one candidate `(surge_gain, sway_gain, trans_vel_washout_tau, trans_pos_washout_tau)` that clears every gate, ready to fly.

- [ ] **Step 1: Establish the baseline row for each reference**

```bash
cd MotionProviderPlugin
for f in cruise_calm ground_takeoff approach_landing onset_takeoff; do
  gunzip -c reference/$f.csv.gz > /tmp/$f.cues.csv
  ./tools/build/washout_replay --cues /tmp/$f.cues.csv --config configuration.toml --out /tmp/$f.base.csv
done
tools/.venv/bin/python tools/washout_metrics.py /tmp/*.base.csv
```

Record the table. Gains are 0 here, so this is the shipped platform — the reference every candidate is compared against.

- [ ] **Step 2: Sweep the surge gain**

```bash
./tools/build/washout_replay --cues /tmp/onset_takeoff.cues.csv --config configuration.toml \
    --sweep washout.surge_gain=0.2,0.4,0.6,0.8,1.0 --out /tmp/sg
tools/.venv/bin/python tools/washout_metrics.py /tmp/sg.*.csv
```

- [ ] **Step 3: Sweep the translational washout pair**

The two are the two poles of one washout and move together, as the heave campaign did. Run one sweep per pinned value:

```bash
for tau in 0.15 0.25 0.4 0.6; do
  ./tools/build/washout_replay --cues /tmp/onset_takeoff.cues.csv --config configuration.toml \
      --set washout.surge_gain=0.5 \
      --set washout.trans_vel_washout_tau=$tau --set washout.trans_pos_washout_tau=$tau \
      --out /tmp/tau_$tau.csv
done
tools/.venv/bin/python tools/washout_metrics.py /tmp/tau_*.csv
```

Note the budget check while reading these: the crossover is `tilt_lp_tau = 1.5 s`, but a short `trans_*_washout_tau` shortens the cue's effective time constant. Below ≈0.24 s at 20 mm the ≈363 mm/s² acceleration ceiling starts to bind, and `sat_sl_acc` is where that shows.

- [ ] **Step 4: Apply the gates**

A candidate is only a candidate if all four hold, on **every** reference file:

1. `sat_envelope` = 0.00 %. Non-zero means surge and sway are stealing from heave and tilt. Lower `surge_limit_mm`/`sway_limit_mm`, not the gate.
2. `lag_ms` ≤ the Step 1 baseline + 15 ms.
3. `band_ratio` not above the baseline.
4. `sat_surge` / `sat_sway` low enough that the cue is shaped rather than flattened — treat sustained clamping as a reason to lower the gain.

Also re-check at a PC-typical framerate, because a double integrator is dt-sensitive:

```bash
./tools/build/washout_replay --cues /tmp/onset_takeoff.cues.csv --config configuration.toml \
    --set washout.surge_gain=<candidate> --resample-dt 0.0111 --out /tmp/cand_90fps.csv
tools/.venv/bin/python tools/washout_metrics.py /tmp/cand_90fps.csv
```

- [ ] **Step 5: Log the candidate**

Add the sweep tables and the chosen candidate to `docs/motion-tuning/tuning-log.md`, with the reasoning for the choice — including any value that was tried and rejected, which is the part the log exists for.

```bash
git add docs/motion-tuning/tuning-log.md
git commit -m "tune(motion): offline candidate for onset cues"
```

---

### Task 10: Rig sessions and acceptance

**Files:**
- Modify: `MotionProviderPlugin/configuration.toml`
- Modify: `docs/motion-tuning/tuning-log.md`
- Modify: `docs/superpowers/specs/2026-08-30-surge-sway-onset-cues-ticket.md`

**Interfaces:**
- Consumes: the candidate from Task 9.
- Produces: adopted values in the flying config, a signed-off acceptance recording, a closed ticket.

- [ ] **Step 1: Fly the candidate on the three phases**

Set the candidate values, **Reload config**, and fly each in turn: takeoff push, braking application, crosswind landing. Judge each separately and write the verdict down before flying the next.

- [ ] **Step 2: Answer the design's open question**

The one thing no offline metric can decide: **is the handover from translation to tilt felt as one cue or two?** If it reads as two — a shove followed by a separate, unrelated lean — the candidate is not the problem and neither is the gain. The structure is, and the fallback is approach C from the design (tilt-error-driven handover). Record which it was.

- [ ] **Step 3: Iterate or advance**

If a phase fails, go back to Task 9 with what the rig showed. Do not tune at the rig without a replay behind it — that is what produced the two 14×-over effects.

- [ ] **Step 4: Acceptance flight**

Fly a full flight with the adopted values and telemetry recording on. Confirm afterwards:

```bash
cd MotionProviderPlugin
./tools/build/washout_replay --cues abnahme-motion-YYYYMMDD-HHMMSS.csv --config configuration.toml --verify
tools/.venv/bin/python tools/washout_metrics.py /tmp/abnahme.replay.csv
```

Expected: `verify: PASS`, `sat_envelope` 0.00 %, `lag_ms` inside the gate.

- [ ] **Step 5: Adopt the values**

Update the six keys in `configuration.toml` to the flown values, each with an `# adopted YYYY-MM-DD` comment in the style of the surrounding lines. Update the file's header comment to name the new acceptance recording.

- [ ] **Step 6: Close the loop**

- `docs/motion-tuning/tuning-log.md`: the campaign entry — what was tried, what was rejected and why, the acceptance recording's name.
- `docs/superpowers/specs/2026-08-30-surge-sway-onset-cues-ticket.md`: mark **Status: done**, pointing at the design and this plan.
- `MotionProviderPlugin/CLAUDE.md`: the washout section's "tuned, not defaults" paragraph gains the onset channel.

```bash
git add MotionProviderPlugin/configuration.toml docs/motion-tuning/tuning-log.md docs/superpowers/specs/2026-08-30-surge-sway-onset-cues-ticket.md MotionProviderPlugin/CLAUDE.md
git commit -m "tune(motion): adopt surge/sway onset cues"
```
