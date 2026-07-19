# MotionProviderPlugin — Phase 3 (Washout filter + effects) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the sampled flight-model cues into a commanded platform pose via a pragmatic classical washout filter plus an additive effects layer (touchdown bump + ground rumble), replacing the AUTO-mode attitude placeholder. All parameters live in the hot-reloadable TOML.

**Architecture:** Two pure-math, stateful modules — `WashoutFilter` (cues → washed pose) and `EffectsLayer` (cues → additive pose offset) — each driven by an injected timestep `dt` and configured by a plain struct with `defaults()`. `MotionConfig` loads both from `~/.motionprovider.toml`. In AUTO mode `MotionProvider` runs `washout + effects` each 60 Hz tick; MANUAL mode still bypasses them. Both modules are unit-tested off-device.

**Tech Stack:** C++17, `<cmath>`, existing CMake + native test target; toml++ for config.

## Global Constraints

- C++17. `WashoutConfig.h`, `WashoutFilter.*`, `EffectsConfig.h`, `EffectsLayer.*` MUST NOT include any X-Plane SDK header (native-testable).
- Filters are **stateful and deterministic** (no RNG, no wall-clock): all time dependence comes from the `dt` argument. Rumble/oscillation uses advancing sine phase, not random noise.
- SANDBOX: `cmake`/`clang`/`make` BLOCKED. Implementers WRITE FILES ONLY; the user builds + runs the native tests.
- Units: cues are as sampled (specific forces in g, rates in deg/s). Internally use SI where noted (`G = 9.80665` m/s²). Pose output: translations mm, rotations deg — same `Pose` struct as the IK.
- Config sign convention: default gains are positive; if a cue drives the platform the wrong way on the rig, the fix is a negative gain in the TOML (do not hard-flip in code).
- Work on branch `feature/motion-provider-plugin`.

## Filter design (authoritative)

Building blocks (per channel, per tick, `dt` seconds):
- Low-pass of x toward state `s`: `s += lpAlpha(dt,tau) * (x - s)`, `lpAlpha = dt/(tau+dt)`.
- High-pass of x: `x - lowpass(x)`.
- Leaky integrator / washout decay factor: `leak(dt,tau) = exp(-dt/tau)` (state `*= leak` each tick → returns to 0 with time-constant tau).
- Rate limit `cur` toward `tgt` at `rate` deg/s: step `= rate*dt`; move `cur` toward `tgt` by at most `step`.

Channels:
- **Heave (Z, mm):** `a = heaveGain*(heaveG-1)*G` → high-pass (removes sustained/bias) → leaky-integrate to velocity → leaky-integrate to position (×1000 m→mm) → clamp ±heaveLimitMm. Sustained accel washes out; transients produce a bump.
- **Tilt-coordination (surge/sway → pitch/roll, deg):** low-pass the horizontal specific force to get the *sustained* component, convert to the tilt whose gravity component matches it (`asin(a/G)`), clamp to tiltLimitDeg, and **rate-limit** the tilt so the lean itself isn't felt as rotation.
- **Rotational (roll/pitch/yaw, deg):** `w = rotGain*rate` (deg/s) → high-pass (removes sustained rotation) → integrate to angle → apply a slow washout leak (rotWashoutTau) so it recenters → clamp ±rotLimitDeg.
- **Final pose:** surge=sway=0 (pragmatic mode); heave from heave channel; roll = tiltRoll + rotRoll; pitch = tiltPitch + rotPitch; yaw = rotYaw.

Effects (additive Pose offset, summed onto the washout pose):
- **Touchdown bump:** on the rising edge of `onGround`, start a decaying sine on heave: `-touchdownGain * exp(-t/decayTau) * sin(2π*freq*t)` until the envelope is negligible.
- **Ground rumble:** while `onGround && groundspeed > 0.5 m/s`, add `amp*sin(phase)` to heave (amp = rumbleGain × min(1, groundspeed/rumbleSpeedRefMps)) plus a small out-of-phase pitch jitter; phase advances by `2π*rumbleFreqHz*dt`.
- **Engine vibration / buffet:** config fields reserved (gain defaults 0), not wired this phase.

## File structure

- Create `src/WashoutConfig.h`, `src/WashoutFilter.h`, `src/WashoutFilter.cpp`.
- Create `src/EffectsConfig.h`, `src/EffectsLayer.h`, `src/EffectsLayer.cpp`.
- Create `tests/test_washout.cpp`, `tests/test_effects.cpp`.
- Modify `src/MotionConfig.h/.cpp` (load `[washout]`, `[effects]`), `tests/test_config.cpp`.
- Modify `src/MotionProvider.h/.cpp` (run washout+effects in AUTO), `src/StatusWindow.cpp` (label).
- Modify `CMakeLists.txt`, `tests/CMakeLists.txt`.

---

### Task 1: WashoutFilter (pure math) + tests

**Files:**
- Create: `src/WashoutConfig.h`, `src/WashoutFilter.h`, `src/WashoutFilter.cpp`, `tests/test_washout.cpp`
- Modify: `CMakeLists.txt` (plugin SOURCES/HEADERS), `tests/CMakeLists.txt` (test_washout target)

**Interfaces:**
- Produces: `struct WashoutConfig` (fields below) + `defaults()`; `class WashoutFilter` with `explicit WashoutFilter(const WashoutConfig&)`, `Pose update(const MotionCues&, double dt)`, `void reset()`, `void setConfig(const WashoutConfig&)`.

- [ ] **Step 1: Write `src/WashoutConfig.h`**

```cpp
#pragma once

// Pragmatic classical washout parameters. Times in seconds, gains dimensionless
// unless noted. Defaults are conservative starting points for tuning.
struct WashoutConfig {
    // Heave (vertical translation), driven by (g_nrml - 1)
    double heaveGain          = 0.5;
    double heaveHpTau         = 1.0;   // high-pass on accel
    double heaveVelWashoutTau = 2.0;   // leaky velocity integrator
    double heavePosWashoutTau = 2.0;   // leaky position integrator
    double heaveLimitMm       = 40.0;

    // Tilt-coordination: sustained surge/sway specific force -> pitch/roll
    double tiltSurgeGain   = 1.0;      // g_axil -> pitch
    double tiltSwayGain    = 1.0;      // g_side -> roll
    double tiltLpTau       = 1.5;      // low-pass to extract sustained accel
    double tiltLimitDeg    = 6.0;
    double tiltRateLimitDps = 5.0;     // max tilt-coordination rate

    // Rotational: angular rate -> angle (high-pass + washout)
    double rotRollGain   = 0.7;
    double rotPitchGain  = 0.7;
    double rotYawGain    = 0.7;
    double rotHpTau      = 1.0;        // high-pass on rate
    double rotWashoutTau = 3.0;        // slow angle recentering
    double rotLimitDeg   = 6.0;

    static WashoutConfig defaults() { return WashoutConfig{}; }
};
```

- [ ] **Step 2: Write `src/WashoutFilter.h`**

```cpp
#pragma once
#include "Pose.h"
#include "MotionCues.h"
#include "WashoutConfig.h"

// Classical (pragmatic) motion-cueing washout: flight cues -> platform pose.
// Stateful; all time dependence is via the dt argument. No X-Plane deps.
class WashoutFilter {
public:
    explicit WashoutFilter(const WashoutConfig& cfg);

    Pose update(const MotionCues& cues, double dt);
    void reset();
    void setConfig(const WashoutConfig& cfg) { cfg_ = cfg; }

private:
    WashoutConfig cfg_;

    // Heave state
    double heaveAccelLp_ = 0.0;
    double heaveVel_ = 0.0;
    double heavePos_ = 0.0;

    // Tilt-coordination state
    double surgeLp_ = 0.0;
    double swayLp_ = 0.0;
    double tiltPitch_ = 0.0;
    double tiltRoll_ = 0.0;

    // Rotational state
    double rollRateLp_ = 0.0, pitchRateLp_ = 0.0, yawRateLp_ = 0.0;
    double rollAngle_ = 0.0, pitchAngle_ = 0.0, yawAngle_ = 0.0;
};
```

- [ ] **Step 3: Write `src/WashoutFilter.cpp`**

```cpp
#include "WashoutFilter.h"
#include <cmath>

namespace {
constexpr double G = 9.80665;
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;

double lpAlpha(double dt, double tau) { return tau > 0.0 ? dt / (tau + dt) : 1.0; }
double leak(double dt, double tau)    { return tau > 0.0 ? std::exp(-dt / tau) : 0.0; }
double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

double rateLimit(double cur, double tgt, double ratePerSec, double dt) {
    const double step = ratePerSec * dt;
    const double d = tgt - cur;
    if (d >  step) return cur + step;
    if (d < -step) return cur - step;
    return tgt;
}
}  // namespace

WashoutFilter::WashoutFilter(const WashoutConfig& cfg) : cfg_(cfg) {}

void WashoutFilter::reset() {
    heaveAccelLp_ = heaveVel_ = heavePos_ = 0.0;
    surgeLp_ = swayLp_ = tiltPitch_ = tiltRoll_ = 0.0;
    rollRateLp_ = pitchRateLp_ = yawRateLp_ = 0.0;
    rollAngle_ = pitchAngle_ = yawAngle_ = 0.0;
}

Pose WashoutFilter::update(const MotionCues& c, double dt) {
    if (dt <= 0.0) dt = 1.0 / 60.0;

    // --- Heave: HP(accel) -> leaky double-integrate -> mm ---
    const double aZ = cfg_.heaveGain * (static_cast<double>(c.heaveG) - 1.0) * G;
    heaveAccelLp_ += lpAlpha(dt, cfg_.heaveHpTau) * (aZ - heaveAccelLp_);
    const double aHp = aZ - heaveAccelLp_;
    heaveVel_ = heaveVel_ * leak(dt, cfg_.heaveVelWashoutTau) + aHp * dt;
    heavePos_ = heavePos_ * leak(dt, cfg_.heavePosWashoutTau) + heaveVel_ * dt * 1000.0;
    heavePos_ = clampd(heavePos_, -cfg_.heaveLimitMm, cfg_.heaveLimitMm);

    // --- Tilt-coordination: sustained horizontal accel -> rate-limited tilt ---
    const double aX = cfg_.tiltSurgeGain * static_cast<double>(c.surgeG) * G;
    const double aY = cfg_.tiltSwayGain  * static_cast<double>(c.swayG)  * G;
    surgeLp_ += lpAlpha(dt, cfg_.tiltLpTau) * (aX - surgeLp_);
    swayLp_  += lpAlpha(dt, cfg_.tiltLpTau) * (aY - swayLp_);
    double tgtPitch = std::asin(clampd(surgeLp_ / G, -1.0, 1.0)) * kRad2Deg;
    double tgtRoll  = std::asin(clampd(swayLp_  / G, -1.0, 1.0)) * kRad2Deg;
    tgtPitch = clampd(tgtPitch, -cfg_.tiltLimitDeg, cfg_.tiltLimitDeg);
    tgtRoll  = clampd(tgtRoll,  -cfg_.tiltLimitDeg, cfg_.tiltLimitDeg);
    tiltPitch_ = rateLimit(tiltPitch_, tgtPitch, cfg_.tiltRateLimitDps, dt);
    tiltRoll_  = rateLimit(tiltRoll_,  tgtRoll,  cfg_.tiltRateLimitDps, dt);

    // --- Rotational: HP(rate) -> integrate -> washout leak ---
    auto rotChan = [&](double gain, double rate, double& rateLp, double& angle) {
        const double w = gain * rate;                // deg/s
        rateLp += lpAlpha(dt, cfg_.rotHpTau) * (w - rateLp);
        const double wHp = w - rateLp;
        angle = (angle + wHp * dt) * leak(dt, cfg_.rotWashoutTau);
        angle = clampd(angle, -cfg_.rotLimitDeg, cfg_.rotLimitDeg);
    };
    rotChan(cfg_.rotRollGain,  c.rollRate,  rollRateLp_,  rollAngle_);
    rotChan(cfg_.rotPitchGain, c.pitchRate, pitchRateLp_, pitchAngle_);
    rotChan(cfg_.rotYawGain,   c.yawRate,   yawRateLp_,   yawAngle_);

    Pose p;
    p.surge = 0.0f;
    p.sway  = 0.0f;
    p.heave = static_cast<float>(heavePos_);
    p.roll  = static_cast<float>(tiltRoll_  + rollAngle_);
    p.pitch = static_cast<float>(tiltPitch_ + pitchAngle_);
    p.yaw   = static_cast<float>(yawAngle_);
    return p;
}
```

- [ ] **Step 4: Write `tests/test_washout.cpp`**

```cpp
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

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
```

- [ ] **Step 5: Wire builds.** In `CMakeLists.txt` add `src/WashoutFilter.cpp` to SOURCES and `src/WashoutFilter.h`, `src/WashoutConfig.h` to HEADERS. In `tests/CMakeLists.txt` add:

```cmake
add_executable(test_washout test_washout.cpp ../src/WashoutFilter.cpp)
target_include_directories(test_washout PRIVATE ../src)
add_test(NAME washout COMMAND test_washout)
```

- [ ] **Step 6: Commit**

```bash
git add MotionProviderPlugin/src/WashoutConfig.h MotionProviderPlugin/src/WashoutFilter.h MotionProviderPlugin/src/WashoutFilter.cpp MotionProviderPlugin/tests/test_washout.cpp MotionProviderPlugin/CMakeLists.txt MotionProviderPlugin/tests/CMakeLists.txt
git commit -m "feat(motion): Phase 3 Task 1 - pragmatic classical washout filter + tests"
```

- [ ] **Step 7: Manual test (user)** — `cd MotionProviderPlugin/tests && cmake -B build && cmake --build build && ./build/test_washout` → `0 failures`.

---

### Task 2: EffectsLayer (pure math) + tests

**Files:**
- Create: `src/EffectsConfig.h`, `src/EffectsLayer.h`, `src/EffectsLayer.cpp`, `tests/test_effects.cpp`
- Modify: `CMakeLists.txt`, `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `struct EffectsConfig` + `defaults()`; `class EffectsLayer` with `explicit EffectsLayer(const EffectsConfig&)`, `Pose update(const MotionCues&, double dt)` (returns an additive offset), `void reset()`, `void setConfig(const EffectsConfig&)`.

- [ ] **Step 1: Write `src/EffectsConfig.h`**

```cpp
#pragma once

// Additive motion effects. Amplitudes in mm (heave) / deg (jitter), freqs in Hz.
struct EffectsConfig {
    // Gear touchdown bump (decaying sine on heave, triggered on landing)
    double touchdownGain    = 8.0;   // mm peak
    double touchdownFreqHz  = 6.0;
    double touchdownDecayTau = 0.25; // s

    // Ground-roll rumble (speed-scaled sine while rolling)
    double rumbleGain        = 2.0;  // mm at reference speed
    double rumbleFreqHz      = 12.0;
    double rumbleSpeedRefMps = 40.0; // full amplitude at/above this groundspeed

    // Reserved (wired in a later phase), no output while gain == 0
    double engineGain = 0.0;
    double buffetGain = 0.0;

    static EffectsConfig defaults() { return EffectsConfig{}; }
};
```

- [ ] **Step 2: Write `src/EffectsLayer.h`**

```cpp
#pragma once
#include "Pose.h"
#include "MotionCues.h"
#include "EffectsConfig.h"

// Additive motion effects layered on top of the washout pose. Stateful; time
// via dt only (deterministic - phase-advanced sines, no RNG). No X-Plane deps.
class EffectsLayer {
public:
    explicit EffectsLayer(const EffectsConfig& cfg);

    Pose update(const MotionCues& cues, double dt);  // additive offset
    void reset();
    void setConfig(const EffectsConfig& cfg) { cfg_ = cfg; }

private:
    EffectsConfig cfg_;
    bool   prevOnGround_ = false;
    bool   tdActive_ = false;
    double tdT_ = 0.0;          // seconds since touchdown
    double rumblePhase_ = 0.0;  // rad
};
```

- [ ] **Step 3: Write `src/EffectsLayer.cpp`**

```cpp
#include "EffectsLayer.h"
#include <cmath>

namespace {
constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
double clampd(double v, double lo, double hi){ return v<lo?lo:(v>hi?hi:v); }
}

EffectsLayer::EffectsLayer(const EffectsConfig& cfg) : cfg_(cfg) {}

void EffectsLayer::reset() {
    prevOnGround_ = false;
    tdActive_ = false;
    tdT_ = 0.0;
    rumblePhase_ = 0.0;
}

Pose EffectsLayer::update(const MotionCues& c, double dt) {
    if (dt <= 0.0) dt = 1.0 / 60.0;
    Pose off;  // all zero

    // Touchdown bump on rising edge of onGround.
    if (c.onGround && !prevOnGround_) { tdActive_ = true; tdT_ = 0.0; }
    prevOnGround_ = c.onGround;
    if (tdActive_) {
        tdT_ += dt;
        const double env = std::exp(-tdT_ / cfg_.touchdownDecayTau);
        off.heave += static_cast<float>(
            -cfg_.touchdownGain * env * std::sin(kTwoPi * cfg_.touchdownFreqHz * tdT_));
        if (env < 0.02) tdActive_ = false;
    }

    // Ground-roll rumble while moving on the ground.
    if (c.onGround && c.groundspeed > 0.5f) {
        const double amp = cfg_.rumbleGain *
            clampd(static_cast<double>(c.groundspeed) / cfg_.rumbleSpeedRefMps, 0.0, 1.0);
        rumblePhase_ += kTwoPi * cfg_.rumbleFreqHz * dt;
        if (rumblePhase_ > kTwoPi) rumblePhase_ -= kTwoPi;
        off.heave += static_cast<float>(amp * std::sin(rumblePhase_));
        off.pitch += static_cast<float>(0.1 * amp * std::sin(rumblePhase_ * 1.7));
    }

    // engineGain / buffetGain reserved (not wired this phase).
    return off;
}
```

- [ ] **Step 4: Write `tests/test_effects.cpp`**

```cpp
#include "EffectsLayer.h"
#include "EffectsConfig.h"
#include "MotionCues.h"
#include <cstdio>
#include <cmath>

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

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
```

- [ ] **Step 5: Wire builds.** `CMakeLists.txt`: add `src/EffectsLayer.cpp` to SOURCES, `src/EffectsLayer.h`, `src/EffectsConfig.h` to HEADERS. `tests/CMakeLists.txt`:

```cmake
add_executable(test_effects test_effects.cpp ../src/EffectsLayer.cpp)
target_include_directories(test_effects PRIVATE ../src)
add_test(NAME effects COMMAND test_effects)
```

- [ ] **Step 6: Commit**

```bash
git add MotionProviderPlugin/src/EffectsConfig.h MotionProviderPlugin/src/EffectsLayer.h MotionProviderPlugin/src/EffectsLayer.cpp MotionProviderPlugin/tests/test_effects.cpp MotionProviderPlugin/CMakeLists.txt MotionProviderPlugin/tests/CMakeLists.txt
git commit -m "feat(motion): Phase 3 Task 2 - additive effects layer (touchdown + rumble) + tests"
```

- [ ] **Step 7: Manual test (user)** — `cmake --build build && ./build/test_effects` → `0 failures`.

---

### Task 3: Load `[washout]` and `[effects]` from TOML

**Files:**
- Modify: `src/MotionConfig.h`, `src/MotionConfig.cpp`, `tests/test_config.cpp`

**Interfaces:**
- Produces: `MotionConfig::loadWashout(const std::string& path) -> WashoutConfig` and `loadEffects(const std::string& path) -> EffectsConfig`, both with the same missing-file/absent-key → defaults contract and lenient int/float parsing as `loadGeometry`.

- [ ] **Step 1: Update `src/MotionConfig.h`** — add includes and declarations:

```cpp
#pragma once
#include <string>
#include "StewartGeometry.h"
#include "WashoutConfig.h"
#include "EffectsConfig.h"

namespace MotionConfig {
    std::string defaultPath();
    StewartGeometry loadGeometry(const std::string& path, bool* outLoaded = nullptr);
    WashoutConfig   loadWashout(const std::string& path);
    EffectsConfig   loadEffects(const std::string& path);
}
```

- [ ] **Step 2: Update `src/MotionConfig.cpp`** — reuse the existing file-scope `getDouble` helper. Add, after `loadGeometry`:

```cpp
WashoutConfig MotionConfig::loadWashout(const std::string& path) {
    WashoutConfig w = WashoutConfig::defaults();
    toml::table tbl;
    try { tbl = toml::parse_file(path); } catch (const toml::parse_error&) { return w; }
    if (auto t = tbl["washout"].as_table()) {
        getDouble(*t, "heave_gain", w.heaveGain);
        getDouble(*t, "heave_hp_tau", w.heaveHpTau);
        getDouble(*t, "heave_vel_washout_tau", w.heaveVelWashoutTau);
        getDouble(*t, "heave_pos_washout_tau", w.heavePosWashoutTau);
        getDouble(*t, "heave_limit_mm", w.heaveLimitMm);
        getDouble(*t, "tilt_surge_gain", w.tiltSurgeGain);
        getDouble(*t, "tilt_sway_gain", w.tiltSwayGain);
        getDouble(*t, "tilt_lp_tau", w.tiltLpTau);
        getDouble(*t, "tilt_limit_deg", w.tiltLimitDeg);
        getDouble(*t, "tilt_rate_limit_dps", w.tiltRateLimitDps);
        getDouble(*t, "rot_roll_gain", w.rotRollGain);
        getDouble(*t, "rot_pitch_gain", w.rotPitchGain);
        getDouble(*t, "rot_yaw_gain", w.rotYawGain);
        getDouble(*t, "rot_hp_tau", w.rotHpTau);
        getDouble(*t, "rot_washout_tau", w.rotWashoutTau);
        getDouble(*t, "rot_limit_deg", w.rotLimitDeg);
    }
    return w;
}

EffectsConfig MotionConfig::loadEffects(const std::string& path) {
    EffectsConfig e = EffectsConfig::defaults();
    toml::table tbl;
    try { tbl = toml::parse_file(path); } catch (const toml::parse_error&) { return e; }
    if (auto t = tbl["effects"].as_table()) {
        getDouble(*t, "touchdown_gain", e.touchdownGain);
        getDouble(*t, "touchdown_freq_hz", e.touchdownFreqHz);
        getDouble(*t, "touchdown_decay_tau", e.touchdownDecayTau);
        getDouble(*t, "rumble_gain", e.rumbleGain);
        getDouble(*t, "rumble_freq_hz", e.rumbleFreqHz);
        getDouble(*t, "rumble_speed_ref_mps", e.rumbleSpeedRefMps);
        getDouble(*t, "engine_gain", e.engineGain);
        getDouble(*t, "buffet_gain", e.buffetGain);
    }
    return e;
}
```

Note: `getDouble` is currently in the anonymous namespace of `MotionConfig.cpp` — it is already visible to these functions in the same translation unit. No header change needed for it.

- [ ] **Step 3: Extend `tests/test_config.cpp`** — add a block before the final print:

```cpp
    // [washout] / [effects] partial override + defaults.
    {
        std::string tmp = tmpPath();
        { std::ofstream f(tmp);
          f << "[washout]\n"
               "heave_gain = 0.9\n"
               "tilt_limit_deg = 4\n"          // int for a double field
               "[effects]\n"
               "rumble_gain = 3.5\n"; }
        WashoutConfig w = MotionConfig::loadWashout(tmp);
        EffectsConfig e = MotionConfig::loadEffects(tmp);
        WashoutConfig wd = WashoutConfig::defaults();
        EffectsConfig ed = EffectsConfig::defaults();
        near(w.heaveGain, 0.9, "washout heave_gain override");
        near(w.tiltLimitDeg, 4.0, "washout tilt_limit_deg int override");
        near(w.rotRollGain, wd.rotRollGain, "washout default kept");
        near(e.rumbleGain, 3.5, "effects rumble_gain override");
        near(e.touchdownGain, ed.touchdownGain, "effects default kept");
        std::remove(tmp.c_str());
    }
```
Add `#include "WashoutConfig.h"` and `#include "EffectsConfig.h"` at the top of the test (they arrive transitively via `MotionConfig.h`, but include explicitly). Update `tests/CMakeLists.txt` `test_config` include dirs if needed — it already includes `../src`, which now contains the new headers, so no change.

- [ ] **Step 4: Commit**

```bash
git add MotionProviderPlugin/src/MotionConfig.h MotionProviderPlugin/src/MotionConfig.cpp MotionProviderPlugin/tests/test_config.cpp
git commit -m "feat(motion): Phase 3 Task 3 - load [washout] and [effects] from TOML"
```

- [ ] **Step 5: Manual test (user)** — `cmake --build build && ./build/test_config` → `0 failures`.

---

### Task 4: Drive AUTO mode from washout + effects

**Files:**
- Modify: `src/MotionProvider.h`, `src/MotionProvider.cpp`, `src/StatusWindow.cpp`

**Interfaces:**
- Consumes: `WashoutFilter`, `EffectsLayer`, `MotionConfig::loadWashout/loadEffects`. AUTO pose = `washout.update(cues, dt) + effects.update(cues, dt)`; MANUAL unchanged. Reload re-reads all three configs and resets the filters; switching to AUTO resets the filters for a clean start.

- [ ] **Step 1: Update `src/MotionProvider.h`** — add includes and members:

```cpp
#include "WashoutFilter.h"
#include "EffectsLayer.h"
```
Add private members:
```cpp
    std::unique_ptr<WashoutFilter> washout_;
    std::unique_ptr<EffectsLayer> effects_;
```

- [ ] **Step 2: Update `src/MotionProvider.cpp`**

Add includes:
```cpp
#include "WashoutFilter.h"
#include "EffectsLayer.h"
```

In `initialize()` (after `kin_` is built), construct the filters from config:
```cpp
    washout_ = std::make_unique<WashoutFilter>(MotionConfig::loadWashout(MotionConfig::defaultPath()));
    effects_ = std::make_unique<EffectsLayer>(MotionConfig::loadEffects(MotionConfig::defaultPath()));
```

In `shutdown()`, add `washout_.reset(); effects_.reset();` (unique_ptr reset).

Replace `reloadConfig()` so it also re-reads and resets the filters:
```cpp
void MotionProvider::reloadConfig() {
    bool loaded = false;
    const std::string path = MotionConfig::defaultPath();
    kin_ = std::make_unique<StewartKinematics>(MotionConfig::loadGeometry(path, &loaded));
    if (washout_) { washout_->setConfig(MotionConfig::loadWashout(path)); washout_->reset(); }
    if (effects_) { effects_->setConfig(MotionConfig::loadEffects(path)); effects_->reset(); }
    lastReloadOk_ = loaded;
    reloadFlashRemaining_ = 2.0f;
}
```

In `onUiAction`, when toggling into AUTO, reset the filters so they start clean (avoids a jump from stale state). Change the `UI_TOGGLE_MODE` case:
```cpp
        case UI_TOGGLE_MODE:
            manualMode_ = !manualMode_;
            if (!manualMode_ && washout_ && effects_) { washout_->reset(); effects_->reset(); }
            break;
```
(Leave the immediate re-solve at the end of `onUiAction` as-is; in AUTO it will re-solve with the current — freshly reset — pose.)

Replace the pose computation in `onFlightLoopTick`:
```cpp
    Pose pose;
    if (manualMode_) {
        pose = manualPose_;
    } else if (washout_ && effects_) {
        Pose w = washout_->update(latestCues_, static_cast<double>(elapsedSec));
        Pose e = effects_->update(latestCues_, static_cast<double>(elapsedSec));
        pose.surge = w.surge + e.surge;
        pose.sway  = w.sway  + e.sway;
        pose.heave = w.heave + e.heave;
        pose.roll  = w.roll  + e.roll;
        pose.pitch = w.pitch + e.pitch;
        pose.yaw   = w.yaw   + e.yaw;
    }
    if (kin_) latestSolve_ = kin_->solve(pose);
```
(Remove the old clamped-attitude placeholder and the `<algorithm>` clamp lambda if now unused. Keep `#include <algorithm>` only if still referenced; otherwise drop it.)

Note: `onUiAction`'s immediate re-solve uses `manualPose_` only in MANUAL. In AUTO, do not advance the washout there (no dt); just `pushStatus()` and let the next tick re-solve. Adjust the tail of `onUiAction`:
```cpp
    if (kin_ && manualMode_) latestSolve_ = kin_->solve(manualPose_);
    pushStatus();
```

- [ ] **Step 3: Update `src/StatusWindow.cpp`** — change the AUTO line label in `draw()` from the placeholder text to reflect real cueing:

```cpp
        drawString(x, y, "AUTO: washout + effects motion cueing",
                   0.7f, 0.8f, 0.9f);
```

- [ ] **Step 4: Commit**

```bash
git add MotionProviderPlugin/src/MotionProvider.h MotionProviderPlugin/src/MotionProvider.cpp MotionProviderPlugin/src/StatusWindow.cpp
git commit -m "feat(motion): Phase 3 Task 4 - AUTO pose from washout + effects"
```

- [ ] **Step 5: Manual verification (user)** — build + load. In AUTO: level flight ≈ home (~32640) with a little rumble on the ground; roll/pitch the aircraft → platform leans then washes back (tilt-coordination + rotational washout); pull g → heave bump then recenters; on touchdown a bump; taxiing adds speed-scaled rumble. Add a `[washout]`/`[effects]` block to `~/.motionprovider.toml`, tweak a gain, **Reload config** → behavior changes and the filters reset cleanly. MANUAL still overrides with the on-screen buttons.

---

## Self-Review

- **Spec coverage:** spec §4 Phase 3 (classical washout + effects layer) and §2/§6 (`WashoutFilter`, `EffectsLayer` modules; tunable via TOML). Pragmatic-classical + core-two-effects scope per the confirmed decisions; engine/buffet are reserved config fields (documented, not wired).
- **Placeholder scan:** no TBD/TODO. The AUTO attitude placeholder is *removed* (replaced by real cueing), as promised since Phase 2. Reserved engine/buffet gains default 0 and produce no output — a stable schema hook, not dead behavior.
- **Type consistency:** `WashoutFilter`/`EffectsLayer` return `Pose` (same struct as the IK); `MotionProvider` sums them field-by-field. Config structs (`WashoutConfig`, `EffectsConfig`) are defined once and used by the filters, `MotionConfig`, and the tests with identical field names. `loadWashout`/`loadEffects` mirror `loadGeometry`'s contract and reuse its lenient `getDouble`.
- **Constraint check:** filter/config headers include no XPLM; `WashoutFilter.cpp`/`EffectsLayer.cpp` link into the native test targets with only `<cmath>`. All time dependence is via `dt`; rumble uses phase-advanced sine (deterministic), no RNG/clock — tests are reproducible. Sign handling is gain-based (tunable, incl. negative) rather than hard-coded.
- **Filter-behaviour tests:** the washout suite asserts the defining properties — rest→home, sustained translational accel washes out, sustained surge holds a steady tilt (coordination), sustained rate washes out, limits clamp, reset clears — rather than golden numbers, so they stay valid across reasonable tuning. Effects suite covers airborne-silence, touchdown fire+decay, and speed-gated rumble.
- **Integration safety:** MANUAL bypasses the filters (unchanged); switching to AUTO and reloading both reset filter state to avoid transients; the IK's reachability flag still guards oversized poses (hard safety limits are Phase 5).
