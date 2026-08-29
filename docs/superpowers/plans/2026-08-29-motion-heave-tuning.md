# Motion Heave Tuning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an offline measurement harness for the motion plugin's cueing chain, then use it to remove the heave channel's permanent saturation without adding perceptible lag.

**Architecture:** The plugin gains a CSV telemetry writer whose cue columns double as a replay input. A host-side CLI links the real filter sources (`WashoutFilter`, `EffectsLayer`, `StewartKinematics`, `SafetyLimiter`) so offline runs are bit-identical to what the plugin would have computed — that identity is enforced by a self-test. Parameter sweeps then run in seconds on the Mac, and only the finalists of each stage are flown on the rig.

**Tech Stack:** C++17, CMake, vendored `toml++`, the X-Plane SDK at `../XPlaneSDK` (plugin only — the harness is SDK-free), Python 3 with numpy for metrics.

**Spec:** `docs/superpowers/specs/2026-08-29-motion-heave-tuning-design.md`

## Global Constraints

- **This sandbox cannot build C++ or run Python.** Every build and run step is executed by the user, who reports the output back. Write the code, state the exact command, and stop at that step until results arrive. Never claim a build passed without the user's output.
- **Additive only in Part A.** Tasks 1–9 must not change any filter's numerical behaviour. The ten existing suites in `MotionProviderPlugin/tests/` staying green is the proof.
- **No behaviour change without its own stage.** One knob per change, counter-tested immediately. The single deliberate exception is Stage 3, which moves `heave_vel_washout_tau` and `heave_pos_washout_tau` together because they are the two poles of one washout.
- **Lag budget: `lag_ms` ≤ baseline + 15 ms.** A candidate breaking it is never flown.
- **Reference aircraft: Piper Arrow III (PA28-201R), vFlightAir custom add-on.** All baseline numbers are valid for this aircraft only.
- **Harness sources stay X-Plane-SDK-free.** `WashoutFilter`, `EffectsLayer`, `StewartKinematics`, `SafetyLimiter`, `MotionConfig` and the new `Telemetry` must remain host-linkable, or the replay tool and the tests cannot build.
- **Float formatting for round-trip exactness:** floats `%.9g`, doubles `%.17g`. The bit-exact self-test depends on this.
- Commit after every task. Branch: `feature/motion-heave-tuning`.

---

## File Structure

**Modified (Part A):**

| File | Responsibility after the change |
|---|---|
| `MotionProviderPlugin/src/WashoutFilter.h/.cpp` | unchanged maths, plus a `WashoutTrace` snapshot of internals |
| `MotionProviderPlugin/src/StewartKinematics.h/.cpp` | `clampToReachable` optionally reports its bisection scale |
| `MotionProviderPlugin/src/SafetyLimiter.h/.cpp` | counts channels limited on the last `limit()` call |
| `MotionProviderPlugin/src/MotionConfig.h/.cpp` | loads the new `[telemetry]` section |
| `MotionProviderPlugin/src/StatusData.h` | `UI_RECORD` action, recording fields in `StatusData` |
| `MotionProviderPlugin/src/StatusWindow.cpp` | draws the Record/Stop button |
| `MotionProviderPlugin/src/MotionProvider.h/.cpp` | owns `Telemetry`, fills one row per tick |
| `MotionProviderPlugin/CMakeLists.txt` | adds `Telemetry.cpp` to the plugin target |
| `MotionProviderPlugin/tests/CMakeLists.txt` | adds `test_telemetry` |

**Created (Part A):**

| File | Responsibility |
|---|---|
| `MotionProviderPlugin/src/TelemetryConfig.h` | `[telemetry]` settings struct |
| `MotionProviderPlugin/src/Telemetry.h/.cpp` | CSV writer: header, one row per tick, periodic flush |
| `MotionProviderPlugin/tests/test_telemetry.cpp` | header/row column-count agreement, formatting round-trip |
| `MotionProviderPlugin/tools/washout_replay.cpp` | replay CLI: read cues, apply overrides, run the chain, emit metrics input |
| `MotionProviderPlugin/tools/CMakeLists.txt` | host build for the replay tool |
| `MotionProviderPlugin/tools/washout_metrics.py` | metric table and plots from run CSVs |
| `docs/motion-tuning/README.md` | harness operating manual |
| `docs/motion-tuning/tuning-log.md` | living per-change log |
| `docs/motion-tuning/baseline-metrics.md` | frozen baseline numbers |

**Created (Part B, by recording):** `MotionProviderPlugin/reference/*.csv.gz`

---

# Part A — Build the harness

### Task 1: Washout trace

**Files:**
- Modify: `MotionProviderPlugin/src/WashoutFilter.h`
- Modify: `MotionProviderPlugin/src/WashoutFilter.cpp`
- Test: `MotionProviderPlugin/tests/test_washout.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces: `struct WashoutTrace` (fields below) and `const WashoutTrace& WashoutFilter::trace() const`. Tasks 5 and 7 read this; Task 9's telemetry row embeds it by value.

- [ ] **Step 1: Write the failing tests**

Append these blocks to `tests/test_washout.cpp`, immediately before the final `std::printf("\n%d checks...` line:

```cpp
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
    // Ratio of |G|max is ~15x between tau 2.0 and tau 0.3; assert a safe 4x.
    {
        auto peakRaw = [](double tau) {
            WashoutConfig cfg = WashoutConfig::defaults();
            cfg.heaveVelWashoutTau = tau;
            cfg.heavePosWashoutTau = tau;
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
              "tau 2.0 -> 0.3 cuts the raw heave excursion by more than 4x");
    }
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd MotionProviderPlugin && cmake -S tests -B tests/build && cmake --build tests/build
```
Expected: compile error — `WashoutFilter` has no member `trace`.

- [ ] **Step 3: Add the trace struct to the header**

In `src/WashoutFilter.h`, insert before `class WashoutFilter`:

```cpp
// Per-tick snapshot of the filter's internals. Written on every update();
// read by Telemetry and by the offline replay tool. Purely observational --
// nothing here feeds back into the filter.
struct WashoutTrace {
    double heaveAHp       = 0.0;   // high-passed vertical specific force, m/s^2
    double heaveVel       = 0.0;   // leaky velocity integrator, m/s
    double heavePosRaw    = 0.0;   // position BEFORE the +/-heaveLimitMm clamp, mm
    bool   heaveClamped   = false;
    double tiltPitch      = 0.0;   // deg, after rate limiting
    double tiltRoll       = 0.0;
    bool   tiltRateActive = false; // rate limiter engaged on either tilt axis
    double rotRollRaw     = 0.0;   // angles BEFORE the +/-rotLimitDeg clamp, deg
    double rotPitchRaw    = 0.0;
    double rotYawRaw      = 0.0;
    bool   rotRollClamped  = false;
    bool   rotPitchClamped = false;
    bool   rotYawClamped   = false;
};
```

Add to the public section, after `void setConfig(...)`:

```cpp
    const WashoutTrace& trace() const { return trace_; }
```

Add to the private section, after the smoothing state:

```cpp
    WashoutTrace trace_;
```

- [ ] **Step 4: Populate the trace in the implementation**

In `src/WashoutFilter.cpp`, change the `rateLimit` helper to report whether it engaged:

```cpp
double rateLimit(double cur, double tgt, double ratePerSec, double dt, bool& limited) {
    const double step = ratePerSec * dt;
    const double d = tgt - cur;
    if (d >  step) { limited = true; return cur + step; }
    if (d < -step) { limited = true; return cur - step; }
    return tgt;
}
```

In `reset()`, add:

```cpp
    trace_ = WashoutTrace{};
```

Replace the heave clamp line with:

```cpp
    trace_.heaveAHp    = aHp;
    trace_.heaveVel    = heaveVel_;
    trace_.heavePosRaw = heavePos_;
    const double heaveLimited = clampd(heavePos_, -cfg_.heaveLimitMm, cfg_.heaveLimitMm);
    trace_.heaveClamped = (heaveLimited != heavePos_);
    heavePos_ = heaveLimited;
```

Replace the two `rateLimit` calls with:

```cpp
    bool tiltLimited = false;
    tiltPitch_ = rateLimit(tiltPitch_, tgtPitch, cfg_.tiltRateLimitDps, dt, tiltLimited);
    tiltRoll_  = rateLimit(tiltRoll_,  tgtRoll,  cfg_.tiltRateLimitDps, dt, tiltLimited);
    trace_.tiltPitch      = tiltPitch_;
    trace_.tiltRoll       = tiltRoll_;
    trace_.tiltRateActive = tiltLimited;
```

Replace the `rotChan` lambda and its three call sites with:

```cpp
    auto rotChan = [&](double gain, double rate, double& rateLp, double& angle,
                       double& rawOut, bool& clampedOut) {
        const double w = gain * rate;                // deg/s
        rateLp += lpAlpha(dt, cfg_.rotHpTau) * (w - rateLp);
        const double wHp = w - rateLp;
        angle = (angle + wHp * dt) * leak(dt, cfg_.rotWashoutTau);
        rawOut = angle;
        const double limited = clampd(angle, -cfg_.rotLimitDeg, cfg_.rotLimitDeg);
        clampedOut = (limited != angle);
        angle = limited;
    };
    rotChan(cfg_.rotRollGain,  c.rollRate,  rollRateLp_,  rollAngle_,
            trace_.rotRollRaw,  trace_.rotRollClamped);
    rotChan(cfg_.rotPitchGain, c.pitchRate, pitchRateLp_, pitchAngle_,
            trace_.rotPitchRaw, trace_.rotPitchClamped);
    rotChan(cfg_.rotYawGain,   c.yawRate,   yawRateLp_,   yawAngle_,
            trace_.rotYawRaw,   trace_.rotYawClamped);
```

- [ ] **Step 5: Run the full test suite**

```bash
cd MotionProviderPlugin && cmake --build tests/build && ctest --test-dir tests/build --output-on-failure
```
Expected: all ten suites PASS, including the four new washout checks. **If `washout` fails on the saturation checks, stop — the analysis is wrong and the campaign must be re-cut before any further task.**

- [ ] **Step 6: Commit**

```bash
git add MotionProviderPlugin/src/WashoutFilter.h MotionProviderPlugin/src/WashoutFilter.cpp \
        MotionProviderPlugin/tests/test_washout.cpp
git commit -m "feat(MotionProviderPlugin): WashoutTrace for offline diagnosis"
```

---

### Task 2: Envelope scale and limiter clip counters

**Files:**
- Modify: `MotionProviderPlugin/src/StewartKinematics.h/.cpp`
- Modify: `MotionProviderPlugin/src/SafetyLimiter.h/.cpp`
- Test: `MotionProviderPlugin/tests/test_kinematics.cpp`, `MotionProviderPlugin/tests/test_safety.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces: `Pose StewartKinematics::clampToReachable(const Pose&, double* outScale = nullptr) const`; `int SafetyLimiter::velClipCount() const` and `int SafetyLimiter::accClipCount() const`. Task 9 reads all three.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_kinematics.cpp`, before its final summary printf:

```cpp
    // clampToReachable reports the uniform scale factor it applied.
    {
        double scale = -1.0;
        Pose home;                                  // all zero -> reachable
        K.clampToReachable(home, &scale);
        check(scale == 1.0, "reachable pose reports scale 1.0");

        Pose absurd; absurd.heave = 5000.0f;        // far outside the envelope
        scale = -1.0;
        K.clampToReachable(absurd, &scale);
        check(scale >= 0.0 && scale < 1.0, "unreachable pose reports a scale below 1");
    }
```

Append to `tests/test_safety.cpp`, before its final summary printf:

```cpp
    // Clip counters: a full-scale step saturates both limits on all six channels.
    {
        SafetyLimiter lim(SafetyConfig::defaults());
        uint16_t home[6]; for (int i = 0; i < 6; ++i) home[i] = 32640;
        lim.reset(home);
        uint16_t want[6]; for (int i = 0; i < 6; ++i) want[i] = 65280;
        uint16_t out[6];
        lim.limit(want, 1.0 / 60.0, out);
        check(lim.velClipCount() == 6, "full-scale step clips velocity on all 6");
        check(lim.accClipCount() == 6, "full-scale step clips acceleration on all 6");
    }

    // A held target clips nothing.
    {
        SafetyLimiter lim(SafetyConfig::defaults());
        uint16_t home[6]; for (int i = 0; i < 6; ++i) home[i] = 32640;
        lim.reset(home);
        uint16_t out[6];
        lim.limit(home, 1.0 / 60.0, out);
        check(lim.velClipCount() == 0 && lim.accClipCount() == 0,
              "held target clips nothing");
    }
```

- [ ] **Step 2: Run to verify they fail**

```bash
cd MotionProviderPlugin && cmake --build tests/build
```
Expected: compile errors — no `outScale` parameter, no `velClipCount`.

- [ ] **Step 3: Add the kinematics scale report**

In `src/StewartKinematics.h`, replace the `clampToReachable` declaration with:

```cpp
    // Uniformly scales the pose toward home until every leg is reachable.
    // `outScale`, if non-null, receives the factor applied (1.0 = the pose was
    // already reachable). Diagnostic only; the returned pose is unchanged by it.
    Pose clampToReachable(const Pose& pose, double* outScale = nullptr) const;
```

In `src/StewartKinematics.cpp`, change the definition's signature to match and add the two writes:

```cpp
Pose StewartKinematics::clampToReachable(const Pose& pose, double* outScale) const {
    if (solve(pose).allReachable) { if (outScale) *outScale = 1.0; return pose; }
```

and immediately before the final `return scaled(lo);`:

```cpp
    if (outScale) *outScale = lo;
```

- [ ] **Step 4: Add the limiter clip counters**

In `src/SafetyLimiter.h`, add to the public section:

```cpp
    // Channels limited on the most recent limit() call, 0..6. Diagnostic only.
    int velClipCount() const { return velClips_; }
    int accClipCount() const { return accClips_; }
```

and to the private section:

```cpp
    int velClips_ = 0;
    int accClips_ = 0;
```

In `src/SafetyLimiter.cpp`, add to `reset()`:

```cpp
    velClips_ = 0;
    accClips_ = 0;
```

and in `limit()`, replace the loop body's first three statements with:

```cpp
    velClips_ = 0;
    accClips_ = 0;

    for (int i = 0; i < 6; ++i) {
        const double target = static_cast<double>(desired[i]);
        const double rawVel = (target - pos_[i]) / dt;
        const double desiredVel = clampd(rawVel, -vMax, vMax);
        if (desiredVel != rawVel) ++velClips_;
        const double rawDv = desiredVel - vel_[i];
        const double dv = clampd(rawDv, -dvMax, dvMax);
        if (dv != rawDv) ++accClips_;
        vel_[i] += dv;
```

(the remaining lines of the loop — the position integration, clamp and `out[i]` write — are unchanged).

- [ ] **Step 5: Run the full suite**

```bash
cd MotionProviderPlugin && cmake --build tests/build && ctest --test-dir tests/build --output-on-failure
```
Expected: all ten suites PASS.

- [ ] **Step 6: Commit**

```bash
git add MotionProviderPlugin/src/StewartKinematics.h MotionProviderPlugin/src/StewartKinematics.cpp \
        MotionProviderPlugin/src/SafetyLimiter.h MotionProviderPlugin/src/SafetyLimiter.cpp \
        MotionProviderPlugin/tests/test_kinematics.cpp MotionProviderPlugin/tests/test_safety.cpp
git commit -m "feat(MotionProviderPlugin): report envelope scale and limiter clips"
```

---

### Task 3: Telemetry writer

**Files:**
- Create: `MotionProviderPlugin/src/TelemetryConfig.h`
- Create: `MotionProviderPlugin/src/Telemetry.h`, `MotionProviderPlugin/src/Telemetry.cpp`
- Create: `MotionProviderPlugin/tests/test_telemetry.cpp`
- Modify: `MotionProviderPlugin/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `MotionCues` (Task 0, existing), `WashoutTrace` (Task 1), `Pose` (existing).
- Produces: `struct TelemetryRow`; `class Telemetry` with `bool start(const std::string&)`, `void stop()`, `bool recording() const`, `const std::string& path() const`, `unsigned long long rows() const`, `void write(const TelemetryRow&)`, and `static const char* header()`. Task 4 wires these into `MotionProvider`; Task 5 parses the format this produces.

- [ ] **Step 1: Write the failing test**

Create `tests/test_telemetry.cpp`:

```cpp
#include "Telemetry.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }

static std::vector<std::string> split(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string f;
    while (std::getline(ss, f, ',')) out.push_back(f);
    return out;
}

int main() {
    const std::string path = "test_telemetry_out.csv";

    // Header and data rows must agree on column count, or every downstream
    // parser silently misreads the file.
    {
        Telemetry t;
        check(t.start(path), "start() opens the file");
        check(t.recording(), "recording() true after start");
        TelemetryRow r;
        r.t = 1.25; r.dtReal = 1.0/60.0; r.dtClamped = 1.0/60.0;
        r.cues.heaveG = 1.5f;
        r.trace.heavePosRaw = 123.456;
        r.trace.heaveClamped = true;
        r.live.heave = 30.0f;
        r.commanded.heave = 12.0f;
        r.reachScale = 0.75;
        for (int i = 0; i < 6; ++i) { r.setpoints[i] = 1000 + i; r.sent[i] = 2000 + i; }
        r.velClips = 3; r.accClips = 1; r.armState = 2;
        t.write(r);
        t.write(r);
        check(t.rows() == 2, "rows() counts written rows");
        t.stop();
        check(!t.recording(), "recording() false after stop");
    }
    {
        std::ifstream in(path);
        std::string headerLine, dataLine;
        std::getline(in, headerLine);
        std::getline(in, dataLine);
        const size_t hc = split(headerLine).size();
        const size_t dc = split(dataLine).size();
        check(hc == dc, "header and data column counts agree");
        check(hc > 40, "header has the full column set");
        check(headerLine.rfind("t_sec,", 0) == 0, "header starts with t_sec");
    }

    // Floats round-trip exactly at the documented precision -- the bit-exact
    // replay self-test depends on this.
    {
        Telemetry t;
        t.start(path);
        TelemetryRow r;
        r.cues.heaveG = 1.0f + 1.0f/3.0f;
        r.dtReal = 1.0/60.0;
        t.write(r);
        t.stop();
        std::ifstream in(path);
        std::string headerLine, dataLine;
        std::getline(in, headerLine);
        std::getline(in, dataLine);
        const std::vector<std::string> h = split(headerLine);
        const std::vector<std::string> d = split(dataLine);
        size_t gIdx = h.size(), dtIdx = h.size();
        for (size_t i = 0; i < h.size(); ++i) {
            if (h[i] == "g_nrml")  gIdx = i;
            if (h[i] == "dt_real") dtIdx = i;
        }
        check(gIdx < h.size() && dtIdx < h.size(), "g_nrml and dt_real columns exist");
        check(static_cast<float>(std::atof(d[gIdx].c_str())) == r.cues.heaveG,
              "float column round-trips exactly");
        check(std::atof(d[dtIdx].c_str()) == r.dtReal,
              "double column round-trips exactly");
    }

    std::remove(path.c_str());
    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
```

Add to `tests/CMakeLists.txt`, after the `test_armgate` block:

```cmake
add_executable(test_telemetry test_telemetry.cpp ../src/Telemetry.cpp ../src/WashoutFilter.cpp)
target_include_directories(test_telemetry PRIVATE ../src)
```

and to the test list:

```cmake
add_test(NAME telemetry COMMAND test_telemetry)
```

- [ ] **Step 2: Run to verify it fails**

```bash
cd MotionProviderPlugin && cmake -S tests -B tests/build && cmake --build tests/build
```
Expected: `Telemetry.h: No such file or directory`.

- [ ] **Step 3: Create the config struct**

Create `src/TelemetryConfig.h`:

```cpp
#pragma once
#include <string>

// [telemetry] section of configuration.toml. Recording is a diagnostic aid, so
// it defaults to off -- an unattended flight should never silently fill a disk.
struct TelemetryConfig {
    bool        enabled = false;   // start recording as soon as the plugin loads
    std::string dir     = "";      // output directory; empty = the plugin directory

    static TelemetryConfig defaults() { return TelemetryConfig{}; }
};
```

- [ ] **Step 4: Create the writer header**

Create `src/Telemetry.h`:

```cpp
#pragma once
#include <cstdint>
#include <fstream>
#include <string>
#include "MotionCues.h"
#include "Pose.h"
#include "WashoutFilter.h"   // WashoutTrace

// One recorded flight-loop tick. The cue fields alone are what the offline
// replay tool consumes; everything else is measurement output.
struct TelemetryRow {
    double       t          = 0.0;   // seconds since recording started
    double       dtReal     = 0.0;   // unclamped flight-loop timestep
    double       dtClamped  = 0.0;   // timestep the filters actually saw
    MotionCues   cues;
    WashoutTrace trace;
    Pose         effects;            // effects-layer contribution this tick
    Pose         live;               // washout + effects, BEFORE the arm blend
    Pose         commanded;          // pose fed to the IK, after blend and clamp
    double       reachScale = 1.0;   // clampToReachable bisection factor
    uint16_t     setpoints[6] = {0, 0, 0, 0, 0, 0};   // post-IK
    uint16_t     sent[6]      = {0, 0, 0, 0, 0, 0};   // post-SafetyLimiter
    int          velClips  = 0;
    int          accClips  = 0;
    int          armState  = 0;
};

// Buffered CSV writer. XPLM-free by design: the tests and the offline replay
// tool link this directly. Flushes about once a second so a sim crash costs at
// most a second of data without paying an fsync per tick.
class Telemetry {
public:
    ~Telemetry();

    bool start(const std::string& path);   // truncates, writes the header row
    void stop();
    bool recording() const { return out_.is_open(); }

    void write(const TelemetryRow& r);

    const std::string&  path() const { return path_; }
    unsigned long long  rows() const { return rows_; }

    // Column names, in the exact order write() emits values.
    static const char* header();

private:
    std::ofstream      out_;
    std::string        path_;
    unsigned long long rows_        = 0;
    double             lastFlushT_  = 0.0;
};
```

- [ ] **Step 5: Create the writer implementation**

Create `src/Telemetry.cpp`:

```cpp
#include "Telemetry.h"
#include <cstdio>

namespace {
// Round-trip-exact formatting. float carries 9 significant decimal digits,
// double 17. The replay self-test compares recomputed values against these,
// so anything shorter would fail it for the wrong reason.
void putF(std::ofstream& o, float v)  { char b[32]; std::snprintf(b, sizeof(b), "%.9g",  static_cast<double>(v)); o << ',' << b; }
void putD(std::ofstream& o, double v) { char b[40]; std::snprintf(b, sizeof(b), "%.17g", v); o << ',' << b; }
void putI(std::ofstream& o, long v)   { o << ',' << v; }
}  // namespace

Telemetry::~Telemetry() { stop(); }

const char* Telemetry::header() {
    return "t_sec,dt_real,dt_clamped,"
           "g_nrml,g_axil,g_side,P,Q,R,theta,phi,onground,gs,rpm,alpha,paused,"
           "heave_a_hp,heave_vel,heave_pos_raw,heave_clamped,"
           "tilt_pitch,tilt_roll,tilt_rate_active,"
           "rot_roll_raw,rot_pitch_raw,rot_yaw_raw,"
           "rot_roll_clamped,rot_pitch_clamped,rot_yaw_clamped,"
           "eff_heave,eff_roll,eff_pitch,eff_yaw,"
           "live_heave,live_roll,live_pitch,live_yaw,"
           "cmd_heave,cmd_roll,cmd_pitch,cmd_yaw,"
           "reach_scale,"
           "sp0,sp1,sp2,sp3,sp4,sp5,"
           "sent0,sent1,sent2,sent3,sent4,sent5,"
           "sl_vel_clip,sl_acc_clip,arm_state";
}

bool Telemetry::start(const std::string& path) {
    stop();
    out_.open(path, std::ios::out | std::ios::trunc);
    if (!out_.is_open()) return false;
    out_ << header() << '\n';
    path_       = path;
    rows_       = 0;
    lastFlushT_ = 0.0;
    return true;
}

void Telemetry::stop() {
    if (out_.is_open()) { out_.flush(); out_.close(); }
}

void Telemetry::write(const TelemetryRow& r) {
    if (!out_.is_open()) return;

    // First column has no leading comma; every put* adds one, so emit t_sec raw.
    char b[40];
    std::snprintf(b, sizeof(b), "%.17g", r.t);
    out_ << b;

    putD(out_, r.dtReal);        putD(out_, r.dtClamped);

    putF(out_, r.cues.heaveG);   putF(out_, r.cues.surgeG);   putF(out_, r.cues.swayG);
    putF(out_, r.cues.rollRate); putF(out_, r.cues.pitchRate); putF(out_, r.cues.yawRate);
    putF(out_, r.cues.pitchDeg); putF(out_, r.cues.rollDeg);
    putI(out_, r.cues.onGround ? 1 : 0);
    putF(out_, r.cues.groundspeed); putF(out_, r.cues.engineRpm); putF(out_, r.cues.alphaDeg);
    putI(out_, r.cues.simPaused ? 1 : 0);

    putD(out_, r.trace.heaveAHp); putD(out_, r.trace.heaveVel); putD(out_, r.trace.heavePosRaw);
    putI(out_, r.trace.heaveClamped ? 1 : 0);
    putD(out_, r.trace.tiltPitch); putD(out_, r.trace.tiltRoll);
    putI(out_, r.trace.tiltRateActive ? 1 : 0);
    putD(out_, r.trace.rotRollRaw); putD(out_, r.trace.rotPitchRaw); putD(out_, r.trace.rotYawRaw);
    putI(out_, r.trace.rotRollClamped ? 1 : 0);
    putI(out_, r.trace.rotPitchClamped ? 1 : 0);
    putI(out_, r.trace.rotYawClamped ? 1 : 0);

    putF(out_, r.effects.heave);   putF(out_, r.effects.roll);
    putF(out_, r.effects.pitch);   putF(out_, r.effects.yaw);
    putF(out_, r.live.heave);      putF(out_, r.live.roll);
    putF(out_, r.live.pitch);      putF(out_, r.live.yaw);
    putF(out_, r.commanded.heave); putF(out_, r.commanded.roll);
    putF(out_, r.commanded.pitch); putF(out_, r.commanded.yaw);

    putD(out_, r.reachScale);
    for (int i = 0; i < 6; ++i) putI(out_, r.setpoints[i]);
    for (int i = 0; i < 6; ++i) putI(out_, r.sent[i]);
    putI(out_, r.velClips); putI(out_, r.accClips); putI(out_, r.armState);

    out_ << '\n';
    ++rows_;

    if (r.t - lastFlushT_ >= 1.0) { out_.flush(); lastFlushT_ = r.t; }
}
```

- [ ] **Step 6: Run the tests**

```bash
cd MotionProviderPlugin && cmake -S tests -B tests/build && cmake --build tests/build && ctest --test-dir tests/build --output-on-failure
```
Expected: eleven suites PASS.

- [ ] **Step 7: Commit**

```bash
git add MotionProviderPlugin/src/Telemetry.h MotionProviderPlugin/src/Telemetry.cpp \
        MotionProviderPlugin/src/TelemetryConfig.h \
        MotionProviderPlugin/tests/test_telemetry.cpp MotionProviderPlugin/tests/CMakeLists.txt
git commit -m "feat(MotionProviderPlugin): CSV telemetry writer"
```

---

### Task 4: Load the `[telemetry]` config section

**Files:**
- Modify: `MotionProviderPlugin/src/MotionConfig.h`, `MotionProviderPlugin/src/MotionConfig.cpp`
- Test: `MotionProviderPlugin/tests/test_config.cpp`

**Interfaces:**
- Consumes: `TelemetryConfig` (Task 3).
- Produces: `TelemetryConfig MotionConfig::loadTelemetry(const std::string& path)`, and a `[telemetry]` block in `writeDefaults`. Task 5 calls `loadTelemetry`.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_config.cpp`, before its final summary printf:

```cpp
    // [telemetry] loads, and absent keys fall back to defaults.
    {
        const std::string p = "test_config_telemetry.toml";
        {
            std::ofstream f(p);
            f << "[telemetry]\n";
            f << "enabled = true\n";
            f << "dir = \"/tmp/motion\"\n";
        }
        TelemetryConfig t = MotionConfig::loadTelemetry(p);
        check(t.enabled, "telemetry.enabled parsed");
        check(t.dir == "/tmp/motion", "telemetry.dir parsed");
        std::remove(p.c_str());
    }
    {
        TelemetryConfig t = MotionConfig::loadTelemetry("does_not_exist.toml");
        check(!t.enabled && t.dir.empty(), "missing file -> telemetry defaults");
    }
```

If `test_config.cpp` does not already include them, add `#include <fstream>` and `#include <cstdio>` at its top.

- [ ] **Step 2: Run to verify it fails**

```bash
cd MotionProviderPlugin && cmake --build tests/build
```
Expected: `loadTelemetry` is not a member of `MotionConfig`.

- [ ] **Step 3: Declare the loader**

In `src/MotionConfig.h`, add `#include "TelemetryConfig.h"` to the include block and, inside the namespace:

```cpp
    // Load telemetry config from a TOML file. Same contract as loadWashout.
    TelemetryConfig loadTelemetry(const std::string& path);
```

- [ ] **Step 4: Implement it**

In `src/MotionConfig.cpp`, follow the shape of the existing `loadSerial`. Add:

```cpp
TelemetryConfig MotionConfig::loadTelemetry(const std::string& path) {
    TelemetryConfig cfg = TelemetryConfig::defaults();
    toml::table tbl;
    try { tbl = toml::parse_file(path); } catch (...) { return cfg; }
    if (auto* t = tbl["telemetry"].as_table()) {
        if (auto v = (*t)["enabled"].value<bool>())        cfg.enabled = *v;
        if (auto v = (*t)["dir"].value<std::string>())     cfg.dir     = *v;
    }
    return cfg;
}
```

Then add the section to `writeDefaults`, after the `[safety]` block, matching the existing emission style:

```cpp
    out << "\n[telemetry]\n";
    out << "enabled = false # set true to auto-start CSV recording on plugin load\n";
    out << "dir = \"\" # output directory; empty = the plugin directory\n";
```

- [ ] **Step 5: Run the tests**

```bash
cd MotionProviderPlugin && cmake --build tests/build && ctest --test-dir tests/build --output-on-failure
```
Expected: eleven suites PASS.

- [ ] **Step 6: Commit**

```bash
git add MotionProviderPlugin/src/MotionConfig.h MotionProviderPlugin/src/MotionConfig.cpp \
        MotionProviderPlugin/tests/test_config.cpp
git commit -m "feat(MotionProviderPlugin): [telemetry] config section"
```

---

### Task 5: Wire telemetry into the flight loop and the status window

**Files:**
- Modify: `MotionProviderPlugin/src/StatusData.h`
- Modify: `MotionProviderPlugin/src/StatusWindow.cpp`
- Modify: `MotionProviderPlugin/src/MotionProvider.h`, `MotionProviderPlugin/src/MotionProvider.cpp`
- Modify: `MotionProviderPlugin/CMakeLists.txt`

**Interfaces:**
- Consumes: `Telemetry`, `TelemetryRow` (Task 3); `loadTelemetry` (Task 4); `trace()` (Task 1); `clampToReachable(..., &scale)`, `velClipCount()`, `accClipCount()` (Task 2).
- Produces: recordings in the telemetry CSV format. Task 6 parses them.

This task has no host-side test — it is X-Plane-side wiring. Its verification is the first recording, in Step 7.

- [ ] **Step 1: Add the UI action and status fields**

In `src/StatusData.h`, add to the `UiAction` enum, after `UI_DISCONNECT`:

```cpp
    ,UI_RECORD        // toggle telemetry recording
```

and to `StatusData`:

```cpp
    bool               recording = false;
    unsigned long long telemetryRows = 0;
    std::string        telemetryPath;   // empty when not recording
```

- [ ] **Step 2: Draw the button**

In `src/StatusWindow.cpp`, find the row where `UI_RELOAD` / `UI_RESCAN_PORTS` buttons are drawn and add one alongside, following the existing `button(...)` call pattern:

```cpp
        x += button(x, y, data_.recording ? "Stop Rec" : "Record", UI_RECORD,
                    data_.recording ? 1.0f : 0.7f, 0.7f, 0.7f) + 8;
```

Below it, when recording, show progress next to the other status lines:

```cpp
        if (data_.recording) {
            drawString(x0, y, "REC " + std::to_string(data_.telemetryRows) + " rows -> " +
                       data_.telemetryPath, 1.0f, 0.5f, 0.5f);
        }
```

Use whatever local names the surrounding code already uses for the cursor (`x`, `y`, `x0`) rather than introducing new ones.

- [ ] **Step 3: Add the members to MotionProvider**

In `src/MotionProvider.h`, add the include and members:

```cpp
#include "Telemetry.h"
#include "TelemetryConfig.h"
```

```cpp
    std::unique_ptr<Telemetry> telemetry_;
    TelemetryConfig            telemetryCfg_;
    double                     telemetryT_ = 0.0;
    Pose                       lastEffectsPose_;
    // mutable: written by blendedCommand(), which is const.
    mutable double             lastReachScale_ = 1.0;

    std::string telemetryFilePath() const;   // dir + timestamped filename
    void        toggleRecording();
```

- [ ] **Step 4: Implement start/stop and the path helper**

In `src/MotionProvider.cpp`, add near the other helpers:

```cpp
std::string MotionProvider::telemetryFilePath() const {
    // Directory from config, else the plugin directory (same place as the TOML).
    std::string dir = telemetryCfg_.dir;
    if (dir.empty()) {
        const size_t f = configPath_.find_last_of("/\\");
        dir = (f == std::string::npos) ? "." : configPath_.substr(0, f);
    }
    const std::time_t now = std::time(nullptr);
    char stamp[32] = {0};
    std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", std::localtime(&now));
    return dir + "/motion-" + stamp + ".csv";
}

void MotionProvider::toggleRecording() {
    if (!telemetry_) telemetry_ = std::make_unique<Telemetry>();
    if (telemetry_->recording()) {
        telemetry_->stop();
        XPLMDebugString("MotionProvider: telemetry stopped\n");
    } else {
        const std::string p = telemetryFilePath();
        telemetryT_ = 0.0;
        const bool ok = telemetry_->start(p);
        XPLMDebugString(("MotionProvider: telemetry " + p +
                         (ok ? " recording\n" : " OPEN FAILED\n")).c_str());
    }
}
```

Add `#include <ctime>` to the file's includes.

In `initialize()`, after the safety config is loaded, add:

```cpp
    telemetryCfg_ = MotionConfig::loadTelemetry(configPath_);
    telemetry_ = std::make_unique<Telemetry>();
    if (telemetryCfg_.enabled) toggleRecording();
```

In `reloadConfig()`, after the safety reload, add (deliberately not restarting an active recording — a config reload mid-recording must not split the file):

```cpp
    telemetryCfg_ = MotionConfig::loadTelemetry(path);
```

In `shutdown()`, before `serial_.reset();`:

```cpp
    if (telemetry_) telemetry_->stop();
    telemetry_.reset();
```

In `onUiAction`, add the case:

```cpp
        case UI_RECORD:      toggleRecording(); break;
```

In `pushStatus()`, add to the snapshot it builds:

```cpp
    d.recording     = telemetry_ && telemetry_->recording();
    d.telemetryRows = telemetry_ ? telemetry_->rows() : 0;
    d.telemetryPath = telemetry_ ? telemetry_->path() : std::string();
```

(use whatever local name `pushStatus` already gives its `StatusData`).

- [ ] **Step 5: Capture the effects pose and the envelope scale**

In `onFlightLoopTick`, inside the `if (!latestCues_.simPaused)` block, replace the effects line so the contribution is kept:

```cpp
            Pose w = washout_->update(latestCues_, dt);
            Pose e = effects_->update(latestCues_, dt);
            lastEffectsPose_ = e;
```

In `blendedCommand`, capture the scale of the final clamp — replace the final return with:

```cpp
    double scale = 1.0;
    const Pose out = kin_->clampToReachable(eff, &scale);
    lastReachScale_ = scale;
    return out;
```

- [ ] **Step 6: Write the row**

At the end of `onFlightLoopTick`, immediately before the `statusAccumSec_` block:

```cpp
    if (telemetry_ && telemetry_->recording()) {
        telemetryT_ += static_cast<double>(elapsedSec);
        TelemetryRow row;
        row.t         = telemetryT_;
        row.dtReal    = static_cast<double>(elapsedSec);
        row.dtClamped = dt;
        row.cues      = latestCues_;
        if (washout_) row.trace = washout_->trace();
        row.effects    = lastEffectsPose_;
        row.live       = lastLivePose_;
        row.commanded  = latestPose_;
        row.reachScale = lastReachScale_;
        for (int i = 0; i < 6; ++i) {
            row.setpoints[i] = latestSolve_.setpoints[i];
            row.sent[i]      = sentSetpoints_[i];
        }
        row.velClips = safety_ ? safety_->velClipCount() : 0;
        row.accClips = safety_ ? safety_->accClipCount() : 0;
        row.armState = static_cast<int>(armRamp_.state());
        telemetry_->write(row);
    }
```

Add `Telemetry.cpp` to the plugin's source list in `CMakeLists.txt` alongside the other `src/*.cpp` entries.

- [ ] **Step 7: Build and make the first recording**

```bash
cd MotionProviderPlugin && ./build-macos.sh
```

Then in X-Plane: load the Arrow III, open the Motion Provider status window, press **Record**, fly for ~60 s, press **Stop Rec**.

- [ ] **Step 8: Check for frame hitches**

The spec requires measuring whether the 60 Hz write disturbs the sim. Inspect the `dt_real` column:

```bash
awk -F, 'NR>1 {print $2}' motion-*.csv | sort -n | tail -5
```
Expected: the largest `dt_real` values are ordinary frame-time variation, not isolated spikes far above the rest. **If recording introduces spikes, stop and move the write onto the serial I/O thread before continuing** — a harness that perturbs what it measures is worthless.

- [ ] **Step 9: Commit**

```bash
git add MotionProviderPlugin/src/StatusData.h MotionProviderPlugin/src/StatusWindow.cpp \
        MotionProviderPlugin/src/MotionProvider.h MotionProviderPlugin/src/MotionProvider.cpp \
        MotionProviderPlugin/CMakeLists.txt
git commit -m "feat(MotionProviderPlugin): record telemetry from the flight loop"
```

---

### Task 6: Replay CLI — read, override, run, write

**Files:**
- Create: `MotionProviderPlugin/tools/washout_replay.cpp`
- Create: `MotionProviderPlugin/tools/CMakeLists.txt`

**Interfaces:**
- Consumes: the telemetry CSV format (Task 3); `MotionConfig::load*` (existing + Task 4); `WashoutFilter`, `EffectsLayer`, `StewartKinematics`, `SafetyLimiter`.
- Produces: the `washout_replay` binary with `--cues`, `--config`, `--set`, `--out`. Tasks 7 and 8 extend it; Task 9's script consumes its output.

- [ ] **Step 1: Write the CMake target**

Create `tools/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.15)
project(washout_replay CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Links the real filter sources -- there is deliberately no second
# implementation of the cueing chain that could drift from the plugin's.
add_executable(washout_replay
    washout_replay.cpp
    ../src/WashoutFilter.cpp
    ../src/EffectsLayer.cpp
    ../src/StewartKinematics.cpp
    ../src/SafetyLimiter.cpp
    ../src/MotionConfig.cpp
    ../src/Telemetry.cpp)
target_include_directories(washout_replay PRIVATE ../src ../third_party/tomlplusplus)
```

- [ ] **Step 2: Write the tool**

Create `tools/washout_replay.cpp`:

```cpp
// Offline replay of the motion cueing chain.
//
// Reads a telemetry CSV (only its dt and cue columns matter), runs the same
// WashoutFilter/EffectsLayer/StewartKinematics/SafetyLimiter the plugin runs,
// and writes a telemetry CSV in the identical format. Because the chain is a
// pure function of (cues, dt, config), a replay reproduces exactly what the
// plugin would have computed -- see --verify.
#include "EffectsLayer.h"
#include "MotionConfig.h"
#include "SafetyLimiter.h"
#include "StewartKinematics.h"
#include "Telemetry.h"
#include "WashoutFilter.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct CueSample {
    double     dt = 1.0 / 60.0;
    MotionCues cues;
    // Recorded outputs, kept for --verify.
    float recLiveHeave = 0.0f, recLiveRoll = 0.0f, recLivePitch = 0.0f, recLiveYaw = 0.0f;
    bool  haveRecorded = false;
};

std::vector<std::string> splitLine(const std::string& s) {
    std::vector<std::string> out;
    std::string field;
    std::stringstream ss(s);
    while (std::getline(ss, field, ',')) out.push_back(field);
    return out;
}

// Column name -> index, from the header row.
std::map<std::string, size_t> headerIndex(const std::string& headerLine) {
    std::map<std::string, size_t> idx;
    const std::vector<std::string> h = splitLine(headerLine);
    for (size_t i = 0; i < h.size(); ++i) idx[h[i]] = i;
    return idx;
}

double col(const std::vector<std::string>& f, const std::map<std::string, size_t>& idx,
           const char* name, double fallback = 0.0) {
    const auto it = idx.find(name);
    if (it == idx.end() || it->second >= f.size()) return fallback;
    return std::atof(f[it->second].c_str());
}

bool loadCues(const std::string& path, std::vector<CueSample>& out, std::string& err) {
    std::ifstream in(path);
    if (!in.is_open()) { err = "cannot open " + path; return false; }
    std::string headerLine;
    if (!std::getline(in, headerLine)) { err = "empty file " + path; return false; }
    const std::map<std::string, size_t> idx = headerIndex(headerLine);
    if (idx.find("g_nrml") == idx.end() || idx.find("dt_real") == idx.end()) {
        err = "missing g_nrml/dt_real columns in " + path;
        return false;
    }
    const bool haveRec = idx.find("live_heave") != idx.end();

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        const std::vector<std::string> f = splitLine(line);
        CueSample s;
        s.dt                = col(f, idx, "dt_real", 1.0 / 60.0);
        s.cues.heaveG       = static_cast<float>(col(f, idx, "g_nrml", 1.0));
        s.cues.surgeG       = static_cast<float>(col(f, idx, "g_axil"));
        s.cues.swayG        = static_cast<float>(col(f, idx, "g_side"));
        s.cues.rollRate     = static_cast<float>(col(f, idx, "P"));
        s.cues.pitchRate    = static_cast<float>(col(f, idx, "Q"));
        s.cues.yawRate      = static_cast<float>(col(f, idx, "R"));
        s.cues.pitchDeg     = static_cast<float>(col(f, idx, "theta"));
        s.cues.rollDeg      = static_cast<float>(col(f, idx, "phi"));
        s.cues.onGround     = col(f, idx, "onground") != 0.0;
        s.cues.groundspeed  = static_cast<float>(col(f, idx, "gs"));
        s.cues.engineRpm    = static_cast<float>(col(f, idx, "rpm"));
        s.cues.alphaDeg     = static_cast<float>(col(f, idx, "alpha"));
        s.cues.simPaused    = col(f, idx, "paused") != 0.0;
        if (haveRec) {
            s.recLiveHeave = static_cast<float>(col(f, idx, "live_heave"));
            s.recLiveRoll  = static_cast<float>(col(f, idx, "live_roll"));
            s.recLivePitch = static_cast<float>(col(f, idx, "live_pitch"));
            s.recLiveYaw   = static_cast<float>(col(f, idx, "live_yaw"));
            s.haveRecorded = true;
        }
        out.push_back(s);
    }
    return true;
}

// --set / --sweep address config fields by "section.key". Pointer-to-member
// tables keep the mapping explicit and complete rather than reflective.
struct WashoutKey { const char* key; double WashoutConfig::* field; };
struct SafetyKey  { const char* key; double SafetyConfig::*  field; };
struct EffectsKey { const char* key; double EffectsConfig::* field; };

const WashoutKey kWashoutKeys[] = {
    {"washout.heave_gain",              &WashoutConfig::heaveGain},
    {"washout.heave_hp_tau",            &WashoutConfig::heaveHpTau},
    {"washout.heave_vel_washout_tau",   &WashoutConfig::heaveVelWashoutTau},
    {"washout.heave_pos_washout_tau",   &WashoutConfig::heavePosWashoutTau},
    {"washout.heave_limit_mm",          &WashoutConfig::heaveLimitMm},
    {"washout.tilt_surge_gain",         &WashoutConfig::tiltSurgeGain},
    {"washout.tilt_sway_gain",          &WashoutConfig::tiltSwayGain},
    {"washout.tilt_lp_tau",             &WashoutConfig::tiltLpTau},
    {"washout.tilt_limit_deg",          &WashoutConfig::tiltLimitDeg},
    {"washout.tilt_rate_limit_dps",     &WashoutConfig::tiltRateLimitDps},
    {"washout.rot_roll_gain",           &WashoutConfig::rotRollGain},
    {"washout.rot_pitch_gain",          &WashoutConfig::rotPitchGain},
    {"washout.rot_yaw_gain",            &WashoutConfig::rotYawGain},
    {"washout.rot_hp_tau",              &WashoutConfig::rotHpTau},
    {"washout.rot_washout_tau",         &WashoutConfig::rotWashoutTau},
    {"washout.rot_limit_deg",           &WashoutConfig::rotLimitDeg},
    {"washout.smooth_tau",              &WashoutConfig::smoothTau},
};
const SafetyKey kSafetyKeys[] = {
    {"safety.max_velocity_cps",      &SafetyConfig::maxVelocity},
    {"safety.max_acceleration_cps2", &SafetyConfig::maxAcceleration},
};
const EffectsKey kEffectsKeys[] = {
    {"effects.touchdown_gain",       &EffectsConfig::touchdownGain},
    {"effects.touchdown_freq_hz",    &EffectsConfig::touchdownFreqHz},
    {"effects.touchdown_decay_tau",  &EffectsConfig::touchdownDecayTau},
    {"effects.rumble_gain",          &EffectsConfig::rumbleGain},
    {"effects.rumble_freq_hz",       &EffectsConfig::rumbleFreqHz},
    {"effects.rumble_speed_ref_mps", &EffectsConfig::rumbleSpeedRefMps},
};

bool applyOverride(const std::string& key, double value,
                   WashoutConfig& w, SafetyConfig& s, EffectsConfig& e) {
    for (const WashoutKey& k : kWashoutKeys) if (key == k.key) { w.*(k.field) = value; return true; }
    for (const SafetyKey&  k : kSafetyKeys)  if (key == k.key) { s.*(k.field) = value; return true; }
    for (const EffectsKey& k : kEffectsKeys) if (key == k.key) { e.*(k.field) = value; return true; }
    return false;
}

void usage() {
    std::printf(
        "usage: washout_replay --cues FILE --config FILE [options]\n"
        "  --set section.key=VALUE   override a config value (repeatable)\n"
        "  --out FILE                write the replayed run as a telemetry CSV\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::string cuesPath, configPath, outPath;
    std::vector<std::pair<std::string, double>> overrides;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* what) -> std::string {
            if (i + 1 >= argc) { std::fprintf(stderr, "%s needs a value\n", what); std::exit(2); }
            return argv[++i];
        };
        if      (a == "--cues")   cuesPath   = next("--cues");
        else if (a == "--config") configPath = next("--config");
        else if (a == "--out")    outPath    = next("--out");
        else if (a == "--set") {
            const std::string kv = next("--set");
            const size_t eq = kv.find('=');
            if (eq == std::string::npos) { std::fprintf(stderr, "--set wants key=value\n"); return 2; }
            overrides.emplace_back(kv.substr(0, eq), std::atof(kv.c_str() + eq + 1));
        } else { usage(); return 2; }
    }
    if (cuesPath.empty() || configPath.empty()) { usage(); return 2; }

    std::vector<CueSample> samples;
    std::string err;
    if (!loadCues(cuesPath, samples, err)) { std::fprintf(stderr, "%s\n", err.c_str()); return 1; }

    StewartGeometry geo = MotionConfig::loadGeometry(configPath);
    WashoutConfig   wcfg = MotionConfig::loadWashout(configPath);
    EffectsConfig   ecfg = MotionConfig::loadEffects(configPath);
    SafetyConfig    scfg = MotionConfig::loadSafety(configPath);

    for (const auto& o : overrides) {
        if (!applyOverride(o.first, o.second, wcfg, scfg, ecfg)) {
            std::fprintf(stderr, "unknown key: %s\n", o.first.c_str());
            return 2;
        }
    }

    WashoutFilter      washout(wcfg);
    EffectsLayer       effects(ecfg);
    StewartKinematics  kin(geo);
    SafetyLimiter      safety(scfg);

    // Replay is always "armed and live": the arm blend is a transition, not part
    // of the cueing chain under test.
    uint16_t home[6];
    { const SolveResult s = kin.solve(Pose{}); for (int i = 0; i < 6; ++i) home[i] = s.setpoints[i]; }
    safety.reset(home);

    Telemetry out;
    if (!outPath.empty() && !out.start(outPath)) {
        std::fprintf(stderr, "cannot write %s\n", outPath.c_str());
        return 1;
    }

    double t = 0.0;
    for (const CueSample& s : samples) {
        double dt = s.dt;
        if (dt > scfg.maxDtSec) dt = scfg.maxDtSec;

        Pose live;
        if (!s.cues.simPaused) {
            const Pose w = washout.update(s.cues, dt);
            const Pose e = effects.update(s.cues, dt);
            live.heave = w.heave + e.heave;  live.roll  = w.roll  + e.roll;
            live.pitch = w.pitch + e.pitch;  live.yaw   = w.yaw   + e.yaw;

            double scale = 1.0;
            const Pose cmd = kin.clampToReachable(live, &scale);
            const SolveResult sol = kin.solve(cmd);
            uint16_t target[6], sent[6];
            for (int i = 0; i < 6; ++i) target[i] = sol.setpoints[i];
            safety.limit(target, dt, sent);

            if (out.recording()) {
                TelemetryRow r;
                r.t = t; r.dtReal = s.dt; r.dtClamped = dt;
                r.cues = s.cues; r.trace = washout.trace();
                r.effects = e; r.live = live; r.commanded = cmd;
                r.reachScale = scale;
                for (int i = 0; i < 6; ++i) { r.setpoints[i] = target[i]; r.sent[i] = sent[i]; }
                r.velClips = safety.velClipCount();
                r.accClips = safety.accClipCount();
                r.armState = 2;   // Armed
                out.write(r);
            }
        }
        t += s.dt;
    }
    out.stop();
    std::printf("replayed %zu samples (%.1f s)\n", samples.size(), t);
    return 0;
}
```

- [ ] **Step 3: Build it**

```bash
cd MotionProviderPlugin && cmake -S tools -B tools/build && cmake --build tools/build
```
Expected: `tools/build/washout_replay` exists.

- [ ] **Step 4: Run it against the Task 5 recording**

```bash
cd MotionProviderPlugin && ./tools/build/washout_replay \
    --cues motion-<stamp>.csv --config configuration.toml --out /tmp/replay.csv
```
Expected: `replayed N samples (T s)` with N matching the recording's row count.

- [ ] **Step 5: Commit**

```bash
git add MotionProviderPlugin/tools/washout_replay.cpp MotionProviderPlugin/tools/CMakeLists.txt
git commit -m "feat(MotionProviderPlugin): offline washout replay CLI"
```

---

### Task 7: The harness self-test, sweeps and dt resampling

**Files:**
- Modify: `MotionProviderPlugin/tools/washout_replay.cpp`

**Interfaces:**
- Consumes: Task 6's tool.
- Produces: `--verify`, `--sweep key=v1,v2,...`, `--resample-dt SEC`. Stage 2 onward depends on `--sweep`.

`--verify` is the gate the spec calls blocking: a replay must reproduce the recording's `live_*` columns exactly. `live_*` is chosen deliberately — it is the pure function of (cues, dt, config), while `cmd_*` also depends on the arm blend, which replay does not model.

- [ ] **Step 1: Add the three flags to the argument parser**

```cpp
        else if (a == "--verify")      doVerify = true;
        else if (a == "--resample-dt") resampleDt = std::atof(next("--resample-dt").c_str());
        else if (a == "--sweep") {
            const std::string kv = next("--sweep");
            const size_t eq = kv.find('=');
            if (eq == std::string::npos) { std::fprintf(stderr, "--sweep wants key=v1,v2\n"); return 2; }
            sweepKey = kv.substr(0, eq);
            std::stringstream ss(kv.substr(eq + 1));
            std::string v;
            while (std::getline(ss, v, ',')) sweepValues.push_back(std::atof(v.c_str()));
        }
```

with the declarations alongside the other locals in `main`:

```cpp
    bool                doVerify = false;
    double              resampleDt = 0.0;
    std::string         sweepKey;
    std::vector<double> sweepValues;
```

and the usage text extended:

```cpp
        "  --verify                  check the replay reproduces the recording's live_* columns\n"
        "  --sweep section.key=A,B,C run once per value, printing a summary table\n"
        "  --resample-dt SEC         re-run at a fixed timestep instead of the recorded one\n"
```

- [ ] **Step 2: Extract the run into a function**

Refactor the per-sample loop in `main` into a reusable function placed above `main`:

```cpp
struct RunResult {
    size_t samples = 0;
    double durationSec = 0.0;
    double maxLiveErr = 0.0;   // max |replayed - recorded| over the live_* columns
    double satHeavePct = 0.0;  // % of ticks with the heave clamp engaged
    double peakHeaveRawMm = 0.0;
};

RunResult runChain(const std::vector<CueSample>& samples,
                   const StewartGeometry& geo, const WashoutConfig& wcfg,
                   const EffectsConfig& ecfg, const SafetyConfig& scfg,
                   double resampleDt, Telemetry* out) {
    WashoutFilter     washout(wcfg);
    EffectsLayer      effects(ecfg);
    StewartKinematics kin(geo);
    SafetyLimiter     safety(scfg);

    uint16_t home[6];
    { const SolveResult s = kin.solve(Pose{}); for (int i = 0; i < 6; ++i) home[i] = s.setpoints[i]; }
    safety.reset(home);

    RunResult res;
    double t = 0.0;
    size_t clamped = 0, counted = 0;

    for (const CueSample& s : samples) {
        const double dtRaw = (resampleDt > 0.0) ? resampleDt : s.dt;
        double dt = dtRaw;
        if (dt > scfg.maxDtSec) dt = scfg.maxDtSec;
        if (s.cues.simPaused) { t += dtRaw; continue; }

        const Pose w = washout.update(s.cues, dt);
        const Pose e = effects.update(s.cues, dt);
        Pose live;
        live.heave = w.heave + e.heave;  live.roll  = w.roll  + e.roll;
        live.pitch = w.pitch + e.pitch;  live.yaw   = w.yaw   + e.yaw;

        double scale = 1.0;
        const Pose cmd = kin.clampToReachable(live, &scale);
        const SolveResult sol = kin.solve(cmd);
        uint16_t target[6], sent[6];
        for (int i = 0; i < 6; ++i) target[i] = sol.setpoints[i];
        safety.limit(target, dt, sent);

        const WashoutTrace& tr = washout.trace();
        ++counted;
        if (tr.heaveClamped) ++clamped;
        const double raw = std::fabs(tr.heavePosRaw);
        if (raw > res.peakHeaveRawMm) res.peakHeaveRawMm = raw;

        if (s.haveRecorded) {
            const double d[4] = {
                std::fabs(static_cast<double>(live.heave) - s.recLiveHeave),
                std::fabs(static_cast<double>(live.roll)  - s.recLiveRoll),
                std::fabs(static_cast<double>(live.pitch) - s.recLivePitch),
                std::fabs(static_cast<double>(live.yaw)   - s.recLiveYaw)};
            for (double v : d) if (v > res.maxLiveErr) res.maxLiveErr = v;
        }

        if (out && out->recording()) {
            TelemetryRow r;
            r.t = t; r.dtReal = dtRaw; r.dtClamped = dt;
            r.cues = s.cues; r.trace = tr;
            r.effects = e; r.live = live; r.commanded = cmd;
            r.reachScale = scale;
            for (int i = 0; i < 6; ++i) { r.setpoints[i] = target[i]; r.sent[i] = sent[i]; }
            r.velClips = safety.velClipCount();
            r.accClips = safety.accClipCount();
            r.armState = 2;
            out->write(r);
        }
        t += dtRaw;
    }
    res.samples     = counted;
    res.durationSec = t;
    res.satHeavePct = counted ? 100.0 * static_cast<double>(clamped) / static_cast<double>(counted) : 0.0;
    return res;
}
```

Replace the old loop in `main` with a call to it.

- [ ] **Step 3: Implement --verify and --sweep in main**

After the config is loaded and overrides applied:

```cpp
    if (!sweepKey.empty()) {
        std::printf("%-28s %10s %10s %14s\n", sweepKey.c_str(),
                    "sat_heave%", "peak_raw_mm", "samples");
        for (double v : sweepValues) {
            WashoutConfig w = wcfg; SafetyConfig s = scfg; EffectsConfig e = ecfg;
            if (!applyOverride(sweepKey, v, w, s, e)) {
                std::fprintf(stderr, "unknown key: %s\n", sweepKey.c_str());
                return 2;
            }
            Telemetry sweepOut;
            if (!outPath.empty()) {
                char name[512];
                std::snprintf(name, sizeof(name), "%s.%g.csv", outPath.c_str(), v);
                sweepOut.start(name);
            }
            const RunResult r = runChain(samples, geo, w, e, s, resampleDt,
                                         outPath.empty() ? nullptr : &sweepOut);
            sweepOut.stop();
            std::printf("%-28g %10.2f %10.1f %14zu\n", v, r.satHeavePct,
                        r.peakHeaveRawMm, r.samples);
        }
        return 0;
    }

    Telemetry out;
    if (!outPath.empty() && !out.start(outPath)) {
        std::fprintf(stderr, "cannot write %s\n", outPath.c_str());
        return 1;
    }
    const RunResult r = runChain(samples, geo, wcfg, ecfg, scfg, resampleDt,
                                 outPath.empty() ? nullptr : &out);
    out.stop();
    std::printf("replayed %zu samples (%.1f s), heave saturated %.2f%% of ticks, "
                "peak raw heave %.1f mm\n",
                r.samples, r.durationSec, r.satHeavePct, r.peakHeaveRawMm);

    if (doVerify) {
        if (overrides.empty() && resampleDt == 0.0) {
            std::printf("verify: max |replay - recorded| over live_* = %.9g mm/deg\n", r.maxLiveErr);
            if (r.maxLiveErr != 0.0) {
                std::fprintf(stderr, "VERIFY FAILED: replay does not reproduce the recording\n");
                return 1;
            }
            std::printf("verify: PASS (bit-exact)\n");
        } else {
            std::fprintf(stderr,
                "VERIFY REFUSED: --verify requires no --set and no --resample-dt\n");
            return 2;
        }
    }
    return 0;
}
```

- [ ] **Step 4: Build**

```bash
cd MotionProviderPlugin && cmake --build tools/build
```

- [ ] **Step 5: Run the self-test — this is the blocking gate**

```bash
cd MotionProviderPlugin && ./tools/build/washout_replay \
    --cues motion-<stamp>.csv --config configuration.toml --verify
```
Expected: `verify: PASS (bit-exact)`.

**If this fails, stop.** Every number produced later would be measuring the replay tool rather than the plugin. Likely causes to check in order: the recording was made with a different `configuration.toml` than the one passed; the recording spans a config reload; float formatting precision; the recording includes disarmed or blending ticks whose `live_*` came from a held pose.

- [ ] **Step 6: Smoke-test the sweep**

```bash
cd MotionProviderPlugin && ./tools/build/washout_replay \
    --cues motion-<stamp>.csv --config configuration.toml \
    --sweep washout.heave_pos_washout_tau=0.3,0.5,1.0,2.0
```
Expected: a four-row table with `sat_heave%` falling as tau falls.

- [ ] **Step 7: Commit**

```bash
git add MotionProviderPlugin/tools/washout_replay.cpp
git commit -m "feat(MotionProviderPlugin): replay verify, sweep and dt resampling"
```

---

### Task 8: Synthetic cue generator

**Files:**
- Modify: `MotionProviderPlugin/tools/washout_replay.cpp`

**Interfaces:**
- Consumes: Task 7's tool.
- Produces: `--synth SPEC` writing a cues-only CSV. Stage 1 depends on it.

- [ ] **Step 1: Add the generator**

Above `main`, add:

```cpp
// Synthetic cue streams, so the filter's response can be characterised with no
// flight at all. SPEC forms:
//   step:<g>:<durSec>            constant g_nrml offset after 1 s
//   sine:<hz>:<g>:<durSec>       sinusoidal g_nrml
//   chirp:<f0>-<f1>:<g>:<durSec> logarithmic sweep, for the frequency response
bool synthCues(const std::string& spec, double dt, std::vector<CueSample>& out,
               std::string& err) {
    std::vector<std::string> p;
    { std::stringstream ss(spec); std::string f;
      while (std::getline(ss, f, ':')) p.push_back(f); }
    if (p.empty()) { err = "empty --synth spec"; return false; }

    const std::string kind = p[0];
    auto sample = [&](double gOffset) {
        CueSample s;
        s.dt = dt;
        s.cues.heaveG = static_cast<float>(1.0 + gOffset);
        return s;
    };

    if (kind == "step" && p.size() >= 3) {
        const double g = std::atof(p[1].c_str()), dur = std::atof(p[2].c_str());
        for (double t = 0.0; t < dur; t += dt) out.push_back(sample(t < 1.0 ? 0.0 : g));
        return true;
    }
    if (kind == "sine" && p.size() >= 4) {
        const double hz = std::atof(p[1].c_str()), g = std::atof(p[2].c_str());
        const double dur = std::atof(p[3].c_str());
        for (double t = 0.0; t < dur; t += dt)
            out.push_back(sample(g * std::sin(2.0 * M_PI * hz * t)));
        return true;
    }
    if (kind == "chirp" && p.size() >= 4) {
        const size_t dash = p[1].find('-');
        if (dash == std::string::npos) { err = "chirp wants f0-f1"; return false; }
        const double f0 = std::atof(p[1].substr(0, dash).c_str());
        const double f1 = std::atof(p[1].c_str() + dash + 1);
        const double g = std::atof(p[2].c_str()), dur = std::atof(p[3].c_str());
        if (f0 <= 0.0 || f1 <= 0.0 || dur <= 0.0) { err = "chirp needs positive f0,f1,dur"; return false; }
        // Logarithmic sweep: phase is the integral of the instantaneous frequency.
        const double k = std::log(f1 / f0) / dur;
        for (double t = 0.0; t < dur; t += dt) {
            const double phase = 2.0 * M_PI * f0 * (std::exp(k * t) - 1.0) / k;
            out.push_back(sample(g * std::sin(phase)));
        }
        return true;
    }
    err = "unrecognised --synth spec: " + spec;
    return false;
}
```

Add `#include <cmath>` if not already present, and define `M_PI` defensively at the top of the anonymous namespace for MSVC:

```cpp
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
```

- [ ] **Step 2: Wire it into the argument parser**

Add the local and the flag:

```cpp
    std::string synthSpec;
    double      synthDt = 1.0 / 60.0;
```

```cpp
        else if (a == "--synth")    synthSpec = next("--synth");
        else if (a == "--synth-dt") synthDt   = std::atof(next("--synth-dt").c_str());
```

Replace the `--cues`/`--config` requirement check and the load with:

```cpp
    if (configPath.empty() || (cuesPath.empty() && synthSpec.empty())) { usage(); return 2; }

    std::vector<CueSample> samples;
    std::string err;
    if (!synthSpec.empty()) {
        if (!synthCues(synthSpec, synthDt, samples, err)) {
            std::fprintf(stderr, "%s\n", err.c_str()); return 2;
        }
    } else if (!loadCues(cuesPath, samples, err)) {
        std::fprintf(stderr, "%s\n", err.c_str()); return 1;
    }
```

Extend the usage text:

```cpp
        "  --synth SPEC              synthesise cues instead of reading a file:\n"
        "                              step:<g>:<durSec>\n"
        "                              sine:<hz>:<g>:<durSec>\n"
        "                              chirp:<f0>-<f1>:<g>:<durSec>\n"
        "  --synth-dt SEC            timestep for --synth (default 1/60)\n"
```

- [ ] **Step 3: Build and smoke-test**

```bash
cd MotionProviderPlugin && cmake --build tools/build && \
./tools/build/washout_replay --synth step:0.3:30 --config configuration.toml --out /tmp/step.csv
```
Expected: `replayed ~1800 samples`, with a non-zero `heave saturated ...%`.

- [ ] **Step 4: Commit**

```bash
git add MotionProviderPlugin/tools/washout_replay.cpp
git commit -m "feat(MotionProviderPlugin): synthetic cue generator for replay"
```

---

### Task 9: Metrics script

**Files:**
- Create: `MotionProviderPlugin/tools/washout_metrics.py`

**Interfaces:**
- Consumes: run CSVs from `washout_replay --out` and recordings from the plugin.
- Produces: the metric table every campaign stage judges against.

- [ ] **Step 1: Write the script**

Create `tools/washout_metrics.py`:

```python
#!/usr/bin/env python3
"""Metrics for motion-platform tuning runs.

Reads telemetry CSVs (from the plugin or from washout_replay) and prints one
row per file. Everything is numpy-only so it runs wherever numpy is installed.

Metric notes:

  wrms       RMS of heave acceleration band-limited to 0.1-0.63 Hz. This is a
             documented band emphasis, NOT a conformant ISO-2631 Wk weighting.
             It is used only to rank candidates against each other.
  lag_ms     Shift maximising the cross-correlation between the drive cue and
             the commanded pose. The washout is a high-pass, not a delay line,
             so the absolute value is not a physical latency -- the meaningful
             quantity is the DELTA against the baseline run.
"""
import argparse
import csv
import sys

import numpy as np

BAND_LO_HZ = 0.1
BAND_HI_HZ = 0.63


def load(path):
    with open(path, newline="") as fh:
        rows = list(csv.reader(fh))
    header, data = rows[0], rows[1:]
    cols = {name: i for i, name in enumerate(header)}
    arr = np.array([[float(v) for v in r] for r in data if r], dtype=float)
    return cols, arr


def column(cols, arr, name, default=0.0):
    if name not in cols:
        return np.full(arr.shape[0], default)
    return arr[:, cols[name]]


def band_limited_rms(signal, fs, lo, hi):
    """RMS of `signal` after keeping only the [lo, hi] Hz band."""
    n = signal.size
    if n < 4:
        return 0.0
    spec = np.fft.rfft(signal - signal.mean())
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    spec[(freqs < lo) | (freqs > hi)] = 0.0
    return float(np.sqrt(np.mean(np.fft.irfft(spec, n) ** 2)))


def band_ratio(signal, fs, lo, hi):
    n = signal.size
    if n < 4:
        return 0.0
    spec = np.abs(np.fft.rfft(signal - signal.mean())) ** 2
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    total = spec.sum()
    if total <= 0.0:
        return 0.0
    return float(spec[(freqs >= lo) & (freqs <= hi)].sum() / total)


def xcorr_lag_ms(drive, response, fs, max_lag_sec=1.0):
    """Shift (ms) of `response` relative to `drive` maximising correlation."""
    d = drive - drive.mean()
    r = response - response.mean()
    if d.std() == 0 or r.std() == 0:
        return 0.0
    max_lag = int(max_lag_sec * fs)
    best_lag, best_val = 0, -np.inf
    for lag in range(0, max_lag + 1):
        a = d[: d.size - lag]
        b = r[lag:]
        if a.size < 16:
            break
        val = float(np.dot(a, b) / a.size)
        if val > best_val:
            best_val, best_lag = val, lag
    return 1000.0 * best_lag / fs


def metrics(path):
    cols, arr = load(path)
    if arr.shape[0] < 8:
        raise SystemExit(f"{path}: too few rows")

    dt = column(cols, arr, "dt_real", 1.0 / 60.0)
    fs = 1.0 / float(np.median(dt))
    n = arr.shape[0]

    heave_mm = column(cols, arr, "live_heave")
    heave_m = heave_mm / 1000.0
    # Second difference -> acceleration. Uniform-dt assumption is fine here:
    # dt jitter is a few percent and this is a comparative metric.
    accel = np.gradient(np.gradient(heave_m, 1.0 / fs), 1.0 / fs)

    g_cue = column(cols, arr, "g_nrml", 1.0) - 1.0

    sat = {
        "heave": 100.0 * column(cols, arr, "heave_clamped").mean(),
        "rot_r": 100.0 * column(cols, arr, "rot_roll_clamped").mean(),
        "rot_p": 100.0 * column(cols, arr, "rot_pitch_clamped").mean(),
        "rot_y": 100.0 * column(cols, arr, "rot_yaw_clamped").mean(),
        "tilt_rate": 100.0 * column(cols, arr, "tilt_rate_active").mean(),
        "envelope": 100.0 * (column(cols, arr, "reach_scale", 1.0) < 1.0).mean(),
        "sl_vel": 100.0 * (column(cols, arr, "sl_vel_clip") > 0).mean(),
        "sl_acc": 100.0 * (column(cols, arr, "sl_acc_clip") > 0).mean(),
    }

    jerks = []
    for i in range(6):
        sent = column(cols, arr, f"sent{i}")
        if sent.size > 3:
            jerks.append(np.percentile(np.abs(np.diff(sent, n=3)), 95))
    jerk_p95 = float(max(jerks)) if jerks else 0.0

    peak_raw = float(np.max(np.abs(column(cols, arr, "heave_pos_raw"))))
    limit_hint = float(np.max(np.abs(heave_mm)))

    return {
        "file": path,
        "rows": n,
        "sec": float(np.sum(dt)),
        "fs": fs,
        "sat_heave": sat["heave"],
        "sat_rot": max(sat["rot_r"], sat["rot_p"], sat["rot_y"]),
        "sat_tilt_rate": sat["tilt_rate"],
        "sat_envelope": sat["envelope"],
        "sat_sl_vel": sat["sl_vel"],
        "sat_sl_acc": sat["sl_acc"],
        "wrms": band_limited_rms(accel, fs, BAND_LO_HZ, BAND_HI_HZ),
        "band_ratio": band_ratio(accel, fs, BAND_LO_HZ, BAND_HI_HZ),
        "jerk_p95": jerk_p95,
        "lag_ms": xcorr_lag_ms(g_cue, heave_mm, fs),
        "peak_raw_mm": peak_raw,
        "peak_out_mm": limit_hint,
    }


# (column name, header format, value format)
HEADERS = [
    ("file", "{:<34}", "{:<34}"),
    ("sec", "{:>7}", "{:>7.1f}"),
    ("sat_heave", "{:>10}", "{:>10.2f}"),
    ("sat_rot", "{:>8}", "{:>8.2f}"),
    ("sat_envelope", "{:>13}", "{:>13.2f}"),
    ("sat_sl_acc", "{:>11}", "{:>11.2f}"),
    ("wrms", "{:>9}", "{:>9.4f}"),
    ("band_ratio", "{:>11}", "{:>11.3f}"),
    ("jerk_p95", "{:>9}", "{:>9.1f}"),
    ("lag_ms", "{:>8}", "{:>8.1f}"),
    ("peak_raw_mm", "{:>12}", "{:>12.1f}"),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+")
    ap.add_argument("--csv", action="store_true", help="emit CSV instead of a table")
    args = ap.parse_args()

    results = [metrics(p) for p in args.files]

    if args.csv:
        w = csv.DictWriter(sys.stdout, fieldnames=list(results[0].keys()))
        w.writeheader()
        w.writerows(results)
        return

    print("".join(hfmt.format(name) for name, hfmt, _ in HEADERS))
    for r in results:
        print("".join(vfmt.format(r[name]) for name, _, vfmt in HEADERS))


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run it against the baseline replay**

```bash
cd MotionProviderPlugin && python3 tools/washout_metrics.py /tmp/replay.csv
```
Expected: one row. **`sat_heave` is the headline number** — the spec predicts 80–100 % for a recording that includes any manoeuvring.

- [ ] **Step 3: Commit**

```bash
git add MotionProviderPlugin/tools/washout_metrics.py
git commit -m "feat(MotionProviderPlugin): tuning metrics script"
```

---

### Task 10: Documentation scaffolding

**Files:**
- Create: `docs/motion-tuning/README.md`, `docs/motion-tuning/tuning-log.md`, `docs/motion-tuning/baseline-metrics.md`

**Interfaces:**
- Consumes: the finished harness.
- Produces: the entry point a fresh session reads before touching anything.

- [ ] **Step 1: Write the operating manual**

Create `docs/motion-tuning/README.md` covering, with the exact commands from Tasks 6–9: how to record in X-Plane (Record button, `[telemetry]` auto-start), how to export cue-only reference files, how to run a single replay, a sweep and `--verify`, how to read each metric and what its gate is, and how to promote a candidate to the rig (edit `configuration.toml`, press Reload config — no rebuild). State the two blocking rules prominently: `--verify` must pass before any number is trusted, and `lag_ms` must stay within baseline + 15 ms.

- [ ] **Step 2: Create the living log**

Create `docs/motion-tuning/tuning-log.md` with a table header and one worked example row explaining each column:

```markdown
# Tuning Log

One row per change. Append, never rewrite. Offline metrics come from
`washout_metrics.py`; the rig verdict is a sentence in the pilot's own words.

| Date | Stage | Parameter | Old → New | sat_heave% | wrms | jerk_p95 | lag_ms (Δ) | Rig verdict | Decision |
|---|---|---|---|---|---|---|---|---|---|
```

- [ ] **Step 3: Create the baseline placeholder**

Create `docs/motion-tuning/baseline-metrics.md` stating that it is filled by Stage 2 and must not be edited afterwards, with the recording environment fields to capture: aircraft (Piper Arrow III, vFlightAir PA28-201R), X-Plane version, weather preset, machine, framerate, and the `configuration.toml` git revision.

- [ ] **Step 4: Commit**

```bash
git add docs/motion-tuning/
git commit -m "docs: motion tuning harness manual and logs"
```

---

# Part B — Run the campaign

Part B is procedural. Each stage ends with a row appended to `docs/motion-tuning/tuning-log.md` and a commit. The gates from the spec apply to every candidate:

1. `lag_ms` ≤ baseline + 15 ms — breaking this disqualifies a candidate before it is ever flown.
2. `sat_heave`, `wrms` and `jerk_p95` must all fall (Stage 8 inverts this — see below).
3. Rig veto: if it feels worse, feel wins, and the disagreement is logged.

---

### Stage 1: Verify the analysis (Mac, no changes)

- [ ] Run the frequency sweep:

```bash
cd MotionProviderPlugin && ./tools/build/washout_replay \
    --synth chirp:0.02-5.0:0.3:300 --config configuration.toml --out /tmp/chirp.csv
python3 tools/washout_metrics.py /tmp/chirp.csv
```

- [ ] Find the saturation threshold by sweeping the input amplitude. Run `--synth sine:0.067:<g>:120` for `g` in `0.01, 0.02, 0.03, 0.05, 0.1` and record where `sat_heave` first exceeds 0.

**Expected:** clamp onset near `|Δg| ≈ 0.022 g`, and the largest raw excursion near 0.067 Hz.

- [ ] **Gate:** if clamp onset is within roughly a factor of two of 0.022 g, the diagnosis holds — continue. **If it is an order of magnitude off, stop the campaign and re-derive.** Record the outcome either way.

- [ ] Append the result to `baseline-metrics.md` and commit.

---

### Stage 2: Record the reference set and freeze the baseline (Mac)

- [ ] Record six segments of 60–90 s each with the Arrow III, one file per segment: `cruise_calm`, `steep_turns`, `climb_descent`, `turbulence`, `ground_takeoff`, `approach_landing`. Weather preset fixed and noted.
- [ ] Record one continuous 8–10 min mixed flight as `acceptance`.
- [ ] Run `--verify` on **every** file. All must pass bit-exact.
- [ ] Export cue-only copies and gzip them into `MotionProviderPlugin/reference/`.
- [ ] Run `washout_metrics.py` over all seven and paste the table into `baseline-metrics.md`, together with the environment fields.
- [ ] **Gate:** `cruise_calm` is the segment that matters most. The spec predicts `sat_heave` of 80–100 % there. Note the actual figure — it is the number the whole campaign is measured against.
- [ ] Commit the reference files and the baseline.

---

### Stage 3: Washout time constants (Mac → rig)

The two poles of one washout, moved together. This is the campaign's primary lever.

- [ ] Sweep offline against every segment:

```bash
for seg in cruise_calm steep_turns climb_descent turbulence approach_landing; do
  ./tools/build/washout_replay --cues reference/$seg.csv --config configuration.toml \
      --sweep washout.heave_pos_washout_tau=2.0,1.0,0.6,0.4,0.25 --out /tmp/$seg
done
```

Repeat with `heave_vel_washout_tau` set to the same value via `--set` on each run, so the pair moves together.

- [ ] Check dt robustness on the two best candidates: re-run with `--resample-dt 0.0111` (90 fps) and `--resample-dt 0.0333` (30 fps). A candidate whose metrics shift materially with framerate is fragile — prefer the stable one.
- [ ] Pick 2–3 finalists on the gates. Log their offline numbers.
- [ ] **Rig session 1.** For each finalist: edit `configuration.toml`, press Reload config, fly `cruise_calm` and `steep_turns` from the saved `.sit`. Record telemetry for each. Judge the feel in words before looking at the numbers.
- [ ] Record the winner in the tuning log with both the numbers and the verdict, set it in `configuration.toml`, commit.

---

### Stage 4: High pass (Mac → same rig session)

- [ ] With Stage 3's winner in place, sweep `washout.heave_hp_tau=0.5,1.0,2.0,4.0` across the segments.
- [ ] **Gate:** watch `band_ratio` — this knob decides how much low-frequency content reaches the platform, which is exactly the sickness band.
- [ ] Fly the finalists in the same rig session as Stage 3 (TOML-only, no rebuild). Log, set, commit.

---

### Stage 5: Anti-windup and soft saturation (Mac, code)

Structural change, expected to be barely perceptible alone — its gate is objective.

- [ ] Add `heave_soft_knee` (default 0.6) to `WashoutConfig`, `MotionConfig::loadWashout`, `writeDefaults`, and the replay tool's `kWashoutKeys` table.
- [ ] Write the failing test in `tests/test_washout.cpp` first: with soft-knee saturation the output must be continuous through a reversal, and the integrator state must no longer be clamped (`heavePosRaw` may exceed the limit while the *output* does not).
- [ ] Implement: stop assigning the clamp back to `heavePos_`; apply a soft knee to the output instead — identity below `knee * limit`, smoothly asymptotic to `limit` above it.
- [ ] **Gate:** `jerk_p95` at reversals falls; `lag_ms` unchanged; all eleven test suites green.
- [ ] Cross-build for Windows (`./build-xc-windows.sh`) so it travels with Stage 6 to the rig.
- [ ] Log and commit. No dedicated rig flight.

---

### Stage 6: Per-DOF smoothing (Mac → rig)

- [ ] Replace `smooth_tau` with per-DOF values (`smooth_tau_heave`, `smooth_tau_rot`), keeping `smooth_tau` readable as a fallback so existing config files keep working.
- [ ] Extend the tests, the config loader, `writeDefaults` and `kWashoutKeys`.
- [ ] Sweep `smooth_tau_heave=0.0,0.01,0.02,0.04,0.08` offline. **`lag_ms` is the binding gate here** — this knob buys smoothness with latency, which is exactly what the campaign refuses to spend.
- [ ] **Rig session 2.** Fly Stage 5 + Stage 6 finalists together. Log, set, commit.

---

### Stage 7: Decouple envelope scaling (Mac, conditional)

- [ ] Check `sat_envelope` in the logs from Stages 3–6.
- [ ] **If it is below ~1 % everywhere, skip this stage and record that it was skipped and why.** The coupling is real but irrelevant if the envelope is never hit.
- [ ] Otherwise: clamp heave against its own limit before the pose-wide bisection, so one saturating DOF stops attenuating the others. Test first, then implement, then log and commit. The rig check rides with Stage 8.

---

### Stage 8: Bring the amplitude back (Mac → rig)

The gate inverts here: raising gain necessarily raises motion.

- [ ] Sweep `washout.heave_gain=0.15,0.25,0.35,0.5,0.7` across all segments.
- [ ] **Gates:** `sat_heave` stays under its Stage-3 target; `jerk_p95` at or below the Stage-7 value; `lag_ms` within budget; `wrms` may rise **only if `band_ratio` does not** — more cue is the goal, more energy in the sickness band is not.
- [ ] **Rig session 3.** Fly the top two. This is the stage where the pilot's judgement matters most: the question is not "is it smooth" but "does it tell me what the aircraft is doing".
- [ ] Log, set, commit.

---

### Stage 9: Cross-check the other channels (Mac)

- [ ] Run the same analysis on the rotational and tilt channels across all segments: `sat_rot`, `sat_tilt_rate`, and their own `lag_ms`.
- [ ] The rotational channel has a single integrator, so its `|G|` is orders of magnitude smaller than heave's — expect much less saturation. **If `sat_rot` also turns out high, that is a second finding and gets its own stage**, not a quick fix appended here.
- [ ] Check the effects layer on `ground_takeoff` and `approach_landing`: `rumble_gain` and `touchdown_gain` were reduced to 45 % during the amplitude-first tuning and may deserve the same treatment as heave.
- [ ] Log findings. No changes without their own stage.

---

### Stage 10: Acceptance and freeze (rig)

- [ ] Replay the `acceptance` segment against the final config; compare every metric against `baseline-metrics.md`.
- [ ] **Rig session 4.** Fly the full acceptance profile. The question to answer in writing: is the pumping gone, and is there more usable cue than at the start?
- [ ] Write the closing summary into `tuning-log.md`: what changed, by how much, what is still open.
- [ ] Freeze `configuration.toml`, commit, and merge the branch.
- [ ] Update the `motion-heave-tuning-campaign` memory with the outcome and the final values.

---

## Self-Review Notes

- **Spec coverage:** telemetry (Tasks 3–5), replay CLI with `--set`/`--sweep`/`--resample-dt`/`--cues-only`/`--synth` (Tasks 6–8), the bit-exact self-test (Task 7 Step 5), metrics and gates (Task 9), reference set (Stage 2), all four software changes — SW1 Stage 5, SW3 Stage 6, SW4 Stage 7, SW2 being the harness itself — and all eleven campaign stages.
- **One deviation from the spec, deliberate:** the spec's telemetry column list named a single `pose_*` group. The plan splits it into `live_*` (pre-blend) and `cmd_*` (post-blend), because only the pre-blend pose is a pure function of (cues, dt, config) and therefore the only thing the bit-exact self-test can check without modelling the arm ramp offline. `--cues-only` is likewise folded into the recording format rather than being a separate flag: the cue columns are already a strict subset, so a cue-only export is a column selection, documented in the README.
- **Open risk:** Task 5 Step 8 may find that 60 Hz file writes perturb frame timing. The mitigation (move the write to the serial I/O thread) is identified but not planned in detail, because whether it is needed is unknown until measured.
