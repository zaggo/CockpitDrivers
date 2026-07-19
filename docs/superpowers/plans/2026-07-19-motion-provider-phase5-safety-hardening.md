# MotionProviderPlugin — Phase 5 (Safety hardening) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add plugin-side safety hardening on the existing BFF link — runaway detection, a connection/pause watchdog, and latching home-on-fault — so an abnormal command, a NaN, a lost serial link, or an X-Plane stall can never drive or hold the actuators unsafely.

**Architecture:** A new pure `SafetyMonitor` evaluates fault conditions each tick (NaN in the commanded pose, sustained out-of-sanity command = runaway, serial-lost-while-armed) and latches a `FaultCode` until explicitly cleared. `MotionProvider` feeds it observations, and on any fault forces a ramp-to-park (disarm) that stays latched until the operator clicks ARM (which clears the fault and re-arms). A dt/pause guard clamps the timestep fed to the filters so an X-Plane stall can't diverge the washout. The status window shows the latched fault reason.

**Tech Stack:** C++17, existing CMake + native test target. No X-Plane SDK in the new pure code. No MotionGateway change (BFF link unchanged); the CRC native protocol + bidirectional heartbeat remain a separate future follow-up.

## Global Constraints

- C++17. `SafetyMonitor`, `SafetyConfig` include NO X-Plane SDK header (native-testable).
- Faults are **latching**: once tripped, the platform ramps to the park pose and stays DISARMED with the reason shown; only an explicit ARM clears the fault and re-arms. No auto-clear.
- Home-on-fault = the existing Phase-4 park pose (low, level) via the arm ramp's `requestDisarm()`.
- Only `Plugin.cpp` calls the plugin ABI; `DataRefManager` stays the only XPLM dataref reader; serial I/O stays on `SerialLink`'s thread.
- SANDBOX: `cmake`/`clang`/`make` BLOCKED. Implementers WRITE FILES ONLY; the user builds, runs native tests, and does all hardware verification. Stage ONLY the files each task lists (no `git add -A`).
- Bump the status window version to `v0.7 (Phase 5)`.
- Work on branch `feature/motion-provider-plugin`.

## Fault model (authoritative)

`enum class FaultCode { None, Nan, Runaway, SerialLost }`. Each tick `SafetyMonitor::update(rawCmd, finite, serialLostWhileArmed, dt)`:
- If already faulted → return (latched; ignore observations until `clear()`).
- `!finite` (any commanded-pose field NaN/inf) → latch **Nan**.
- `serialLostWhileArmed` → latch **SerialLost**.
- Any raw-command DOF beyond a hard sanity bound (`|rot| > runawayTiltDeg` or `|trans| > runawayTransMm`) sustained for `> runawayHoldSec` → latch **Runaway** (catches a diverging washout / bad config); accumulator resets whenever the command is back in bounds.

`runawayHoldSec` uses the same (clamped) dt as the filters. The raw command checked is the *pre-envelope-clamp* pose, so divergence is caught before the clamp masks it.

`MotionProvider`: `serialLostWhileArmed` = arm ramp not fully disarmed AND a port has ever connected AND `serial_->isConnected()` is now false. On a fault edge → `armRamp_.requestDisarm()`. ARM click while faulted → `monitor_.clear()` then arm.

## File structure

- Create `src/SafetyMonitor.h`, `src/SafetyMonitor.cpp`.
- Modify `src/SafetyConfig.h` (runaway + max-dt fields), `src/MotionConfig.cpp` (load them).
- Modify `src/MotionProvider.h/.cpp` (own monitor, fault detection, latched disarm, clear-on-arm, dt guard).
- Modify `src/StatusData.h` (fault code/reason), `src/StatusWindow.cpp` (fault display + version).
- Create `tests/test_monitor.cpp`; modify `CMakeLists.txt`, `tests/CMakeLists.txt`.

---

### Task 1: SafetyMonitor (pure) + config + tests

**Files:**
- Create: `src/SafetyMonitor.h`, `src/SafetyMonitor.cpp`, `tests/test_monitor.cpp`
- Modify: `src/SafetyConfig.h`, `src/MotionConfig.cpp`, `CMakeLists.txt`, `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `enum class FaultCode { None, Nan, Runaway, SerialLost }`; `class SafetyMonitor` with `void setConfig(const SafetyConfig&)`, `void update(const Pose& rawCmd, bool finite, bool serialLostWhileArmed, double dt)`, `FaultCode fault() const`, `const char* reason() const`, `void clear()`.

- [ ] **Step 1: Add fields to `src/SafetyConfig.h`** — insert before `defaults()`:

```cpp
    // Runaway / watchdog (Phase 5). A command DOF beyond these hard sanity
    // bounds, sustained for runawayHoldSec, latches a Runaway fault. maxDtSec
    // clamps the timestep fed to the filters so an X-Plane stall can't diverge
    // the washout.
    double runawayTiltDeg = 45.0;   // |roll|/|pitch|/|yaw| sanity bound (deg)
    double runawayTransMm = 500.0;  // |surge|/|sway|/|heave| sanity bound (mm)
    double runawayHoldSec = 1.0;    // sustained out-of-bounds -> fault
    double maxDtSec       = 0.1;    // filter timestep clamp
```

- [ ] **Step 2: Load them in `src/MotionConfig.cpp`** — add to the `[safety]` block in `loadSafety`:

```cpp
        getDouble(*t, "runaway_tilt_deg", s.runawayTiltDeg);
        getDouble(*t, "runaway_trans_mm", s.runawayTransMm);
        getDouble(*t, "runaway_hold_sec", s.runawayHoldSec);
        getDouble(*t, "max_dt_sec", s.maxDtSec);
```

- [ ] **Step 3: Write `src/SafetyMonitor.h`**

```cpp
#pragma once
#include "Pose.h"
#include "SafetyConfig.h"

enum class FaultCode { None, Nan, Runaway, SerialLost };

// Evaluates and LATCHES safety faults. Pure, dt-driven, no X-Plane deps.
class SafetyMonitor {
public:
    void setConfig(const SafetyConfig& cfg) { cfg_ = cfg; }

    // Observe one tick. rawCmd is the pre-clamp commanded pose; finite is false
    // if any commanded value is NaN/inf; serialLostWhileArmed is true if the
    // link dropped while not fully disarmed. Once faulted, further calls are
    // ignored until clear().
    void update(const Pose& rawCmd, bool finite, bool serialLostWhileArmed, double dt);

    FaultCode   fault() const { return fault_; }
    const char* reason() const;
    void clear() { fault_ = FaultCode::None; oobAccum_ = 0.0; }

private:
    SafetyConfig cfg_;
    FaultCode fault_ = FaultCode::None;
    double oobAccum_ = 0.0;   // seconds the command has been out of sanity bounds
};
```

- [ ] **Step 4: Write `src/SafetyMonitor.cpp`**

```cpp
#include "SafetyMonitor.h"
#include <cmath>

void SafetyMonitor::update(const Pose& c, bool finite, bool serialLostWhileArmed, double dt) {
    if (fault_ != FaultCode::None) return;   // latched
    if (dt < 0.0) dt = 0.0;

    if (!finite) { fault_ = FaultCode::Nan; return; }
    if (serialLostWhileArmed) { fault_ = FaultCode::SerialLost; return; }

    const bool oob =
        std::fabs(c.roll)  > cfg_.runawayTiltDeg ||
        std::fabs(c.pitch) > cfg_.runawayTiltDeg ||
        std::fabs(c.yaw)   > cfg_.runawayTiltDeg ||
        std::fabs(c.surge) > cfg_.runawayTransMm ||
        std::fabs(c.sway)  > cfg_.runawayTransMm ||
        std::fabs(c.heave) > cfg_.runawayTransMm;

    if (oob) {
        oobAccum_ += dt;
        if (oobAccum_ >= cfg_.runawayHoldSec) fault_ = FaultCode::Runaway;
    } else {
        oobAccum_ = 0.0;
    }
}

const char* SafetyMonitor::reason() const {
    switch (fault_) {
        case FaultCode::Nan:        return "FAULT: NaN in command";
        case FaultCode::Runaway:    return "FAULT: runaway command";
        case FaultCode::SerialLost: return "FAULT: serial link lost";
        default:                    return "";
    }
}
```

- [ ] **Step 5: Write `tests/test_monitor.cpp`**

```cpp
#include "SafetyMonitor.h"
#include "SafetyConfig.h"
#include "Pose.h"
#include <cstdio>
#include <cmath>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }

int main() {
    const double dt = 1.0/60.0;
    const SafetyConfig cfg = SafetyConfig::defaults();  // tilt 45, trans 500, hold 1.0

    // Nominal command -> no fault.
    {
        SafetyMonitor m; m.setConfig(cfg);
        Pose ok; ok.pitch = 3.0f; ok.heave = 10.0f;
        for (int i=0;i<300;i++) m.update(ok, true, false, dt);
        check(m.fault() == FaultCode::None, "nominal -> no fault");
    }

    // NaN -> latched immediately.
    {
        SafetyMonitor m; m.setConfig(cfg);
        Pose bad; bad.pitch = std::nanf("");
        m.update(bad, false, false, dt);   // finite=false
        check(m.fault() == FaultCode::Nan, "NaN latches");
        m.update(Pose{}, true, false, dt); // stays latched
        check(m.fault() == FaultCode::Nan, "fault is latched");
        m.clear();
        check(m.fault() == FaultCode::None, "clear resets");
    }

    // Serial lost while armed -> latched.
    {
        SafetyMonitor m; m.setConfig(cfg);
        m.update(Pose{}, true, true, dt);
        check(m.fault() == FaultCode::SerialLost, "serial-lost latches");
    }

    // Sustained out-of-bounds -> Runaway after holdSec; brief OOB does not.
    {
        SafetyMonitor m; m.setConfig(cfg);
        Pose oob; oob.pitch = 90.0f;   // > 45 deg
        // brief (< 1s): 30 ticks = 0.5s
        for (int i=0;i<30;i++) m.update(oob, true, false, dt);
        check(m.fault() == FaultCode::None, "brief OOB -> no fault yet");
        // back in bounds resets the accumulator
        for (int i=0;i<30;i++) m.update(Pose{}, true, false, dt);
        check(m.fault() == FaultCode::None, "in-bounds keeps clear");
        // sustained > 1s
        for (int i=0;i<80;i++) m.update(oob, true, false, dt);  // ~1.33s
        check(m.fault() == FaultCode::Runaway, "sustained OOB -> runaway");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
```

- [ ] **Step 6: Wire builds.** `CMakeLists.txt`: SOURCES += `src/SafetyMonitor.cpp`; HEADERS += `src/SafetyMonitor.h`. `tests/CMakeLists.txt`:

```cmake
add_executable(test_monitor test_monitor.cpp ../src/SafetyMonitor.cpp)
target_include_directories(test_monitor PRIVATE ../src)
add_test(NAME monitor COMMAND test_monitor)
```

- [ ] **Step 7: Commit** (stage ONLY these files)

```bash
git add MotionProviderPlugin/src/SafetyMonitor.h MotionProviderPlugin/src/SafetyMonitor.cpp MotionProviderPlugin/src/SafetyConfig.h MotionProviderPlugin/src/MotionConfig.cpp MotionProviderPlugin/tests/test_monitor.cpp MotionProviderPlugin/CMakeLists.txt MotionProviderPlugin/tests/CMakeLists.txt
git commit -m "feat(motion): Phase 5 Task 1 - SafetyMonitor (runaway/NaN/serial-lost fault latch)"
```

- [ ] **Step 8: Manual test (user)** — `cd MotionProviderPlugin/tests && cmake -B build && cmake --build build && ./build/test_monitor` → `0 failures`.

---

### Task 2: Integrate faults + watchdog + dt guard + fault display

**Files:**
- Modify: `src/MotionProvider.h`, `src/MotionProvider.cpp`, `src/StatusData.h`, `src/StatusWindow.cpp`

**Interfaces:**
- Consumes: `SafetyMonitor` (Task 1). Produces: latched fault behavior; StatusData `faultCode` (int) + `faultReason` (string); window fault line; version v0.7.

- [ ] **Step 1: `src/MotionProvider.h`** — add `#include "SafetyMonitor.h"`; add members:

```cpp
    SafetyMonitor monitor_;
    bool serialWasConnected_ = false;   // latches true once a link comes up
```

- [ ] **Step 2: `src/StatusData.h`** — add fields (after `armBlend`):

```cpp
    int         faultCode = 0;       // FaultCode: 0 None,1 Nan,2 Runaway,3 SerialLost
    std::string faultReason;         // human-readable, empty if no fault
```

- [ ] **Step 3: `src/MotionProvider.cpp`** — integrate.

In `initialize()` after building `safety_`: `monitor_.setConfig(safetyCfg_);`. In `reloadConfig()` after `safetyCfg_` reload: `monitor_.setConfig(safetyCfg_);`.

In `onUiAction`, change the ARM case so ARM clears a latched fault first:

```cpp
        case UI_ARM_TOGGLE:
            if (monitor_.fault() != FaultCode::None) monitor_.clear();  // ARM clears + re-arms
            armRamp_.toggle();
            break;
```

Rewrite the top of `onFlightLoopTick` to clamp dt for the filters, and the middle to run the monitor and force disarm on fault. The full function:

```cpp
void MotionProvider::onFlightLoopTick(float elapsedSec) {
    if (dataRefs_) latestCues_ = dataRefs_->sample();

    if (reloadFlashRemaining_ > 0.0f) {
        reloadFlashRemaining_ -= elapsedSec;
        if (reloadFlashRemaining_ < 0.0f) reloadFlashRemaining_ = 0.0f;
    }

    // Pause/stall guard: clamp the timestep the stateful filters/ramp see so a
    // long X-Plane stall can't diverge them. Serial reconnect uses real dt.
    double dt = static_cast<double>(elapsedSec);
    if (dt > safetyCfg_.maxDtSec) dt = safetyCfg_.maxDtSec;

    Pose rawLive;
    if (manualMode_) {
        rawLive = manualPose_;
    } else if (washout_ && effects_) {
        Pose w = washout_->update(latestCues_, dt);
        Pose e = effects_->update(latestCues_, dt);
        rawLive.surge = w.surge + e.surge;  rawLive.sway  = w.sway  + e.sway;
        rawLive.heave = w.heave + e.heave;  rawLive.roll  = w.roll  + e.roll;
        rawLive.pitch = w.pitch + e.pitch;  rawLive.yaw   = w.yaw   + e.yaw;
    }

    // Watchdog + runaway/NaN monitor (before the envelope clamp masks divergence).
    if (serial_ && serial_->isConnected()) serialWasConnected_ = true;
    const bool notDisarmed = (armRamp_.state() != ArmState::Disarmed);
    const bool serialLost = notDisarmed && serialWasConnected_ &&
                            serial_ && !serial_->isConnected();
    const bool finite =
        std::isfinite(rawLive.surge) && std::isfinite(rawLive.sway) &&
        std::isfinite(rawLive.heave) && std::isfinite(rawLive.roll) &&
        std::isfinite(rawLive.pitch) && std::isfinite(rawLive.yaw);
    monitor_.update(rawLive, finite, serialLost, dt);
    if (monitor_.fault() != FaultCode::None) armRamp_.requestDisarm();  // home-on-fault (latched)

    // Arm ramp + pose-space blend -> IK.
    armRamp_.update(dt, safetyCfg_.armRampSec, safetyCfg_.disarmRampSec);
    latestPose_ = blendedCommand(rawLive);
    if (kin_) latestSolve_ = kin_->solve(latestPose_);

    uint16_t target[6];
    for (int i = 0; i < 6; ++i) target[i] = latestSolve_.setpoints[i];
    if (safety_) safety_->limit(target, dt, sentSetpoints_);
    else for (int i=0;i<6;i++) sentSetpoints_[i] = target[i];

    if (serial_) {
        uint8_t frame[BffEncoder::kFrameSize];
        BffEncoder::encode(sentSetpoints_, frame);
        serial_->setFrame(frame, sizeof(frame));
        serial_->update(elapsedSec);   // real dt for reconnect timing
    }

    statusAccumSec_ += elapsedSec;
    if (statusAccumSec_ >= 1.0f) { statusAccumSec_ = 0.0f; pushStatus(); }
}
```

Add `#include <cmath>` at the top for `std::isfinite`.

Extend `pushStatus()`:

```cpp
    sd.faultCode = static_cast<int>(monitor_.fault());
    sd.faultReason = monitor_.reason();
```

- [ ] **Step 4: `src/StatusWindow.cpp`** — show the fault and bump the version.

Change the title to `"Motion Provider v0.7 (Phase 5)"`. In `draw()`, right after the ARM button / serial-status block (before the port line), add a prominent fault line when faulted:

```cpp
    if (data_.faultCode != 0) {
        drawString(x, y, data_.faultReason.c_str(), 1.0f, 0.3f, 0.3f);
        y -= 16;
    }
```

- [ ] **Step 5: Commit** (stage ONLY these files)

```bash
git add MotionProviderPlugin/src/MotionProvider.h MotionProviderPlugin/src/MotionProvider.cpp MotionProviderPlugin/src/StatusData.h MotionProviderPlugin/src/StatusWindow.cpp
git commit -m "feat(motion): Phase 5 Task 2 - latched home-on-fault, watchdog, dt guard, fault display"
```

- [ ] **Step 6: Manual verification (user)** — build + load.
  - Normal flight: no fault; ARM/DISARM as before.
  - Unplug the serial cable while ARMED → within a tick the window shows `FAULT: serial link lost`, the platform ramps to park, and it stays disarmed. Re-plug (auto-reconnects), then click ARM → fault clears and it re-arms.
  - Pause X-Plane for a while, unpause → no washout jerk (dt clamp), no spurious fault.
  - (Optional) set an absurd `[washout]` gain so the filter diverges → `FAULT: runaway command` after ~1 s, parked. Reload sane config, ARM to clear.

---

## Self-Review

- **Spec coverage:** spec §4 Phase 5 — runaway detection (sustained out-of-sanity command), connection watchdog (serial-lost-while-armed + pause/dt guard), graceful home-on-fault (latched ramp to park). Startup soft-start was completed earlier (the arm ramp). The optional CRC native protocol + MotionGateway rewrite is explicitly deferred to a separate follow-up (chosen scope: plugin-side hardening on BFF).
- **Placeholder scan:** no TBD/TODO. All thresholds are config-tunable with conservative defaults.
- **Type consistency:** `FaultCode` used in `SafetyMonitor` and mapped to `StatusData.faultCode` (int) / `faultReason`; `SafetyMonitor::update(const Pose&, bool, bool, double)` fed from `MotionProvider`; `armRamp_.requestDisarm()` (Phase 4) is the home-on-fault mechanism; `serialWasConnected_` gates false "serial lost" before first connect. dt clamp uses `safetyCfg_.maxDtSec`; the monitor and filters share the clamped dt, `serial_->update` keeps real dt.
- **Constraint check:** `SafetyMonitor`/`SafetyConfig` are XPLM-free (native `test_monitor` links only `SafetyMonitor.cpp`). Faults latch (no auto-clear); ARM is the sole clear path. `DataRefManager`/`Plugin.cpp`/`SerialLink` roles unchanged; no MotionGateway change.
- **Test note:** `test_monitor` covers all three fault types, the latch (stays faulted through good updates), `clear()`, the sustained-vs-brief runaway distinction, and the in-bounds accumulator reset — the properties that matter, not golden numbers.
- **Safety review:** the monitor checks the *raw* command (pre-envelope-clamp), so a diverging washout is caught before the clamp hides it; NaN is caught immediately; serial loss only faults once a link had been established and only while not disarmed; every fault path ends in the same latched ramp-to-park that the operator must acknowledge with ARM.
