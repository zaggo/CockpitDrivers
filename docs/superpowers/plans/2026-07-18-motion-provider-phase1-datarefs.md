# MotionProviderPlugin — Phase 1 (Dataref Listening) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Resolve and sample the X-Plane flight-model datarefs that feed the motion platform, and display the live cue values in the status window for debugging.

**Architecture:** A new `DataRefManager` becomes the single owner of all `XPLMFindDataRef`/`XPLMGetData*` calls (mirroring DCUProviderPlugin). It resolves datarefs on aircraft load and returns a plain `MotionCues` snapshot struct on demand. `MotionProvider` owns it, samples once per 60 Hz output tick, and pushes the latest snapshot to `StatusWindow` at 1 Hz; `StatusWindow` renders the values.

**Tech Stack:** C++17, X-Plane 12 SDK (XPLMDataAccess), existing CMake pipeline.

## Global Constraints

- C++17.
- `DataRefManager` is the ONLY file allowed to call `XPLMFindDataRef` / `XPLMGetData*`. `Plugin.cpp` remains the only file calling the plugin ABI entry points.
- Datarefs are resolved in `onAircraftLoaded()` (called at init and on `XPLM_MSG_PLANE_LOADED`); any absent dataref must fall back to 0/default via null-checked readers, never crash.
- Motion-output tick stays 60 Hz (`Plugin.cpp` `kFlightLoopIntervalSec`); the status window refreshes at 1 Hz (existing accumulator in `MotionProvider::onFlightLoopTick`).
- SANDBOX: `cmake`/`clang`/`make` are BLOCKED. Implementers WRITE FILES ONLY; build + X-Plane load are verified manually by the user. Phase 1 has no off-device-testable pure logic (all code is XPLM-coupled passthrough), so verification is: builds, and the cue values display and move correctly in X-Plane.
- Work on branch `feature/motion-provider-plugin` (current branch, continues from Phase 0).

## Dataref set (stock X-Plane 12, present on every aircraft — no third-party/lazy resolve needed)

| Cue | Dataref | Type | Unit |
|---|---|---|---|
| surge (fwd/aft specific force) | `sim/flightmodel/forces/g_axil` | float | g |
| sway (lateral specific force) | `sim/flightmodel/forces/g_side` | float | g |
| heave (normal specific force) | `sim/flightmodel/forces/g_nrml` | float | g |
| roll rate | `sim/flightmodel/position/P` | float | deg/s |
| pitch rate | `sim/flightmodel/position/Q` | float | deg/s |
| yaw rate | `sim/flightmodel/position/R` | float | deg/s |
| pitch attitude | `sim/flightmodel/position/theta` | float | deg |
| roll attitude | `sim/flightmodel/position/phi` | float | deg |
| on ground | `sim/flightmodel/failures/onground_any` | int | 0/1 |
| groundspeed | `sim/flightmodel/position/groundspeed` | float | m/s |
| engine RPM (engine 0) | `sim/cockpit2/engine/indicators/engine_speed_rpm` | float[] | rpm |
| angle of attack | `sim/flightmodel/position/alpha` | float | deg |

These are the debugging inputs; the user confirms the live values look right when the window displays them (that is the purpose of this phase). If any reads wrong on the user's aircraft, the fix is a one-line dataref-path change.

## File structure

- Create `MotionProviderPlugin/src/MotionCues.h` — the plain snapshot struct (no .cpp; shared by DataRefManager, MotionProvider, StatusWindow).
- Create `MotionProviderPlugin/src/DataRefManager.h` / `.cpp` — resolve + sample.
- Modify `MotionProviderPlugin/CMakeLists.txt` — add the new source/headers.
- Modify `MotionProviderPlugin/src/MotionProvider.h` / `.cpp` — own DataRefManager, sample per tick, hold latest cues, push to window.
- Modify `MotionProviderPlugin/src/StatusWindow.h` / `.cpp` — `update(const MotionCues&)`, render cue lines, widen window.

---

### Task 1: MotionCues struct + DataRefManager

**Files:**
- Create: `MotionProviderPlugin/src/MotionCues.h`
- Create: `MotionProviderPlugin/src/DataRefManager.h`
- Create: `MotionProviderPlugin/src/DataRefManager.cpp`
- Modify: `MotionProviderPlugin/CMakeLists.txt` (add to SOURCES/HEADERS)

**Interfaces:**
- Consumes: nothing from earlier Phase-1 tasks.
- Produces: `struct MotionCues` (12 fields, listed below); `class DataRefManager` with `initialize() -> void`, `onAircraftLoaded() -> void`, `sample() const -> MotionCues`. Task 2 consumes these.

- [ ] **Step 1: Write `MotionProviderPlugin/src/MotionCues.h`**

```cpp
#pragma once

// One sampled snapshot of the flight-model inputs the motion platform reacts to.
// Plain data; produced by DataRefManager, consumed by MotionProvider/StatusWindow
// (and, in later phases, the washout filter and effects layer).
struct MotionCues {
    // Specific forces (body frame), in g
    float surgeG = 0.0f;   // sim/flightmodel/forces/g_axil
    float swayG  = 0.0f;   // sim/flightmodel/forces/g_side
    float heaveG = 0.0f;   // sim/flightmodel/forces/g_nrml

    // Angular rates, deg/s
    float rollRate  = 0.0f; // sim/flightmodel/position/P
    float pitchRate = 0.0f; // sim/flightmodel/position/Q
    float yawRate   = 0.0f; // sim/flightmodel/position/R

    // Attitude, deg
    float pitchDeg = 0.0f;  // sim/flightmodel/position/theta
    float rollDeg  = 0.0f;  // sim/flightmodel/position/phi

    // Effects inputs
    bool  onGround    = false; // sim/flightmodel/failures/onground_any
    float groundspeed = 0.0f;  // sim/flightmodel/position/groundspeed (m/s)
    float engineRpm   = 0.0f;  // sim/cockpit2/engine/indicators/engine_speed_rpm[0]
    float alphaDeg    = 0.0f;  // sim/flightmodel/position/alpha
};
```

- [ ] **Step 2: Write `MotionProviderPlugin/src/DataRefManager.h`**

```cpp
#pragma once
#include "XPLMDataAccess.h"
#include "MotionCues.h"

// The single owner of all XPLMFindDataRef / XPLMGetData* calls for this plugin.
class DataRefManager {
public:
    DataRefManager();
    ~DataRefManager();

    DataRefManager(const DataRefManager&) = delete;
    DataRefManager& operator=(const DataRefManager&) = delete;

    // Resolve datarefs once at startup (delegates to onAircraftLoaded()).
    void initialize();

    // (Re)resolve datarefs; called at init and on XPLM_MSG_PLANE_LOADED.
    void onAircraftLoaded();

    // Read all cues into a fresh snapshot. Safe to call every tick.
    MotionCues sample() const;

private:
    static float readFloat(XPLMDataRef dr, float def = 0.0f);
    static int   readInt(XPLMDataRef dr, int def = 0);
    static float readFloatArrayElem(XPLMDataRef dr, int index);

    XPLMDataRef dr_gAxil_ = nullptr;
    XPLMDataRef dr_gSide_ = nullptr;
    XPLMDataRef dr_gNrml_ = nullptr;
    XPLMDataRef dr_p_ = nullptr;
    XPLMDataRef dr_q_ = nullptr;
    XPLMDataRef dr_r_ = nullptr;
    XPLMDataRef dr_theta_ = nullptr;
    XPLMDataRef dr_phi_ = nullptr;
    XPLMDataRef dr_onGround_ = nullptr;
    XPLMDataRef dr_groundspeed_ = nullptr;
    XPLMDataRef dr_engineRpm_ = nullptr;
    XPLMDataRef dr_alpha_ = nullptr;
};
```

- [ ] **Step 3: Write `MotionProviderPlugin/src/DataRefManager.cpp`**

```cpp
#include "DataRefManager.h"

DataRefManager::DataRefManager() = default;
DataRefManager::~DataRefManager() = default;

void DataRefManager::initialize() {
    onAircraftLoaded();
}

void DataRefManager::onAircraftLoaded() {
    dr_gAxil_       = XPLMFindDataRef("sim/flightmodel/forces/g_axil");
    dr_gSide_       = XPLMFindDataRef("sim/flightmodel/forces/g_side");
    dr_gNrml_       = XPLMFindDataRef("sim/flightmodel/forces/g_nrml");
    dr_p_           = XPLMFindDataRef("sim/flightmodel/position/P");
    dr_q_           = XPLMFindDataRef("sim/flightmodel/position/Q");
    dr_r_           = XPLMFindDataRef("sim/flightmodel/position/R");
    dr_theta_       = XPLMFindDataRef("sim/flightmodel/position/theta");
    dr_phi_         = XPLMFindDataRef("sim/flightmodel/position/phi");
    dr_onGround_    = XPLMFindDataRef("sim/flightmodel/failures/onground_any");
    dr_groundspeed_ = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
    dr_engineRpm_   = XPLMFindDataRef("sim/cockpit2/engine/indicators/engine_speed_rpm");
    dr_alpha_       = XPLMFindDataRef("sim/flightmodel/position/alpha");
}

MotionCues DataRefManager::sample() const {
    MotionCues c;
    c.surgeG      = readFloat(dr_gAxil_);
    c.swayG       = readFloat(dr_gSide_);
    c.heaveG      = readFloat(dr_gNrml_);
    c.rollRate    = readFloat(dr_p_);
    c.pitchRate   = readFloat(dr_q_);
    c.yawRate     = readFloat(dr_r_);
    c.pitchDeg    = readFloat(dr_theta_);
    c.rollDeg     = readFloat(dr_phi_);
    c.onGround    = readInt(dr_onGround_) != 0;
    c.groundspeed = readFloat(dr_groundspeed_);
    c.engineRpm   = readFloatArrayElem(dr_engineRpm_, 0);
    c.alphaDeg    = readFloat(dr_alpha_);
    return c;
}

float DataRefManager::readFloat(XPLMDataRef dr, float def) {
    return dr ? XPLMGetDataf(dr) : def;
}

int DataRefManager::readInt(XPLMDataRef dr, int def) {
    return dr ? XPLMGetDatai(dr) : def;
}

float DataRefManager::readFloatArrayElem(XPLMDataRef dr, int index) {
    if (!dr) return 0.0f;
    float v = 0.0f;
    XPLMGetDatavf(dr, &v, index, 1);
    return v;
}
```

- [ ] **Step 4: Add the new files to `MotionProviderPlugin/CMakeLists.txt`**

In the `set(SOURCES ...)` block, add `src/DataRefManager.cpp`. In the `set(HEADERS ...)` block, add `src/DataRefManager.h` and `src/MotionCues.h`. After the edit the two blocks read exactly:

```cmake
set(SOURCES
    src/Plugin.cpp
    src/MotionProvider.cpp
    src/StatusWindow.cpp
    src/ConfigUtils.cpp
    src/DataRefManager.cpp
)
set(HEADERS
    src/MotionProvider.h
    src/StatusWindow.h
    src/ConfigUtils.h
    src/DataRefManager.h
    src/MotionCues.h
)
```

- [ ] **Step 5: Commit**

Do NOT build (sandbox-blocked). DataRefManager is not wired to anything yet; it compiles standalone.

```bash
git add MotionProviderPlugin/src/MotionCues.h MotionProviderPlugin/src/DataRefManager.h MotionProviderPlugin/src/DataRefManager.cpp MotionProviderPlugin/CMakeLists.txt
git commit -m "feat(motion): Phase 1 Task 1 - MotionCues snapshot + DataRefManager"
```

(git prints benign `.config/git/ignore` / `maintenance` warnings; confirm with `git log --oneline -1`.)

---

### Task 2: Wire sampling into MotionProvider + display cues in StatusWindow

**Files:**
- Modify: `MotionProviderPlugin/src/MotionProvider.h`
- Modify: `MotionProviderPlugin/src/MotionProvider.cpp`
- Modify: `MotionProviderPlugin/src/StatusWindow.h`
- Modify: `MotionProviderPlugin/src/StatusWindow.cpp`

**Interfaces:**
- Consumes: `DataRefManager` (`initialize/onAircraftLoaded/sample`) and `MotionCues` from Task 1; the existing `MotionProvider`/`StatusWindow` from Phase 0.
- Produces: `StatusWindow::update` now takes `const MotionCues&`. The window displays the live cue snapshot. This is the Phase 1 deliverable.

- [ ] **Step 1: Update `MotionProvider.h` — own DataRefManager, hold latest cues**

The Phase 0 header forward-declares `StatusWindow` and owns it. Add a `DataRefManager` (forward-declared, owned via `unique_ptr`) and a `MotionCues latestCues_` value member (needs the full definition, so include `MotionCues.h`). Replace the entire file with:

```cpp
#pragma once
#include <memory>
#include "MotionCues.h"

class StatusWindow;
class DataRefManager;

class MotionProvider {
public:
    MotionProvider();
    ~MotionProvider();

    MotionProvider(const MotionProvider&) = delete;
    MotionProvider& operator=(const MotionProvider&) = delete;

    // Called from Plugin.cpp (X-Plane ABI thread).
    bool initialize();
    void shutdown();
    void onFlightLoopTick(float elapsedSec);
    void onAircraftLoaded();

private:
    std::unique_ptr<StatusWindow> statusWindow_;
    std::unique_ptr<DataRefManager> dataRefs_;

    // Most recent sampled snapshot (updated every 60 Hz tick).
    MotionCues latestCues_;

    // Status window refresh accumulator (~1 Hz), independent of the 60 Hz tick.
    float statusAccumSec_ = 0.0f;
};
```

- [ ] **Step 2: Update `MotionProvider.cpp` — create, resolve, sample, push**

Replace the entire file with:

```cpp
#include "MotionProvider.h"
#include "StatusWindow.h"
#include "DataRefManager.h"
#include "XPLMUtilities.h"

MotionProvider::MotionProvider() = default;
MotionProvider::~MotionProvider() = default;

bool MotionProvider::initialize() {
    dataRefs_ = std::make_unique<DataRefManager>();
    dataRefs_->initialize();

    statusWindow_ = std::make_unique<StatusWindow>();
    statusWindow_->initialize();

    XPLMDebugString("MotionProvider: initialized\n");
    return true;
}

void MotionProvider::shutdown() {
    if (statusWindow_) {
        statusWindow_->destroy();
        statusWindow_.reset();
    }
    dataRefs_.reset();
}

void MotionProvider::onFlightLoopTick(float elapsedSec) {
    // Sample the flight-model inputs every tick (later phases feed the washout
    // filter from this); the window only needs it at ~1 Hz.
    if (dataRefs_) {
        latestCues_ = dataRefs_->sample();
    }

    statusAccumSec_ += elapsedSec;
    if (statusAccumSec_ >= 1.0f) {
        statusAccumSec_ = 0.0f;
        if (statusWindow_) {
            statusWindow_->update(latestCues_);
        }
    }
}

void MotionProvider::onAircraftLoaded() {
    if (dataRefs_) {
        dataRefs_->onAircraftLoaded();
    }
}
```

- [ ] **Step 3: Update `StatusWindow.h` — take a MotionCues, store it**

Change `update()` to `update(const MotionCues&)`, add a `MotionCues cues_` member, and include `MotionCues.h`. Replace the entire file with:

```cpp
#pragma once
#include "XPLMDisplay.h"
#include "XPLMMenus.h"
#include "XPLMGraphics.h"
#include "MotionCues.h"
#include <string>

class StatusWindow {
public:
    StatusWindow();
    ~StatusWindow();

    StatusWindow(const StatusWindow&) = delete;
    StatusWindow& operator=(const StatusWindow&) = delete;

    void initialize();
    void destroy();
    void setVisible(bool visible);
    bool isVisible() const;

    // Refresh the displayed cue snapshot (called ~1 Hz).
    void update(const MotionCues& cues);

private:
    static void drawCallback(XPLMWindowID inWindowID, void* inRefcon);
    static void keyCallback(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags,
                            char inVirtualKey, void* inRefcon, int losingFocus);
    static void menuCallback(void* inMenuRef, void* inItemRef);

    void draw();
    void drawString(int x, int y, const std::string& text, float r, float g, float b);

    XPLMWindowID windowId_;
    int menuItemIdx_;
    XPLMMenuID pluginMenuId_;
    bool lastKnownVisible_;

    MotionCues cues_;
};
```

- [ ] **Step 4: Update `StatusWindow.cpp` — store cues, render them, widen window**

Four edits to the Phase 0 file:

(a) Add `#include <cstdio>` after the existing includes at the top:

```cpp
#include "StatusWindow.h"
#include "ConfigUtils.h"
#include "XPLMUtilities.h"
#include <cstdio>
```

(b) In `initialize()`, widen the window to fit the cue lines — change `params.bottom = 380;` to:

```cpp
    params.bottom = 300;
```

(c) Replace the `update()` body so it stores the snapshot (keeping the existing native-close-button visibility sync):

```cpp
void StatusWindow::update(const MotionCues& cues) {
    cues_ = cues;

    // Keep persisted visibility in sync with the native close button
    // (round-rectangle decoration has no callback of its own).
    bool nowVisible = isVisible();
    if (nowVisible != lastKnownVisible_) {
        lastKnownVisible_ = nowVisible;
        saveStatusWindowVisible(nowVisible);
    }
}
```

(d) Replace the `draw()` body so it renders the cue lines below the title:

```cpp
void StatusWindow::draw() {
    if (!windowId_) return;
    int left, top, right, bottom;
    XPLMGetWindowGeometry(windowId_, &left, &top, &right, &bottom);
    XPLMDrawTranslucentDarkBox(left, top, right, bottom);

    int x = left + 10;
    int y = top - 20;
    char buf[128];

    drawString(x, y, "Motion Provider v0.2 (Phase 1)", 0.8f, 1.0f, 0.8f);
    y -= 20;

    std::snprintf(buf, sizeof(buf), "Spec force (g)   surge %+.2f  sway %+.2f  heave %+.2f",
                  cues_.surgeG, cues_.swayG, cues_.heaveG);
    drawString(x, y, buf, 0.9f, 0.9f, 0.9f); y -= 16;

    std::snprintf(buf, sizeof(buf), "Rate (deg/s)     roll %+.1f  pitch %+.1f  yaw %+.1f",
                  cues_.rollRate, cues_.pitchRate, cues_.yawRate);
    drawString(x, y, buf, 0.9f, 0.9f, 0.9f); y -= 16;

    std::snprintf(buf, sizeof(buf), "Attitude (deg)   pitch %+.1f  roll %+.1f",
                  cues_.pitchDeg, cues_.rollDeg);
    drawString(x, y, buf, 0.9f, 0.9f, 0.9f); y -= 16;

    std::snprintf(buf, sizeof(buf), "Ground %s   GS %.1f m/s",
                  cues_.onGround ? "YES" : "no ", cues_.groundspeed);
    drawString(x, y, buf, 0.8f, 0.8f, 0.8f); y -= 16;

    std::snprintf(buf, sizeof(buf), "Eng RPM %.0f   AoA %+.1f deg",
                  cues_.engineRpm, cues_.alphaDeg);
    drawString(x, y, buf, 0.8f, 0.8f, 0.8f); y -= 16;
}
```

- [ ] **Step 5: Commit**

Do NOT build (sandbox-blocked). Phase 1 is now complete pending the user's manual build + X-Plane load.

```bash
git add MotionProviderPlugin/src/MotionProvider.h MotionProviderPlugin/src/MotionProvider.cpp MotionProviderPlugin/src/StatusWindow.h MotionProviderPlugin/src/StatusWindow.cpp
git commit -m "feat(motion): Phase 1 Task 2 - sample cues per tick, display live in status window"
```

(git prints benign warnings; confirm with `git log --oneline -1`.)

- [ ] **Step 6: Manual verification (user, on their machine)**

`cd MotionProviderPlugin && ./build-macos.sh`, load in X-Plane 12, open the status window. Confirm: heave ≈ +1.0 g in level flight, pitch/roll attitude track the aircraft, rates move during maneuvers, `Ground YES` on the runway and groundspeed matches, engine RPM non-zero with engine running, AoA reasonable. Values updating ~1 Hz.

---

## Self-Review

- **Spec coverage:** spec §4 Phase 1 ("set up datarefs and listen; display some in the status window for debugging") and §8 (the proposed cue set) are implemented — `DataRefManager` resolves/samples exactly the §8 datarefs; `StatusWindow` displays all twelve. Re-resolve on aircraft change is covered via `MotionProvider::onAircraftLoaded → dataRefs_->onAircraftLoaded()`, wired to `XPLM_MSG_PLANE_LOADED` in the Phase 0 `Plugin.cpp`.
- **Placeholder scan:** no TBD/TODO; all code is complete. Dataref paths are concrete. The only deferred item is the user's live-value confirmation, which is the phase's stated purpose, not a plan gap.
- **Type consistency:** `MotionCues` field names used in `DataRefManager::sample()` (Task 1) match those rendered in `StatusWindow::draw()` and stored via `update(const MotionCues&)` (Task 2). `StatusWindow::update` signature change (parameterless → `const MotionCues&`) is applied in both the header (Task 2 Step 3) and the caller `MotionProvider::onFlightLoopTick` (Task 2 Step 2) — no dangling old-signature call. `DataRefManager` owned by `unique_ptr` with forward declaration in `MotionProvider.h` and full include in `.cpp`, matching the same safe incomplete-type pattern used for `StatusWindow` in Phase 0.
- **Constraint check:** all `XPLMFindDataRef`/`XPLMGetData*` calls live only in `DataRefManager.cpp`; `Plugin.cpp` untouched; null-checked readers give 0/default fallback; 60 Hz sample / 1 Hz display split preserved.
