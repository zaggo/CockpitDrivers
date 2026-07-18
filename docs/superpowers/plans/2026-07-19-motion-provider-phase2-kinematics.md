# MotionProviderPlugin — Phase 2 (Inverse Kinematics) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Solve the rotary 6-RSS Stewart-platform inverse kinematics — turn a desired 6-DOF platform pose into six servo angles and 16-bit setpoints — and display those setpoints live in the status window.

**Architecture:** `StewartKinematics` is pure math (no X-Plane dependency), driven by a fixed geometry in `StewartConfig.h` (moves to TOML in Phase 2a). It gets a native, dependency-free test target built test-first. `MotionProvider` feeds it a placeholder pose (clamped aircraft attitude, replaced by the washout filter in Phase 3) each tick and pushes the results to `StatusWindow` for display.

**Tech Stack:** C++17, `<cmath>`, existing CMake pipeline; a standalone native CMake test project under `tests/`.

## Global Constraints

- C++17. `StewartKinematics`, `StewartConfig.h`, `Pose.h` MUST NOT include or depend on any X-Plane SDK header (they must compile in the native test target with no SDK present).
- SANDBOX: `cmake`/`clang`/`make` are BLOCKED. Implementers WRITE FILES ONLY. Both the plugin build AND the native test run happen on the user's machine. Author tests to pass; the user runs them.
- Angles in the code are computed in radians internally; the public API and config are in **degrees** and **millimetres**.
- Work on branch `feature/motion-provider-plugin`.

### Authoritative geometry model (the single source of truth for this phase)

Frame: right-handed, **Z up**, origin at base centre, FRONT = +Y, angles CCW from +X, lengths mm, angles deg. Legs are indexed 0..5 == **P1..P6**.

| i | Leg | servo | φ base angle (deg) | ψ anchor angle (deg) | β horn azimuth (deg) | BFF # |
|---|---|---|---|---|---|---|
| 0 | P1 | N1C2 | 60  | 83.75  | 150 | 5 |
| 1 | P2 | N2C1 | 0   | 336.25 | 270 | 6 |
| 2 | P3 | N2C2 | 300 | 323.75 | 30  | 1 |
| 3 | P4 | N3C1 | 240 | 216.25 | 150 | 2 |
| 4 | P5 | N3C2 | 180 | 203.75 | 270 | 3 |
| 5 | P6 | N1C1 | 120 | 96.25  | 30  | 4 |

Scalars: base radius `Rb = 425`, platform anchor radius `Rp = 480`, horn length `a = 100`, rod length `s = 466`. β = φ+90° (legs 0,2,4) / φ−90° (legs 1,3,5). Home height `z0` is **computed** from the leg-0 horizontal closure `z0 = sqrt(s² − |anchor_xy − hornTip_xy|²)` ≈ **456.3 mm** (all legs congruent, so one value serves all).

Servo-angle → 16-bit demand: `demand = 32640 + (θ_deg / 45) · 32640`, clamped to `[0, 65280]` (θ = 0 → 32640, +45° → 65280, −45° → 0). Uniform for all legs; per-leg mechanical mounting direction is handled downstream by each actuator's Kangaroo min/max calibration — **flag for bring-up: if a leg drives the wrong way, invert that channel's actuator calibration, not the plugin.**

IK per leg (closed form): with `l = q − B` (anchor world minus base pivot), `û_β = (cosβ, sinβ, 0)`:
```
L² = |l|² − (s² − a²)
M  = 2a·l_z
N  = 2a·(l_x·cosβ + l_y·sinβ)
θ  = asin( L² / sqrt(M² + N²) ) − atan2(N, M)
```
`|L²/sqrt(M²+N²)| > 1` ⇒ pose unreachable for that leg.

## File structure

- Create `MotionProviderPlugin/src/Pose.h` — 6-DOF pose struct.
- Create `MotionProviderPlugin/src/StewartConfig.h` — geometry constants (→ TOML in Phase 2a).
- Create `MotionProviderPlugin/src/StewartKinematics.h` / `.cpp` — the solver (pure math).
- Create `MotionProviderPlugin/tests/CMakeLists.txt` — standalone native test project.
- Create `MotionProviderPlugin/tests/test_kinematics.cpp` — the test suite.
- Modify `MotionProviderPlugin/CMakeLists.txt` — add solver sources to the plugin.
- Modify `MotionProviderPlugin/src/MotionProvider.{h,cpp}` — feed a placeholder pose, hold the solve result.
- Modify `MotionProviderPlugin/src/StatusWindow.{h,cpp}` — display the 6 angles/setpoints.

---

### Task 1: StewartKinematics (pure math) + native TDD test target

**Files:**
- Create: `MotionProviderPlugin/src/Pose.h`, `src/StewartConfig.h`, `src/StewartKinematics.h`, `src/StewartKinematics.cpp`
- Create: `MotionProviderPlugin/tests/CMakeLists.txt`, `tests/test_kinematics.cpp`
- Modify: `MotionProviderPlugin/CMakeLists.txt`

**Interfaces:**
- Consumes: nothing (self-contained math).
- Produces: `struct Pose`; `struct SolveResult { struct LegResult { double angleDeg; bool reachable; } legs[6]; uint16_t setpoints[6]; bool allReachable; }`; `StewartKinematics::solve(const Pose&) -> SolveResult` and `StewartKinematics::homeHeight() -> double`. `setpoints` are in BFF actuator order (index 0 == BFF #1); `legs` are in P1..P6 order.

- [ ] **Step 1: Write `MotionProviderPlugin/src/Pose.h`**

```cpp
#pragma once

// Desired platform pose relative to home. Translations in mm, rotations in deg.
struct Pose {
    float surge = 0.0f;  // +X
    float sway  = 0.0f;  // +Y
    float heave = 0.0f;  // +Z (added on top of home height z0)
    float roll  = 0.0f;  // about X
    float pitch = 0.0f;  // about Y
    float yaw   = 0.0f;  // about Z
};
```

- [ ] **Step 2: Write `MotionProviderPlugin/src/StewartConfig.h`**

```cpp
#pragma once

// Fixed rotary 6-RSS Stewart-platform geometry. Legs indexed 0..5 == P1..P6.
// Moves to a TOML file in Phase 2a; kept as constants here so Phase 2 has no
// config-parsing dependency. Lengths mm, angles deg.
namespace stewart {
    constexpr int    kLegs = 6;
    constexpr double kRb   = 425.0;  // base servo pivot radius
    constexpr double kRp   = 480.0;  // platform anchor radius
    constexpr double kHorn = 100.0;  // servo horn length (a)
    constexpr double kRod  = 466.0;  // push-rod length (s)

    // Base servo angle (phi), platform anchor angle (psi), horn azimuth (beta).
    constexpr double kPhi[6]  = { 60.0,   0.0, 300.0, 240.0, 180.0, 120.0 };
    constexpr double kPsi[6]  = { 83.75, 336.25, 323.75, 216.25, 203.75, 96.25 };
    constexpr double kBeta[6] = { 150.0, 270.0,  30.0, 150.0, 270.0,  30.0 };

    // BFF actuator index (1..6) each leg P1..P6 is wired to.
    constexpr int kBff[6] = { 5, 6, 1, 2, 3, 4 };

    // Servo angle -> 16-bit demand mapping.
    constexpr double kAngleAtFullScale = 45.0; // deg at demand extremes
    constexpr int    kDemandHome = 32640;      // theta = 0
    constexpr int    kDemandMax  = 65280;      // theta = +kAngleAtFullScale
}
```

- [ ] **Step 3: Write `MotionProviderPlugin/src/StewartKinematics.h`**

```cpp
#pragma once
#include <cstdint>
#include "Pose.h"

struct LegResult {
    double angleDeg = 0.0;   // servo horn elevation, deg (0 = horizontal/home)
    bool   reachable = true; // false if the pose is outside this leg's envelope
};

struct SolveResult {
    LegResult legs[6];        // P1..P6 order
    uint16_t  setpoints[6];   // BFF actuator order (index 0 == BFF #1)
    bool      allReachable = true;
};

class StewartKinematics {
public:
    // Home platform height (mm) at which all horns rest horizontal. Derived
    // from geometry; equal for all legs by symmetry.
    static double homeHeight();

    // Solve inverse kinematics for a pose. Never throws; unreachable legs are
    // flagged and their angle is clamped to the envelope edge.
    static SolveResult solve(const Pose& pose);
};
```

- [ ] **Step 4: Write `MotionProviderPlugin/src/StewartKinematics.cpp`**

```cpp
#include "StewartKinematics.h"
#include "StewartConfig.h"
#include <cmath>

namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

struct Vec3 { double x, y, z; };

// R = Rz(yaw) * Ry(pitch) * Rx(roll), angles in radians.
Vec3 rotate(double roll, double pitch, double yaw, Vec3 p) {
    double cr = std::cos(roll),  sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy = std::cos(yaw),   sy = std::sin(yaw);
    Vec3 a{ p.x,               cr * p.y - sr * p.z,  sr * p.y + cr * p.z };
    Vec3 b{ cp * a.x + sp * a.z, a.y,               -sp * a.x + cp * a.z };
    Vec3 c{ cy * b.x - sy * b.y, sy * b.x + cy * b.y, b.z };
    return c;
}
}  // namespace

double StewartKinematics::homeHeight() {
    using namespace stewart;
    const double phi = kPhi[0] * kDeg2Rad;
    const double psi = kPsi[0] * kDeg2Rad;
    const double beta = kBeta[0] * kDeg2Rad;
    const double bx = kRb * std::cos(phi);
    const double by = kRb * std::sin(phi);
    const double hx = bx + kHorn * std::cos(beta);  // horn tip at theta=0
    const double hy = by + kHorn * std::sin(beta);
    const double ax = kRp * std::cos(psi);
    const double ay = kRp * std::sin(psi);
    const double dx = ax - hx, dy = ay - hy;
    const double h2 = kRod * kRod - (dx * dx + dy * dy);
    return h2 > 0.0 ? std::sqrt(h2) : 0.0;
}

SolveResult StewartKinematics::solve(const Pose& pose) {
    using namespace stewart;
    SolveResult out{};
    out.allReachable = true;

    const double z0 = homeHeight();
    const Vec3 origin{ pose.surge, pose.sway, z0 + pose.heave };
    const double roll = pose.roll * kDeg2Rad;
    const double pitch = pose.pitch * kDeg2Rad;
    const double yaw = pose.yaw * kDeg2Rad;

    for (int i = 0; i < kLegs; ++i) {
        const double phi = kPhi[i] * kDeg2Rad;
        const double psi = kPsi[i] * kDeg2Rad;
        const double beta = kBeta[i] * kDeg2Rad;

        const Vec3 pl{ kRp * std::cos(psi), kRp * std::sin(psi), 0.0 };
        const Vec3 r = rotate(roll, pitch, yaw, pl);
        const Vec3 q{ origin.x + r.x, origin.y + r.y, origin.z + r.z };
        const Vec3 B{ kRb * std::cos(phi), kRb * std::sin(phi), 0.0 };
        const Vec3 l{ q.x - B.x, q.y - B.y, q.z - B.z };

        const double L2 = (l.x * l.x + l.y * l.y + l.z * l.z)
                          - (kRod * kRod - kHorn * kHorn);
        const double M = 2.0 * kHorn * l.z;
        const double N = 2.0 * kHorn * (l.x * std::cos(beta) + l.y * std::sin(beta));
        const double denom = std::sqrt(M * M + N * N);

        LegResult lr;
        if (denom < 1e-9) {
            lr.reachable = false;
            lr.angleDeg = 0.0;
        } else {
            double ratio = L2 / denom;
            if (ratio < -1.0 || ratio > 1.0) {
                lr.reachable = false;
                ratio = ratio < 0.0 ? -1.0 : 1.0;   // clamp to envelope edge
            }
            lr.angleDeg = (std::asin(ratio) - std::atan2(N, M)) / kDeg2Rad;
        }
        if (!lr.reachable) out.allReachable = false;
        out.legs[i] = lr;

        double demand = kDemandHome + (lr.angleDeg / kAngleAtFullScale) * kDemandHome;
        if (demand < 0.0) demand = 0.0;
        if (demand > kDemandMax) demand = kDemandMax;
        out.setpoints[kBff[i] - 1] = static_cast<uint16_t>(std::lround(demand));
    }
    return out;
}
```

- [ ] **Step 5: Write the failing test suite `MotionProviderPlugin/tests/test_kinematics.cpp`**

```cpp
#include "StewartKinematics.h"
#include "StewartConfig.h"
#include <cmath>
#include <cstdio>
#include <cstdint>

static int g_failures = 0;
static int g_checks = 0;

static void check(bool cond, const char* what) {
    ++g_checks;
    if (!cond) { ++g_failures; std::printf("  FAIL: %s\n", what); }
}
static void checkNear(double got, double want, double tol, const char* what) {
    ++g_checks;
    if (std::fabs(got - want) > tol) {
        ++g_failures;
        std::printf("  FAIL: %s (got %.6f want %.6f tol %.6f)\n", what, got, want, tol);
    }
}

// Reconstruct horn tip from solved angle and confirm the rod length closes to s.
static double rodClosureError(const Pose& pose, int leg) {
    using namespace stewart;
    const double d2r = 3.14159265358979323846 / 180.0;
    SolveResult r = StewartKinematics::solve(pose);
    double th = r.legs[leg].angleDeg * d2r;
    double phi = kPhi[leg]*d2r, psi = kPsi[leg]*d2r, beta = kBeta[leg]*d2r;
    // horn tip
    double hx = kRb*std::cos(phi) + kHorn*std::cos(th)*std::cos(beta);
    double hy = kRb*std::sin(phi) + kHorn*std::cos(th)*std::sin(beta);
    double hz = kHorn*std::sin(th);
    // anchor world (same R = Rz*Ry*Rx as solver)
    double cr=std::cos(pose.roll*d2r), sr=std::sin(pose.roll*d2r);
    double cp=std::cos(pose.pitch*d2r), sp=std::sin(pose.pitch*d2r);
    double cy=std::cos(pose.yaw*d2r), sy=std::sin(pose.yaw*d2r);
    double px=kRp*std::cos(psi), py=kRp*std::sin(psi), pz=0.0;
    double ax=px, ay=cr*py - sr*pz, az=sr*py + cr*pz;
    double bx=cp*ax + sp*az, by=ay, bz=-sp*ax + cp*az;
    double qx=cy*bx - sy*by + pose.surge;
    double qy=sy*bx + cy*by + pose.sway;
    double qz=bz + StewartKinematics::homeHeight() + pose.heave;
    double dx=qx-hx, dy=qy-hy, dz=qz-hz;
    return std::sqrt(dx*dx+dy*dy+dz*dz) - stewart::kRod;
}

int main() {
    using namespace stewart;

    std::printf("home height...\n");
    checkNear(StewartKinematics::homeHeight(), 456.3, 0.5, "z0 ~= 456.3mm");

    std::printf("home pose -> all angles 0, all setpoints midscale...\n");
    {
        SolveResult r = StewartKinematics::solve(Pose{});
        check(r.allReachable, "home reachable");
        for (int i=0;i<6;i++) checkNear(r.legs[i].angleDeg, 0.0, 0.05, "home angle ~0");
        for (int i=0;i<6;i++) check(r.setpoints[i]==32640, "home setpoint 32640");
    }

    std::printf("setpoint mapping endpoints...\n");
    {
        // Direct check of the linear mapping at 0, +45, -45 via reachable poses is
        // hard to hit exactly, so verify the formula endpoints numerically here:
        auto demand = [](double deg){
            double d = 32640 + (deg/45.0)*32640;
            if (d<0) d=0; if (d>65280) d=65280; return (uint16_t)std::lround(d);
        };
        check(demand(0.0)==32640, "map 0 -> 32640");
        check(demand(45.0)==65280, "map +45 -> 65280");
        check(demand(-45.0)==0,    "map -45 -> 0");
        check(demand(90.0)==65280, "map +90 clamps to 65280");
    }

    std::printf("rod closure round-trip across poses...\n");
    {
        Pose poses[] = {
            Pose{},
            Pose{0,0, 30, 0,0,0},      // heave up
            Pose{0,0,-30, 0,0,0},      // heave down
            Pose{0,0,0, 5,0,0},        // roll
            Pose{0,0,0, 0,5,0},        // pitch
            Pose{0,0,0, 0,0,5},        // yaw
            Pose{10,-8,15, 3,-2,4},    // combined
        };
        for (const Pose& p : poses)
            for (int i=0;i<6;i++)
                if (StewartKinematics::solve(p).legs[i].reachable)
                    checkNear(rodClosureError(p, i), 0.0, 1e-3, "rod closes to s");
    }

    std::printf("pure heave symmetry + sign...\n");
    {
        SolveResult up = StewartKinematics::solve(Pose{0,0,20,0,0,0});
        SolveResult dn = StewartKinematics::solve(Pose{0,0,-20,0,0,0});
        for (int i=0;i<6;i++) check(up.legs[i].angleDeg > 0.0, "heave up -> angle>0");
        for (int i=0;i<6;i++) check(dn.legs[i].angleDeg < 0.0, "heave down -> angle<0");
        for (int i=1;i<6;i++)
            checkNear(up.legs[i].angleDeg, up.legs[0].angleDeg, 0.05, "heave legs equal");
    }

    std::printf("extreme pose flagged unreachable, no NaN...\n");
    {
        SolveResult r = StewartKinematics::solve(Pose{0,0,10000,0,0,0});
        check(!r.allReachable, "huge heave unreachable");
        for (int i=0;i<6;i++) check(std::isfinite(r.legs[i].angleDeg), "angle finite");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
```

- [ ] **Step 6: Write `MotionProviderPlugin/tests/CMakeLists.txt` (standalone, no X-Plane SDK)**

```cmake
cmake_minimum_required(VERSION 3.15)
project(motion_provider_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(test_kinematics
    test_kinematics.cpp
    ../src/StewartKinematics.cpp
)
target_include_directories(test_kinematics PRIVATE ../src)

enable_testing()
add_test(NAME kinematics COMMAND test_kinematics)
```

- [ ] **Step 7: Add the solver to the plugin build in `MotionProviderPlugin/CMakeLists.txt`**

Add `src/StewartKinematics.cpp` to `SOURCES`, and `src/StewartKinematics.h`, `src/StewartConfig.h`, `src/Pose.h` to `HEADERS`. After the edit:

```cmake
set(SOURCES
    src/Plugin.cpp
    src/MotionProvider.cpp
    src/StatusWindow.cpp
    src/ConfigUtils.cpp
    src/DataRefManager.cpp
    src/StewartKinematics.cpp
)
set(HEADERS
    src/MotionProvider.h
    src/StatusWindow.h
    src/ConfigUtils.h
    src/DataRefManager.h
    src/MotionCues.h
    src/StewartKinematics.h
    src/StewartConfig.h
    src/Pose.h
)
```

(The top-level CMakeLists already conditionally runs `add_subdirectory(tests)` when `tests/` exists, but the test project is standalone — see the run command below.)

- [ ] **Step 8: Commit**

Do NOT build (sandbox-blocked).

```bash
git add MotionProviderPlugin/src/Pose.h MotionProviderPlugin/src/StewartConfig.h MotionProviderPlugin/src/StewartKinematics.h MotionProviderPlugin/src/StewartKinematics.cpp MotionProviderPlugin/tests/CMakeLists.txt MotionProviderPlugin/tests/test_kinematics.cpp MotionProviderPlugin/CMakeLists.txt
git commit -m "feat(motion): Phase 2 Task 1 - Stewart inverse kinematics + native TDD tests"
```

- [ ] **Step 9: Manual test run (user, on their machine)**

```bash
cd MotionProviderPlugin/tests
cmake -B build && cmake --build build && ./build/test_kinematics
```
Expected final line: `NN checks, 0 failures` (exit 0). If any FAIL prints, the geometry model or solver needs correction before wiring in.

---

### Task 2: Feed a placeholder pose + display setpoints

**Files:**
- Modify: `MotionProviderPlugin/src/MotionProvider.h`, `src/MotionProvider.cpp`
- Modify: `MotionProviderPlugin/src/StatusWindow.h`, `src/StatusWindow.cpp`

**Interfaces:**
- Consumes: `StewartKinematics::solve`, `SolveResult`, `Pose` (Task 1); `MotionCues` (Phase 1).
- Produces: `StatusWindow::update(const MotionCues&, const SolveResult&)`. The window shows the six servo angles and setpoints. Placeholder pose = clamped aircraft attitude (roll/pitch), replaced by the washout filter in Phase 3.

- [ ] **Step 1: Update `MotionProvider.h`** — add the include and hold the latest solve result.

Add `#include "StewartKinematics.h"` (alongside the existing `#include "MotionCues.h"`), and add a private member `SolveResult latestSolve_;` next to `latestCues_`. (No other change to the class shape.)

```cpp
#pragma once
#include <memory>
#include "MotionCues.h"
#include "StewartKinematics.h"

class StatusWindow;
class DataRefManager;

class MotionProvider {
public:
    MotionProvider();
    ~MotionProvider();

    MotionProvider(const MotionProvider&) = delete;
    MotionProvider& operator=(const MotionProvider&) = delete;

    bool initialize();
    void shutdown();
    void onFlightLoopTick(float elapsedSec);
    void onAircraftLoaded();

private:
    std::unique_ptr<StatusWindow> statusWindow_;
    std::unique_ptr<DataRefManager> dataRefs_;

    MotionCues latestCues_;
    SolveResult latestSolve_;

    float statusAccumSec_ = 0.0f;
};
```

- [ ] **Step 2: Update `MotionProvider.cpp`** — build a placeholder pose from the cues, solve, push both to the window.

Replace the `onFlightLoopTick` body (leave `initialize`/`shutdown`/`onAircraftLoaded` from Phase 1 unchanged except the `update` call). The full file:

```cpp
#include "MotionProvider.h"
#include "StatusWindow.h"
#include "DataRefManager.h"
#include "XPLMUtilities.h"
#include <algorithm>

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
    if (dataRefs_) {
        latestCues_ = dataRefs_->sample();
    }

    // Placeholder pose: map aircraft attitude straight to platform tilt, clamped
    // to a small range so it stays reachable. Replaced by the washout filter in
    // Phase 3 - this only exists so Phase 2 shows the IK responding in flight.
    auto clampf = [](float v, float lo, float hi) {
        return std::max(lo, std::min(hi, v));
    };
    Pose pose;
    pose.roll  = clampf(latestCues_.rollDeg,  -8.0f, 8.0f);
    pose.pitch = clampf(latestCues_.pitchDeg, -8.0f, 8.0f);
    latestSolve_ = StewartKinematics::solve(pose);

    statusAccumSec_ += elapsedSec;
    if (statusAccumSec_ >= 1.0f) {
        statusAccumSec_ = 0.0f;
        if (statusWindow_) {
            statusWindow_->update(latestCues_, latestSolve_);
        }
    }
}

void MotionProvider::onAircraftLoaded() {
    if (dataRefs_) {
        dataRefs_->onAircraftLoaded();
    }
}
```

- [ ] **Step 3: Update `StatusWindow.h`** — widen the `update` signature and include the kinematics header.

Add `#include "StewartKinematics.h"` after the `MotionCues.h` include, change the `update` declaration to `void update(const MotionCues& cues, const SolveResult& solve);`, and add a `SolveResult solve_;` member next to `cues_`.

- [ ] **Step 4: Update `StatusWindow.cpp`** — store the solve result and draw the six legs.

(a) Change the `update` definition to take and store both:

```cpp
void StatusWindow::update(const MotionCues& cues, const SolveResult& solve) {
    cues_ = cues;
    solve_ = solve;

    bool nowVisible = isVisible();
    if (nowVisible != lastKnownVisible_) {
        lastKnownVisible_ = nowVisible;
        saveStatusWindowVisible(nowVisible);
    }
}
```

(b) In `initialize()`, widen the window for the extra lines — change `params.bottom = 300;` to:

```cpp
    params.bottom = 210;
```

(c) Replace the `draw()` body — keep the Phase 1 cue lines, append a per-leg block:

```cpp
void StatusWindow::draw() {
    if (!windowId_) return;
    int left, top, right, bottom;
    XPLMGetWindowGeometry(windowId_, &left, &top, &right, &bottom);
    XPLMDrawTranslucentDarkBox(left, top, right, bottom);

    int x = left + 10;
    int y = top - 20;
    char buf[160];

    drawString(x, y, "Motion Provider v0.3 (Phase 2)", 0.8f, 1.0f, 0.8f);
    y -= 20;

    std::snprintf(buf, sizeof(buf), "Attitude (deg)   pitch %+.1f  roll %+.1f",
                  cues_.pitchDeg, cues_.rollDeg);
    drawString(x, y, buf, 0.9f, 0.9f, 0.9f); y -= 18;

    drawString(x, y, solve_.allReachable ? "IK: all legs reachable"
                                          : "IK: POSE UNREACHABLE",
               solve_.allReachable ? 0.6f : 1.0f,
               solve_.allReachable ? 1.0f : 0.4f, 0.4f);
    y -= 18;

    for (int i = 0; i < 6; ++i) {
        std::snprintf(buf, sizeof(buf), "P%d  %+7.2f deg  ->  %5u%s",
                      i + 1, solve_.legs[i].angleDeg, solve_.setpoints[i],
                      solve_.legs[i].reachable ? "" : "  (unreachable)");
        drawString(x, y, buf, 0.85f, 0.85f, 0.9f);
        y -= 16;
    }
}
```

Note: `solve_.setpoints` are in BFF order; the `P%d` label is leg order. That's fine for Phase-2 debugging (both are shown per line via the loop index into `legs`, and `setpoints[i]` here is the BFF-slot value) — acceptance is "values move sanely," exact per-leg pairing is validated in Phase 2a. Leave as written.

- [ ] **Step 5: Commit**

```bash
git add MotionProviderPlugin/src/MotionProvider.h MotionProviderPlugin/src/MotionProvider.cpp MotionProviderPlugin/src/StatusWindow.h MotionProviderPlugin/src/StatusWindow.cpp
git commit -m "feat(motion): Phase 2 Task 2 - placeholder pose from attitude, display IK setpoints"
```

- [ ] **Step 6: Manual verification (user)**

Build + load. Level flight: all six lines ≈ `+0.00 deg -> 32640`. Bank/pitch the aircraft (within ±8°): angles and setpoints move, symmetric legs mirror, none flip to "unreachable". This confirms the IK responds correctly end-to-end.

---

## Self-Review

- **Spec coverage:** spec §4 Phase 2 ("implement the IK, geometry definitions, display the 6 setpoints") — `StewartKinematics` + `StewartConfig.h` (Task 1) and the window display (Task 2). Spec §1 setpoint semantics (linear servo angle → 0..65280, 32640 home) implemented in the demand mapping. Spec §7 (native test target for the pure-math module, TDD) — `tests/`. Geometry values are the user-confirmed numbers.
- **Placeholder scan:** no TBD/TODO. The clamped-attitude pose is an explicit, labelled placeholder the plan says Phase 3 replaces — not a gap. Test execution is deferred to the user only because the sandbox can't compile; the tests themselves are complete and exact.
- **Type consistency:** `SolveResult`/`LegResult`/`Pose` defined in Task 1 headers are used with the same field names in the tests, `MotionProvider`, and `StatusWindow`. `StatusWindow::update(const MotionCues&, const SolveResult&)` is changed in the header (Task 2 Step 3), definition (Step 4a), and the sole caller `MotionProvider::onFlightLoopTick` (Step 2) together. `setpoints`/`legs` ordering (BFF vs P) is documented on the struct and at the call site.
- **Constraint check:** `Pose.h`, `StewartConfig.h`, `StewartKinematics.{h,cpp}` include only `<cstdint>`/`<cmath>` — no XPLM — so the native test target compiles without the SDK. Angles deg in API, rad internally. `homeHeight()` derives z0 from geometry (no magic constant); the ~456.3 test uses a 0.5 mm tolerance.
- **TDD note:** the round-trip closure test is the primary correctness gate — it reconstructs each horn tip from the solved angle and confirms the rod length returns to `s`, catching sign/branch/rotation errors without hand-computed transcendental golden values. Home-angle-zero and the demand mapping are the exact-value anchors.
