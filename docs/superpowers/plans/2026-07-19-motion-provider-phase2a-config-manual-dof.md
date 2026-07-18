# MotionProviderPlugin — Phase 2a (TOML config + manual DOF) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the platform geometry into a hot-reloadable TOML file, and add keyboard control to set the 6-DOF pose by hand so the IK and platform envelope can be explored one axis at a time.

**Architecture:** The compile-time geometry constants become a runtime `StewartGeometry` value; `StewartKinematics` becomes an instance built from it. `MotionConfig` loads a `StewartGeometry` from `~/.motionprovider.toml` (falling back to built-in defaults per missing key), parsed with the header-only `toml++`. The status window gains a "Reload config" button and keyboard DOF control; `MotionProvider` owns the manual-control state and picks manual pose vs. the attitude placeholder each tick.

**Tech Stack:** C++17, `toml++` (single-header, vendored), existing CMake pipeline + the native test target from Phase 2.

## Global Constraints

- C++17. `StewartGeometry`, `StewartKinematics`, `Pose`, `MotionConfig` MUST NOT include any X-Plane SDK header (they compile in the native test target).
- SANDBOX: `cmake`/`clang`/`make` are BLOCKED. Implementers WRITE FILES ONLY; the user builds and runs the native tests.
- **USER PREREQUISITE (before Task 2 builds):** download the `toml++` v3.4.0 single-header amalgamation `toml.hpp` and place it at `MotionProviderPlugin/third_party/tomlplusplus/toml.hpp`. It is not fetchable from the sandbox. Source: https://github.com/marzer/tomlplusplus `toml.hpp` release asset. It is git-ignored via a rule added in Task 2 (large vendored header; the user keeps a local copy) — OR commit it if preferred; the plan git-ignores it by default.
- Config file path: `~/.motionprovider.toml` (tuning config; separate from the flat `~/.motionprovider.cfg` that ConfigUtils uses for port/window state).
- Behavior parity: after the Task 1 refactor, `StewartGeometry::defaults()` must reproduce the exact Phase-2 numbers, so the existing kinematics tests keep passing unchanged in result.
- Work on branch `feature/motion-provider-plugin`.

## File structure

- Create `MotionProviderPlugin/src/StewartGeometry.h` — runtime geometry value + `defaults()`.
- Delete `MotionProviderPlugin/src/StewartConfig.h` — folded into `StewartGeometry` defaults.
- Modify `StewartKinematics.h/.cpp` — instance built from `StewartGeometry`, caches z0.
- Create `MotionProviderPlugin/src/MotionConfig.h/.cpp` — TOML → `StewartGeometry`.
- Create `MotionProviderPlugin/src/StatusData.h` — aggregate passed to the window.
- Modify `StatusWindow.h/.cpp` — `update(const StatusData&)`, reload button, DOF keys.
- Modify `MotionProvider.h/.cpp` — own kinematics instance + manual state; reload + key handling.
- Modify `CMakeLists.txt`, `tests/CMakeLists.txt`, `tests/test_kinematics.cpp`, and add `tests/test_config.cpp`.

---

### Task 1: Runtime geometry — `StewartGeometry` + instance-based `StewartKinematics`

Pure refactor; behavior identical, tests still pass.

**Files:**
- Create: `MotionProviderPlugin/src/StewartGeometry.h`
- Delete: `MotionProviderPlugin/src/StewartConfig.h`
- Modify: `src/StewartKinematics.h`, `src/StewartKinematics.cpp`, `tests/test_kinematics.cpp`, `CMakeLists.txt`, `src/MotionProvider.h`, `src/MotionProvider.cpp`

**Interfaces:**
- Produces: `struct StewartGeometry` (fields below) with `static StewartGeometry defaults()`; `StewartKinematics` constructed `explicit StewartKinematics(const StewartGeometry&)`, methods `homeHeight() const`, `solve(const Pose&) const`. `SolveResult`/`LegResult`/`Pose` unchanged.

- [ ] **Step 1: Create `MotionProviderPlugin/src/StewartGeometry.h`**

```cpp
#pragma once
#include <cstdint>

// Runtime rotary 6-RSS geometry. defaults() reproduces the confirmed rig.
// Legs indexed 0..5 == P1..P6. Lengths mm, angles deg.
struct StewartGeometry {
    double baseRadius     = 425.0;   // Rb: base servo pivot radius
    double platformRadius = 480.0;   // Rp: platform anchor radius
    double hornLength     = 100.0;   // a
    double rodLength      = 466.0;   // s

    double phiDeg[6]  = { 60.0,   0.0, 300.0, 240.0, 180.0, 120.0 };
    double psiDeg[6]  = { 83.75, 336.25, 323.75, 216.25, 203.75, 96.25 };
    double betaDeg[6] = { 150.0, 270.0,  30.0, 150.0, 270.0,  30.0 };
    int    bff[6]     = { 5, 6, 1, 2, 3, 4 };

    double angleAtFullScale = 45.0;  // deg mapped to demand extremes
    int    demandHome = 32640;
    int    demandMax  = 65280;

    static StewartGeometry defaults() { return StewartGeometry{}; }
};
```

- [ ] **Step 2: Replace `MotionProviderPlugin/src/StewartKinematics.h`**

```cpp
#pragma once
#include <cstdint>
#include "Pose.h"
#include "StewartGeometry.h"

struct LegResult {
    double angleDeg = 0.0;
    bool   reachable = true;
};

struct SolveResult {
    LegResult legs[6];
    uint16_t  setpoints[6];   // BFF actuator order (index 0 == BFF #1)
    bool      allReachable = true;
};

class StewartKinematics {
public:
    explicit StewartKinematics(const StewartGeometry& geo);

    double homeHeight() const { return z0_; }      // mm
    SolveResult solve(const Pose& pose) const;
    const StewartGeometry& geometry() const { return geo_; }

private:
    StewartGeometry geo_;
    double z0_ = 0.0;
};
```

- [ ] **Step 3: Replace `MotionProviderPlugin/src/StewartKinematics.cpp`**

```cpp
#include "StewartKinematics.h"
#include <cmath>

namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
struct Vec3 { double x, y, z; };

Vec3 rotate(double roll, double pitch, double yaw, Vec3 p) {
    double cr = std::cos(roll),  sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy = std::cos(yaw),   sy = std::sin(yaw);
    Vec3 a{ p.x,                cr * p.y - sr * p.z,  sr * p.y + cr * p.z };
    Vec3 b{ cp * a.x + sp * a.z, a.y,                -sp * a.x + cp * a.z };
    Vec3 c{ cy * b.x - sy * b.y, sy * b.x + cy * b.y, b.z };
    return c;
}
}  // namespace

StewartKinematics::StewartKinematics(const StewartGeometry& geo) : geo_(geo) {
    // Home height from leg-0 horizontal closure (all legs congruent).
    const double phi = geo_.phiDeg[0] * kDeg2Rad;
    const double psi = geo_.psiDeg[0] * kDeg2Rad;
    const double beta = geo_.betaDeg[0] * kDeg2Rad;
    const double hx = geo_.baseRadius * std::cos(phi) + geo_.hornLength * std::cos(beta);
    const double hy = geo_.baseRadius * std::sin(phi) + geo_.hornLength * std::sin(beta);
    const double ax = geo_.platformRadius * std::cos(psi);
    const double ay = geo_.platformRadius * std::sin(psi);
    const double dx = ax - hx, dy = ay - hy;
    const double h2 = geo_.rodLength * geo_.rodLength - (dx * dx + dy * dy);
    z0_ = h2 > 0.0 ? std::sqrt(h2) : 0.0;
}

SolveResult StewartKinematics::solve(const Pose& pose) const {
    SolveResult out{};
    out.allReachable = true;

    const Vec3 origin{ pose.surge, pose.sway, z0_ + pose.heave };
    const double roll = pose.roll * kDeg2Rad;
    const double pitch = pose.pitch * kDeg2Rad;
    const double yaw = pose.yaw * kDeg2Rad;
    const double s2ma2 = geo_.rodLength * geo_.rodLength - geo_.hornLength * geo_.hornLength;

    for (int i = 0; i < 6; ++i) {
        const double phi = geo_.phiDeg[i] * kDeg2Rad;
        const double psi = geo_.psiDeg[i] * kDeg2Rad;
        const double beta = geo_.betaDeg[i] * kDeg2Rad;

        const Vec3 pl{ geo_.platformRadius * std::cos(psi),
                       geo_.platformRadius * std::sin(psi), 0.0 };
        const Vec3 r = rotate(roll, pitch, yaw, pl);
        const Vec3 q{ origin.x + r.x, origin.y + r.y, origin.z + r.z };
        const Vec3 B{ geo_.baseRadius * std::cos(phi),
                      geo_.baseRadius * std::sin(phi), 0.0 };
        const Vec3 l{ q.x - B.x, q.y - B.y, q.z - B.z };

        const double L2 = (l.x * l.x + l.y * l.y + l.z * l.z) - s2ma2;
        const double M = 2.0 * geo_.hornLength * l.z;
        const double N = 2.0 * geo_.hornLength * (l.x * std::cos(beta) + l.y * std::sin(beta));
        const double denom = std::sqrt(M * M + N * N);

        LegResult lr;
        if (denom < 1e-9) {
            lr.reachable = false;
            lr.angleDeg = 0.0;
        } else {
            double ratio = L2 / denom;
            if (ratio < -1.0 || ratio > 1.0) {
                lr.reachable = false;
                ratio = ratio < 0.0 ? -1.0 : 1.0;
            }
            lr.angleDeg = (std::asin(ratio) - std::atan2(N, M)) / kDeg2Rad;
        }
        if (!lr.reachable) out.allReachable = false;
        out.legs[i] = lr;

        double demand = geo_.demandHome
                        + (lr.angleDeg / geo_.angleAtFullScale) * geo_.demandHome;
        if (demand < 0.0) demand = 0.0;
        if (demand > geo_.demandMax) demand = geo_.demandMax;
        out.setpoints[geo_.bff[i] - 1] = static_cast<uint16_t>(std::lround(demand));
    }
    return out;
}
```

- [ ] **Step 4: Update `tests/test_kinematics.cpp` to use an instance**

Replace the `#include "StewartConfig.h"` line with `#include "StewartGeometry.h"`, and at the top of `main()` construct one instance and use it everywhere. Concretely: replace every `StewartKinematics::solve(` with `k.solve(` and every `StewartKinematics::homeHeight()` with `k.homeHeight()`, and replace uses of `stewart::kPhi/kPsi/kBeta/kRb/kRp/kHorn/kRod` in the `rodClosureError` helper and mapping test with the corresponding `StewartGeometry` default fields. The full updated file:

```cpp
#include "StewartKinematics.h"
#include "StewartGeometry.h"
#include <cmath>
#include <cstdio>
#include <cstdint>

static int g_failures = 0;
static int g_checks = 0;
static const StewartGeometry G = StewartGeometry::defaults();
static const StewartKinematics K(G);

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

static double rodClosureError(const Pose& pose, int leg) {
    const double d2r = 3.14159265358979323846 / 180.0;
    SolveResult r = K.solve(pose);
    double th = r.legs[leg].angleDeg * d2r;
    double phi = G.phiDeg[leg]*d2r, psi = G.psiDeg[leg]*d2r, beta = G.betaDeg[leg]*d2r;
    double hx = G.baseRadius*std::cos(phi) + G.hornLength*std::cos(th)*std::cos(beta);
    double hy = G.baseRadius*std::sin(phi) + G.hornLength*std::cos(th)*std::sin(beta);
    double hz = G.hornLength*std::sin(th);
    double cr=std::cos(pose.roll*d2r), sr=std::sin(pose.roll*d2r);
    double cp=std::cos(pose.pitch*d2r), sp=std::sin(pose.pitch*d2r);
    double cy=std::cos(pose.yaw*d2r), sy=std::sin(pose.yaw*d2r);
    double px=G.platformRadius*std::cos(psi), py=G.platformRadius*std::sin(psi), pz=0.0;
    double ax=px, ay=cr*py - sr*pz, az=sr*py + cr*pz;
    double bx=cp*ax + sp*az, by=ay, bz=-sp*ax + cp*az;
    double qx=cy*bx - sy*by + pose.surge;
    double qy=sy*bx + cy*by + pose.sway;
    double qz=bz + K.homeHeight() + pose.heave;
    double dx=qx-hx, dy=qy-hy, dz=qz-hz;
    return std::sqrt(dx*dx+dy*dy+dz*dz) - G.rodLength;
}

int main() {
    std::printf("home height...\n");
    checkNear(K.homeHeight(), 456.3, 0.5, "z0 ~= 456.3mm");

    std::printf("home pose -> all angles 0, all setpoints midscale...\n");
    {
        SolveResult r = K.solve(Pose{});
        check(r.allReachable, "home reachable");
        for (int i=0;i<6;i++) checkNear(r.legs[i].angleDeg, 0.0, 0.05, "home angle ~0");
        for (int i=0;i<6;i++) check(r.setpoints[i]==32640, "home setpoint 32640");
    }

    std::printf("setpoint mapping endpoints...\n");
    {
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
            Pose{}, Pose{0,0,30,0,0,0}, Pose{0,0,-30,0,0,0},
            Pose{0,0,0,5,0,0}, Pose{0,0,0,0,5,0}, Pose{0,0,0,0,0,5},
            Pose{10,-8,15,3,-2,4},
        };
        for (const Pose& p : poses)
            for (int i=0;i<6;i++)
                if (K.solve(p).legs[i].reachable)
                    checkNear(rodClosureError(p, i), 0.0, 1e-3, "rod closes to s");
    }

    std::printf("pure heave symmetry + sign...\n");
    {
        SolveResult up = K.solve(Pose{0,0,20,0,0,0});
        SolveResult dn = K.solve(Pose{0,0,-20,0,0,0});
        for (int i=0;i<6;i++) check(up.legs[i].angleDeg > 0.0, "heave up -> angle>0");
        for (int i=0;i<6;i++) check(dn.legs[i].angleDeg < 0.0, "heave down -> angle<0");
        for (int i=1;i<6;i++)
            checkNear(up.legs[i].angleDeg, up.legs[0].angleDeg, 0.05, "heave legs equal");
    }

    std::printf("extreme pose flagged unreachable, no NaN...\n");
    {
        SolveResult r = K.solve(Pose{0,0,10000,0,0,0});
        check(!r.allReachable, "huge heave unreachable");
        for (int i=0;i<6;i++) check(std::isfinite(r.legs[i].angleDeg), "angle finite");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
```

- [ ] **Step 5: Update `CMakeLists.txt`** — drop `src/StewartConfig.h`, add `src/StewartGeometry.h` to HEADERS. Resulting HEADERS block:

```cmake
set(HEADERS
    src/MotionProvider.h
    src/StatusWindow.h
    src/ConfigUtils.h
    src/DataRefManager.h
    src/MotionCues.h
    src/StewartKinematics.h
    src/StewartGeometry.h
    src/Pose.h
)
```

- [ ] **Step 6: Update `MotionProvider.h/.cpp`** to own a `StewartKinematics` instance.

In `MotionProvider.h`: keep `#include "StewartKinematics.h"`; add member `std::unique_ptr<StewartKinematics> kin_;` next to the others (remove nothing else). In `MotionProvider.cpp` `initialize()`, after creating dataRefs_, add `kin_ = std::make_unique<StewartKinematics>(StewartGeometry::defaults());`; in `onFlightLoopTick`, change `latestSolve_ = StewartKinematics::solve(pose);` to `latestSolve_ = kin_->solve(pose);`; in `shutdown()` add `kin_.reset();`. (Include `<memory>` already present.)

- [ ] **Step 7: Delete `src/StewartConfig.h` and commit**

```bash
git rm MotionProviderPlugin/src/StewartConfig.h
git add MotionProviderPlugin/src/StewartGeometry.h MotionProviderPlugin/src/StewartKinematics.h MotionProviderPlugin/src/StewartKinematics.cpp MotionProviderPlugin/tests/test_kinematics.cpp MotionProviderPlugin/CMakeLists.txt MotionProviderPlugin/src/MotionProvider.h MotionProviderPlugin/src/MotionProvider.cpp
git commit -m "refactor(motion): Phase 2a Task 1 - runtime StewartGeometry, instance-based kinematics"
```

- [ ] **Step 8: Manual test (user)** — `cd MotionProviderPlugin/tests && cmake -B build && cmake --build build && ./build/test_kinematics` → still `… checks, 0 failures` (behavior unchanged by the refactor).

---

### Task 2: `MotionConfig` — load geometry from `~/.motionprovider.toml`

**Files:**
- Create: `MotionProviderPlugin/src/MotionConfig.h`, `src/MotionConfig.cpp`
- Create: `MotionProviderPlugin/tests/test_config.cpp`
- Modify: `CMakeLists.txt`, `tests/CMakeLists.txt`, `.gitignore` (repo root), `MotionProviderPlugin/src/MotionProvider.cpp`

**PREREQUISITE (user):** place `toml.hpp` at `MotionProviderPlugin/third_party/tomlplusplus/toml.hpp` (see Global Constraints).

**Interfaces:**
- Produces: `MotionConfig::loadGeometry(const std::string& path) -> StewartGeometry` (returns `defaults()` on missing/invalid file; per-key fallback for absent keys); `MotionConfig::defaultPath() -> std::string` (`$HOME/.motionprovider.toml`).

- [ ] **Step 1: Create `MotionProviderPlugin/src/MotionConfig.h`**

```cpp
#pragma once
#include <string>
#include "StewartGeometry.h"

namespace MotionConfig {
    // Full path to the tuning config (~/.motionprovider.toml).
    std::string defaultPath();

    // Load geometry from a TOML file. Missing file or parse error -> defaults();
    // individual absent keys fall back to their default value.
    StewartGeometry loadGeometry(const std::string& path);
}
```

- [ ] **Step 2: Create `MotionProviderPlugin/src/MotionConfig.cpp`**

```cpp
#include "MotionConfig.h"
#include "toml.hpp"
#include <cstdlib>

std::string MotionConfig::defaultPath() {
    const char* home = std::getenv("HOME");
    return home ? std::string(home) + "/.motionprovider.toml"
                : std::string(".motionprovider.toml");
}

StewartGeometry MotionConfig::loadGeometry(const std::string& path) {
    StewartGeometry g = StewartGeometry::defaults();

    toml::table tbl;
    try {
        tbl = toml::parse_file(path);
    } catch (const toml::parse_error&) {
        return g;  // missing or invalid file -> full defaults
    }

    auto geo = tbl["geometry"].as_table();
    if (geo) {
        if (auto v = (*geo)["base_radius_mm"].value<double>())     g.baseRadius = *v;
        if (auto v = (*geo)["platform_radius_mm"].value<double>()) g.platformRadius = *v;
        if (auto v = (*geo)["horn_length_mm"].value<double>())     g.hornLength = *v;
        if (auto v = (*geo)["rod_length_mm"].value<double>())      g.rodLength = *v;

        auto readArr6d = [](const toml::table& t, const char* key, double out[6]) {
            if (auto arr = t[key].as_array()) {
                for (int i = 0; i < 6 && i < static_cast<int>(arr->size()); ++i)
                    if (auto v = arr->get(i)->value<double>()) out[i] = *v;
            }
        };
        auto readArr6i = [](const toml::table& t, const char* key, int out[6]) {
            if (auto arr = t[key].as_array()) {
                for (int i = 0; i < 6 && i < static_cast<int>(arr->size()); ++i)
                    if (auto v = arr->get(i)->value<int64_t>()) out[i] = static_cast<int>(*v);
            }
        };
        readArr6d(*geo, "base_angle_deg",   g.phiDeg);
        readArr6d(*geo, "anchor_angle_deg", g.psiDeg);
        readArr6d(*geo, "horn_azimuth_deg", g.betaDeg);
        readArr6i(*geo, "bff_actuator",     g.bff);
    }

    auto servo = tbl["servo"].as_table();
    if (servo) {
        if (auto v = (*servo)["angle_at_full_scale_deg"].value<double>()) g.angleAtFullScale = *v;
        if (auto v = (*servo)["demand_home"].value<int64_t>()) g.demandHome = static_cast<int>(*v);
        if (auto v = (*servo)["demand_max"].value<int64_t>())  g.demandMax  = static_cast<int>(*v);
    }

    return g;
}
```

- [ ] **Step 3: Create `MotionProviderPlugin/tests/test_config.cpp`**

```cpp
#include "MotionConfig.h"
#include "StewartGeometry.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <cmath>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }
static void near(double a,double b,const char*w){ ++g_checks; if(std::fabs(a-b)>1e-9){++g_failures; std::printf("  FAIL: %s (%.6f vs %.6f)\n",w,a,b);} }

int main() {
    // Missing file -> defaults.
    {
        StewartGeometry g = MotionConfig::loadGeometry("/no/such/file.toml");
        StewartGeometry d = StewartGeometry::defaults();
        near(g.baseRadius, d.baseRadius, "missing file -> default Rb");
        near(g.rodLength,  d.rodLength,  "missing file -> default rod");
    }

    // Partial file -> overrides present keys, defaults the rest.
    {
        std::string tmp = std::string(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp")
                          + "/mp_test.toml";
        {
            std::ofstream f(tmp);
            f << "[geometry]\n"
                 "base_radius_mm = 500.0\n"
                 "base_angle_deg = [1,2,3,4,5,6]\n"
                 "[servo]\n"
                 "demand_home = 30000\n";
        }
        StewartGeometry g = MotionConfig::loadGeometry(tmp);
        StewartGeometry d = StewartGeometry::defaults();
        near(g.baseRadius, 500.0, "override Rb");
        near(g.platformRadius, d.platformRadius, "default Rp kept");
        near(g.phiDeg[0], 1.0, "override phi[0]");
        near(g.phiDeg[5], 6.0, "override phi[5]");
        check(g.demandHome == 30000, "override demand_home");
        check(g.demandMax == d.demandMax, "default demand_max kept");
        std::remove(tmp.c_str());
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
```

- [ ] **Step 4: Wire the plugin build.** In `CMakeLists.txt`: add `src/MotionConfig.cpp` to SOURCES, `src/MotionConfig.h` to HEADERS, and add the toml++ include dir to `target_include_directories(MotionProvider PRIVATE ...)`:

```cmake
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/tomlplusplus
```

- [ ] **Step 5: Wire the test build.** Replace `tests/CMakeLists.txt` with:

```cmake
cmake_minimum_required(VERSION 3.15)
project(motion_provider_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(test_kinematics test_kinematics.cpp ../src/StewartKinematics.cpp)
target_include_directories(test_kinematics PRIVATE ../src)

add_executable(test_config test_config.cpp ../src/MotionConfig.cpp)
target_include_directories(test_config PRIVATE ../src ../third_party/tomlplusplus)

enable_testing()
add_test(NAME kinematics COMMAND test_kinematics)
add_test(NAME config COMMAND test_config)
```

- [ ] **Step 6: Git-ignore the vendored header.** Append to the repo-root `.gitignore`:

```
# vendored single-header toml++ (user supplies locally)
MotionProviderPlugin/third_party/tomlplusplus/toml.hpp
```

- [ ] **Step 7: Load config at plugin start.** In `MotionProvider.cpp` `initialize()`, change the kinematics construction to load from config:

```cpp
    kin_ = std::make_unique<StewartKinematics>(
        MotionConfig::loadGeometry(MotionConfig::defaultPath()));
```
and add `#include "MotionConfig.h"` at the top.

- [ ] **Step 8: Commit**

```bash
git add MotionProviderPlugin/src/MotionConfig.h MotionProviderPlugin/src/MotionConfig.cpp MotionProviderPlugin/tests/test_config.cpp MotionProviderPlugin/tests/CMakeLists.txt MotionProviderPlugin/CMakeLists.txt MotionProviderPlugin/src/MotionProvider.cpp .gitignore
git commit -m "feat(motion): Phase 2a Task 2 - load geometry from ~/.motionprovider.toml (toml++)"
```

- [ ] **Step 9: Manual test (user)** — vendor `toml.hpp`, then `cd MotionProviderPlugin/tests && cmake -B build && cmake --build build && ./build/test_config && ./build/test_kinematics` → both `0 failures`. Optionally drop a `~/.motionprovider.toml` overriding `horn_length_mm` and confirm the built plugin's setpoints shift.

---

### Task 3: "Reload config" button + `StatusData` aggregate

**Files:**
- Create: `MotionProviderPlugin/src/StatusData.h`
- Modify: `src/StatusWindow.h`, `src/StatusWindow.cpp`, `src/MotionProvider.h`, `src/MotionProvider.cpp`, `CMakeLists.txt`

**Interfaces:**
- Produces: `struct StatusData { MotionCues cues; SolveResult solve; bool manualMode; int manualAxis; Pose manualPose; bool lastReloadOk; }`; `StatusWindow::update(const StatusData&)`; `StatusWindow::setReloadCallback(std::function<void()>)`. `MotionProvider::reloadConfig()`.

- [ ] **Step 1: Create `MotionProviderPlugin/src/StatusData.h`**

```cpp
#pragma once
#include "MotionCues.h"
#include "StewartKinematics.h"
#include "Pose.h"

// Everything the status window renders in one snapshot.
struct StatusData {
    MotionCues  cues;
    SolveResult solve;
    bool        manualMode = false;
    int         manualAxis = 0;      // 0..5: surge,sway,heave,roll,pitch,yaw
    Pose        manualPose;
    bool        lastReloadOk = true;
};
```

- [ ] **Step 2: Update `StatusWindow.h`** — swap the two-arg update for `update(const StatusData&)`, add a reload callback and a stored `StatusData`, and a button rectangle. Replace the include of `MotionCues.h`/`StewartKinematics.h` with `#include "StatusData.h"`, add `#include <functional>`. Replace the public `update` declaration and add:

```cpp
    void update(const StatusData& data);
    void setReloadCallback(std::function<void()> cb);
```
Replace the `MotionCues cues_; SolveResult solve_;` members with:
```cpp
    StatusData data_;
    std::function<void()> reloadCallback_;
    int btnLeft_ = 0, btnTop_ = 0, btnRight_ = 0, btnBottom_ = 0; // reload button hitbox
```

- [ ] **Step 3: Update `StatusWindow.cpp`** — store `data_`, draw a clickable "Reload config" button, hit-test it in the mouse handler, and render manual state.

(a) `update`:
```cpp
void StatusWindow::update(const StatusData& data) {
    data_ = data;
    bool nowVisible = isVisible();
    if (nowVisible != lastKnownVisible_) {
        lastKnownVisible_ = nowVisible;
        saveStatusWindowVisible(nowVisible);
    }
}
```
(b) `setReloadCallback`:
```cpp
void StatusWindow::setReloadCallback(std::function<void()> cb) {
    reloadCallback_ = std::move(cb);
}
```
(c) the mouse click adapter in `initialize()` currently returns 1 and ignores clicks; route it to the instance:
```cpp
    params.handleMouseClickFunc = [](XPLMWindowID w, int x, int y, XPLMMouseStatus s, void* ref) -> int {
        StatusWindow* self = static_cast<StatusWindow*>(ref);
        if (self) self->mouseCallback(w, x, y, s, ref);
        return 1;
    };
```
and add a private method `mouseCallback` (declare in the header alongside the other statics is unnecessary — make it a non-static member):
Header: add `void mouseCallback(XPLMWindowID, int x, int y, XPLMMouseStatus, void*);`
Impl:
```cpp
void StatusWindow::mouseCallback(XPLMWindowID, int x, int y, XPLMMouseStatus s, void*) {
    if (s != xplm_MouseDown) return;
    if (x >= btnLeft_ && x <= btnRight_ && y <= btnTop_ && y >= btnBottom_) {
        if (reloadCallback_) reloadCallback_();
    }
}
```
(d) `params.bottom` — widen for the extra lines; set `params.bottom = 170;`
(e) `draw()`:
```cpp
void StatusWindow::draw() {
    if (!windowId_) return;
    int left, top, right, bottom;
    XPLMGetWindowGeometry(windowId_, &left, &top, &right, &bottom);
    XPLMDrawTranslucentDarkBox(left, top, right, bottom);

    int x = left + 10;
    int y = top - 20;
    char buf[160];

    drawString(x, y, "Motion Provider v0.4 (Phase 2a)", 0.8f, 1.0f, 0.8f);
    y -= 20;

    static const char* kAxis[6] = { "surge","sway","heave","roll","pitch","yaw" };
    if (data_.manualMode) {
        std::snprintf(buf, sizeof(buf),
            "MANUAL  axis=%s  [%.1f %.1f %.1f | %.1f %.1f %.1f]",
            kAxis[data_.manualAxis],
            data_.manualPose.surge, data_.manualPose.sway, data_.manualPose.heave,
            data_.manualPose.roll, data_.manualPose.pitch, data_.manualPose.yaw);
        drawString(x, y, buf, 1.0f, 0.9f, 0.5f);
    } else {
        drawString(x, y, "AUTO (attitude placeholder)   [M] manual", 0.7f, 0.8f, 0.9f);
    }
    y -= 16;
    drawString(x, y, "[M] mode  [Tab] axis  [Up/Dn] nudge  [R] reset",
               0.6f, 0.6f, 0.65f);
    y -= 18;

    drawString(x, y, data_.solve.allReachable ? "IK: all legs reachable"
                                               : "IK: POSE UNREACHABLE",
               data_.solve.allReachable ? 0.6f : 1.0f,
               data_.solve.allReachable ? 1.0f : 0.4f, 0.4f);
    y -= 16;

    for (int i = 0; i < 6; ++i) {
        std::snprintf(buf, sizeof(buf), "P%d  %+7.2f deg  ->  %5u%s",
                      i + 1, data_.solve.legs[i].angleDeg, data_.solve.setpoints[i],
                      data_.solve.legs[i].reachable ? "" : "  (unreachable)");
        drawString(x, y, buf, 0.85f, 0.85f, 0.9f);
        y -= 16;
    }

    // Reload button
    y -= 4;
    btnLeft_ = x; btnTop_ = y + 12; btnRight_ = x + 150; btnBottom_ = y - 4;
    drawString(x, y, data_.lastReloadOk ? "[ Reload config ]" : "[ Reload FAILED ]",
               0.7f, data_.lastReloadOk ? 0.9f : 0.4f, 0.9f);
}
```

- [ ] **Step 4: Update `MotionProvider`** — build a `StatusData`, add `reloadConfig()`, wire the callback.

`MotionProvider.h`: add `#include "StatusData.h"`; add members `bool manualMode_ = false; int manualAxis_ = 0; Pose manualPose_; bool lastReloadOk_ = true;` and method `void reloadConfig();`.
`MotionProvider.cpp`:
- in `initialize()` after creating the window: `statusWindow_->setReloadCallback([this]{ reloadConfig(); });`
- add:
```cpp
void MotionProvider::reloadConfig() {
    StewartGeometry g = MotionConfig::loadGeometry(MotionConfig::defaultPath());
    kin_ = std::make_unique<StewartKinematics>(g);
    lastReloadOk_ = true;   // loadGeometry never throws; defaults on error
}
```
- in `onFlightLoopTick`, replace the `statusWindow_->update(...)` call with building a `StatusData`:
```cpp
        if (statusWindow_) {
            StatusData sd;
            sd.cues = latestCues_;
            sd.solve = latestSolve_;
            sd.manualMode = manualMode_;
            sd.manualAxis = manualAxis_;
            sd.manualPose = manualPose_;
            sd.lastReloadOk = lastReloadOk_;
            statusWindow_->update(sd);
        }
```

- [ ] **Step 5: Commit**

```bash
git add MotionProviderPlugin/src/StatusData.h MotionProviderPlugin/src/StatusWindow.h MotionProviderPlugin/src/StatusWindow.cpp MotionProviderPlugin/src/MotionProvider.h MotionProviderPlugin/src/MotionProvider.cpp MotionProviderPlugin/CMakeLists.txt
git commit -m "feat(motion): Phase 2a Task 3 - StatusData aggregate + Reload config button"
```

- [ ] **Step 6: Manual (user)** — build+load, edit `~/.motionprovider.toml` (e.g. `rod_length_mm = 470`), click "Reload config", confirm setpoints shift and the button label stays "[ Reload config ]".

---

### Task 4: Manual DOF keyboard control

**Files:**
- Modify: `src/StatusWindow.h`, `src/StatusWindow.cpp`, `src/MotionProvider.h`, `src/MotionProvider.cpp`

**Interfaces:**
- Produces: `StatusWindow::setKeyCommandCallback(std::function<void(char)>)`; `MotionProvider::onManualKey(char)`. Manual pose replaces the attitude placeholder when manual mode is on.

- [ ] **Step 1: `StatusWindow.h`** — add `void setKeyCommandCallback(std::function<void(char)> cb);` and member `std::function<void(char)> keyCommandCallback_;`.

- [ ] **Step 2: `StatusWindow.cpp`** — forward DOF keys from `keyCallback` to the callback (keep ESC hiding the window):

```cpp
void StatusWindow::keyCallback(XPLMWindowID, char inKey, XPLMKeyFlags, char inVirtualKey,
                               void* inRefcon, int) {
    StatusWindow* self = static_cast<StatusWindow*>(inRefcon);
    if (!self) return;
    if (inKey == 27) { self->setVisible(false); return; }   // ESC hides
    if (self->keyCommandCallback_) self->keyCommandCallback_(inKey);
}
```
and:
```cpp
void StatusWindow::setKeyCommandCallback(std::function<void(char)> cb) {
    keyCommandCallback_ = std::move(cb);
}
```

- [ ] **Step 3: `MotionProvider.cpp`** — wire the callback and handle keys; feed manual pose to the IK when manual mode is on.

In `initialize()` after the reload callback: `statusWindow_->setKeyCommandCallback([this](char k){ onManualKey(k); });`

Add `onManualKey` (declare `void onManualKey(char key);` in the header):
```cpp
void MotionProvider::onManualKey(char key) {
    const float kTransStep = 2.0f;   // mm
    const float kRotStep   = 0.5f;   // deg
    switch (key) {
        case 'm': case 'M': manualMode_ = !manualMode_; return;
        case '\t': manualAxis_ = (manualAxis_ + 1) % 6; return;  // Tab
        case 'r': case 'R': manualPose_ = Pose{}; return;
        default: break;
    }
    if (!manualMode_) return;
    float dir = 0.0f;
    if (key == '+' || key == '=') dir = 1.0f;
    else if (key == '-' || key == '_') dir = -1.0f;
    else return;
    switch (manualAxis_) {
        case 0: manualPose_.surge += dir * kTransStep; break;
        case 1: manualPose_.sway  += dir * kTransStep; break;
        case 2: manualPose_.heave += dir * kTransStep; break;
        case 3: manualPose_.roll  += dir * kRotStep;   break;
        case 4: manualPose_.pitch += dir * kRotStep;   break;
        case 5: manualPose_.yaw   += dir * kRotStep;   break;
    }
}
```
(Note: X-Plane delivers arrow keys as virtual keys, not chars, so this uses `+`/`-` for nudging — simpler and reliable through the char path. The draw() hint text in Task 3 says "Up/Dn"; change that hint string to `[+/-] nudge`.)

In `onFlightLoopTick`, replace the placeholder-pose block so manual mode wins:
```cpp
    Pose pose;
    if (manualMode_) {
        pose = manualPose_;
    } else {
        auto clampf = [](float v, float lo, float hi){ return std::max(lo, std::min(hi, v)); };
        pose.roll  = clampf(latestCues_.rollDeg,  -8.0f, 8.0f);
        pose.pitch = clampf(latestCues_.pitchDeg, -8.0f, 8.0f);
    }
    latestSolve_ = kin_->solve(pose);
```

- [ ] **Step 4: Fix the hint string** in `StatusWindow::draw()` (Task 3 Step 3e): change `"[M] mode  [Tab] axis  [Up/Dn] nudge  [R] reset"` to `"[M] mode  [Tab] axis  [+/-] nudge  [R] reset"`.

- [ ] **Step 5: Commit**

```bash
git add MotionProviderPlugin/src/StatusWindow.h MotionProviderPlugin/src/StatusWindow.cpp MotionProviderPlugin/src/MotionProvider.h MotionProviderPlugin/src/MotionProvider.cpp
git commit -m "feat(motion): Phase 2a Task 4 - manual DOF keyboard control"
```

- [ ] **Step 6: Manual verification (user)** — build+load, open window, press `M` (→ MANUAL), `Tab` to pick an axis, `+`/`-` to nudge, watch the six setpoints track and note where a leg first flags "(unreachable)". `R` resets to home (all 32640). `M` again returns to AUTO.

---

## Self-Review

- **Spec coverage:** spec §4 Phase 2a (manual DOF UI to set the 6 poses) — Task 4. Spec §5/§6 config decisions (TOML via toml++, hot-reloadable via a Reload button, geometry in config) — Tasks 1–3. Geometry defaults match the Phase-2 confirmed numbers (behavior parity constraint), so the Task-1 refactor is a no-op in results.
- **Placeholder scan:** no TBD/TODO. The vendored `toml.hpp` is a stated user prerequisite (sandbox can't fetch), not a plan gap. The attitude placeholder remains only as the AUTO-mode fallback, explicitly superseded by the washout filter in Phase 3.
- **Type consistency:** `StewartKinematics` goes static→instance in Task 1 across header, impl, tests, and the `MotionProvider` call site together. `StatusWindow::update` moves two-arg→`StatusData` in Task 3 across header, impl, and the sole caller. `StatusData` fields used in `draw()` match the struct. `StewartGeometry` field names are identical in the struct, `MotionConfig` loader, and tests. Keyboard nudge uses `+`/`-` (char path) consistently with the corrected hint string (Task 4 Step 4).
- **Constraint check:** `StewartGeometry.h`, `StewartKinematics.*`, `Pose.h`, `MotionConfig.*`, `StatusData.h` include no XPLM header; `MotionConfig` and both math modules build in the native test target (test_config links only MotionConfig.cpp + toml++). Config path `~/.motionprovider.toml`; loader never throws (defaults on error). Only `Plugin.cpp` touches the ABI; `DataRefManager` remains the only XPLM data reader.
- **Test note:** `test_config` covers the two behaviors that matter for hot-reload safety — missing file → full defaults, and partial file → per-key override with the rest defaulted — without needing the real home file. The kinematics suite is unchanged in intent and must still report 0 failures after the Task-1 refactor (the parity gate).
