# Rudder Axis Curve Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Tunable, persisted MSFS-style S-curve for the rudder pedal axis in `DCUProviderPlugin`, with its own slider window and curve plot.

**Architecture:** A pure `AxisCurve` struct (five parameters + `apply()`) sits between the incoming `SerialMessageRudder` value and `DataRefManager::setRudderRatio()`. `ConfigUtils` persists it under an axis-prefixed key set in the existing `~/.dcuprovider.cfg`. A new `CurveWindow` (XPLM window + immediate-mode GL plot + mouse-dragged sliders) edits the struct in place; `StatusWindow` gains a generic `appendMenuItem()` so the new window hangs off the existing "DCU Provider" menu.

**Tech Stack:** C++17, X-Plane SDK (XPLM 4.0 window/menu/graphics), OpenGL immediate mode (already linked on mac/win), CMake. Tests: framework-free `check()` executables under `tests/` driven by CTest, same shape as `MotionProviderPlugin/tests`.

**Spec:** `docs/superpowers/specs/2026-10-11-rudder-axis-curve-design.md`

## Global Constraints

- All work in `DCUProviderPlugin/`; `RudderCAN`, `DCU`, `shared/CANBase` untouched.
- Toe brakes stay raw (linear). Only `setRudderRatio()` gets the curve.
- Config keys exactly: `curve.rudder.sens_pos`, `curve.rudder.sens_neg`, `curve.rudder.neutral`, `curve.rudder.dead_zone`, `curve.rudder.extremity`, `curve_window_visible`.
- Parameter ranges: `sensPos`/`sensNeg`/`neutral` −1..1, `deadZone`/`extremityDeadZone` 0..1 in the struct, slider caps 0.5 for the last two.
- Sensitivity exponent `p = 3^(-s)`; `s = −1` is cubic, `s = +1` is cube root.
- Save on slider release (`xplm_MouseUp`) and on reset, never per drag frame.
- Everything on X-Plane's main thread, no locking.
- Plugin build must stay green on macOS (`./build-macos.sh`). Windows cross-build (`./build-xc-windows.sh`) is the second gate if the MinGW toolchain is present.
- Commits: conventional prefix (`feat(plugin):`, `test(plugin):`, `refactor(plugin):`, `docs:`), each ends with `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>`.

## Review Focus

1. **Out-of-range wire values** (`rudder` int16 outside ±1000 from a buggy node) → output still clamped to −1..1, never NaN. Pinned in Task 1.
2. **Garbage in the config file** (`curve.rudder.sens_pos=abc`, `=nan`, `=5`) → loads as 0 / clamped, no exception, no NaN reaching the sim. Pinned in Task 1 (`clampToRange` NaN) and Task 2 (parse).
3. **Neutral at the stop** (`neutral = ±1`) → no division by zero, output finite and monotonic. Pinned in Task 1.
4. **Dead zone + extremity overlapping** (both 0.5, leaving a zero-width active band) → finite, monotonic, hits 0 and ±1. Pinned in Task 1.
5. **Partial config** (only some `curve.rudder.*` keys present, e.g. after a hand edit) → missing ones default to 0, present ones kept. Pinned in Task 2.

---

### Task 1: `AxisCurve` math + native test harness

**Files:**
- Create: `DCUProviderPlugin/src/AxisCurve.h`
- Create: `DCUProviderPlugin/src/AxisCurve.cpp`
- Create: `DCUProviderPlugin/tests/CMakeLists.txt`
- Create: `DCUProviderPlugin/tests/test_axiscurve.cpp`
- Modify: `DCUProviderPlugin/CMakeLists.txt` (SOURCES/HEADERS lists, ~lines 75-100)

**Interfaces:**
- Consumes: nothing.
- Produces:
  ```cpp
  struct AxisCurve {
      float sensPos = 0.f, sensNeg = 0.f, neutral = 0.f, deadZone = 0.f, extremityDeadZone = 0.f;
      float apply(float in) const;
      void  clampToRange();
  };
  ```

- [ ] **Step 1: Create the test harness CMake file**

`DCUProviderPlugin/tests/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.15)
project(dcu_provider_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(test_axiscurve test_axiscurve.cpp ../src/AxisCurve.cpp)
target_include_directories(test_axiscurve PRIVATE ../src)

enable_testing()
add_test(NAME axiscurve COMMAND test_axiscurve)
```

- [ ] **Step 2: Write the failing test**

`DCUProviderPlugin/tests/test_axiscurve.cpp`:

```cpp
#include "AxisCurve.h"
#include <cmath>
#include <cstdio>
#include <limits>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w) { ++g_checks; if (!c) { ++g_failures; std::printf("  FAIL: %s\n", w); } }
static bool near(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) <= eps; }

// Sweep -1..1 and verify output never decreases and is always finite and in range.
static bool monotonicAndFinite(const AxisCurve& c) {
    float prev = -2.f;
    for (int i = 0; i <= 400; ++i) {
        float x = -1.f + 2.f * i / 400.f;
        float y = c.apply(x);
        if (std::isnan(y) || y < -1.f || y > 1.f) return false;
        if (y < prev - 1e-6f) return false;
        prev = y;
    }
    return true;
}

int main() {
    // Defaults are the identity.
    {
        AxisCurve c;
        check(near(c.apply(0.f), 0.f), "identity: 0 -> 0");
        check(near(c.apply(0.5f), 0.5f), "identity: 0.5 -> 0.5");
        check(near(c.apply(-0.25f), -0.25f), "identity: -0.25 -> -0.25");
        check(near(c.apply(1.f), 1.f) && near(c.apply(-1.f), -1.f), "identity: endpoints");
    }

    // Out-of-range and NaN input is clamped (Review Focus 1).
    {
        AxisCurve c;
        check(near(c.apply(3.f), 1.f), "input 3 clamps to 1");
        check(near(c.apply(-7.f), -1.f), "input -7 clamps to -1");
        check(near(c.apply(std::numeric_limits<float>::quiet_NaN()), 0.f), "NaN input -> 0");
    }

    // Sensitivity -1 is cubic (flat centre), +1 is cube root (steep centre).
    {
        AxisCurve c; c.sensPos = -1.f; c.sensNeg = -1.f;
        check(near(c.apply(0.5f), 0.125f), "sens -1: 0.5 -> 0.125 (x^3)");
        check(near(c.apply(-0.5f), -0.125f), "sens -1: -0.5 -> -0.125");
        check(near(c.apply(1.f), 1.f), "sens -1 still reaches 1");
        c.sensPos = 1.f; c.sensNeg = 1.f;
        check(near(c.apply(0.5f), std::cbrt(0.5f)), "sens +1: 0.5 -> cbrt(0.5)");
        check(near(c.apply(-0.125f), -0.5f), "sens +1: -0.125 -> -0.5");
    }

    // Sensitivity halves are independent.
    {
        AxisCurve c; c.sensPos = -1.f; c.sensNeg = 0.f;
        check(near(c.apply(0.5f), 0.125f), "sensPos only affects positive half");
        check(near(c.apply(-0.5f), -0.5f), "negative half stays linear");
    }

    // Dead zone: zero inside, continuous at the edge, still reaches 1.
    {
        AxisCurve c; c.deadZone = 0.2f;
        check(near(c.apply(0.1f), 0.f), "inside dead zone -> 0");
        check(near(c.apply(-0.2f), 0.f), "dead zone edge -> 0");
        check(near(c.apply(0.21f), 0.0125f, 1e-3f), "just past dead zone is tiny, not a jump");
        check(near(c.apply(0.6f), 0.5f), "dead zone rescales: 0.6 -> 0.5");
        check(near(c.apply(1.f), 1.f), "dead zone still reaches 1");
    }

    // Extremity dead zone: saturates early, continuous.
    {
        AxisCurve c; c.extremityDeadZone = 0.2f;
        check(near(c.apply(0.8f), 1.f), "extremity edge -> 1");
        check(near(c.apply(0.9f), 1.f), "inside extremity -> 1");
        check(near(c.apply(-0.95f), -1.f), "negative extremity -> -1");
        check(near(c.apply(0.4f), 0.5f), "extremity rescales: 0.4 -> 0.5");
    }

    // Neutral shifts the zero point; stops still reach +-1.
    {
        AxisCurve c; c.neutral = 0.2f;
        check(near(c.apply(0.2f), 0.f), "neutral 0.2: raw 0.2 -> 0");
        check(near(c.apply(1.f), 1.f), "neutral 0.2: raw 1 -> 1");
        check(near(c.apply(-1.f), -1.f), "neutral 0.2: raw -1 -> -1");
        check(near(c.apply(0.6f), 0.5f), "neutral 0.2: raw 0.6 -> 0.5 (half of remaining span)");
        check(near(c.apply(-0.4f), -0.5f), "neutral 0.2: raw -0.4 -> -0.5");
    }

    // Neutral at the stop must not divide by zero (Review Focus 3).
    {
        AxisCurve c; c.neutral = 1.f;
        check(monotonicAndFinite(c), "neutral = 1: finite and monotonic");
        c.neutral = -1.f;
        check(monotonicAndFinite(c), "neutral = -1: finite and monotonic");
    }

    // Overlapping dead zones (Review Focus 4).
    {
        AxisCurve c; c.deadZone = 0.5f; c.extremityDeadZone = 0.5f;
        check(monotonicAndFinite(c), "dz 0.5 + ext 0.5: finite and monotonic");
        check(near(c.apply(0.f), 0.f) && near(c.apply(1.f), 1.f), "dz 0.5 + ext 0.5: 0 -> 0, 1 -> 1");
        c.deadZone = 1.f; c.extremityDeadZone = 1.f;
        check(monotonicAndFinite(c), "dz 1 + ext 1: still finite");
    }

    // Mixed parameters: monotonic end to end.
    {
        AxisCurve c; c.sensPos = -0.35f; c.sensNeg = 0.6f; c.neutral = -0.1f; c.deadZone = 0.05f; c.extremityDeadZone = 0.1f;
        check(monotonicAndFinite(c), "mixed params: finite and monotonic");
        check(c.apply(-1.f) == -1.f && c.apply(1.f) == 1.f, "mixed params: stops reach +-1 exactly");
    }

    // clampToRange() sanitises out-of-range and NaN values (Review Focus 2).
    {
        AxisCurve c;
        c.sensPos = 5.f; c.sensNeg = -5.f; c.neutral = std::numeric_limits<float>::quiet_NaN();
        c.deadZone = -1.f; c.extremityDeadZone = 2.f;
        c.clampToRange();
        check(c.sensPos == 1.f && c.sensNeg == -1.f, "clampToRange: sensitivities clamp to +-1");
        check(c.neutral == 0.f, "clampToRange: NaN -> 0");
        check(c.deadZone == 0.f && c.extremityDeadZone == 1.f, "clampToRange: dead zones clamp to 0..1");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
```

- [ ] **Step 3: Run to verify it fails**

Run from `DCUProviderPlugin/`:
```bash
cmake -S tests -B tests/build && cmake --build tests/build
```
Expected: FAIL — `AxisCurve.h: No such file or directory` (or missing `../src/AxisCurve.cpp`).

- [ ] **Step 4: Write the implementation**

`DCUProviderPlugin/src/AxisCurve.h`:

```cpp
#pragma once

// MSFS-style response curve for one joystick axis. Pure math, no X-Plane
// dependency, so it is unit-tested natively (tests/test_axiscurve.cpp).
//
// apply() runs: clamp -> neutral shift -> dead zone -> extremity dead zone ->
// sensitivity power curve -> clamp. Every stage is continuous and monotonic.
struct AxisCurve {
    float sensPos  = 0.f;   // -1..1, positive half: -1 flat centre (x^3), +1 steep centre (cbrt)
    float sensNeg  = 0.f;   // -1..1, negative half
    float neutral  = 0.f;   // -1..1, raw value that maps to output 0
    float deadZone = 0.f;   //  0..1, |x| below this (after neutral) -> 0
    float extremityDeadZone = 0.f; // 0..1, |x| above 1-this -> 1

    // in -1..1 (anything else is clamped, NaN -> 0) -> out -1..1
    float apply(float in) const;

    // Clamp every parameter to its range; NaN -> 0. Call after loading from config.
    void clampToRange();
};
```

`DCUProviderPlugin/src/AxisCurve.cpp`:

```cpp
#include "AxisCurve.h"
#include <algorithm>
#include <cmath>

namespace {

// Smallest span any rescale divides by. Keeps neutral = +-1 and
// deadZone/extremity = 1 finite instead of dividing by zero.
constexpr float kMinSpan = 0.05f;

float clamp1(float v) { return std::max(-1.f, std::min(1.f, v)); }

float sane(float v, float lo, float hi) {
    if (std::isnan(v)) return 0.f;
    return std::max(lo, std::min(hi, v));
}

} // namespace

void AxisCurve::clampToRange() {
    sensPos = sane(sensPos, -1.f, 1.f);
    sensNeg = sane(sensNeg, -1.f, 1.f);
    neutral = sane(neutral, -1.f, 1.f);
    deadZone = sane(deadZone, 0.f, 1.f);
    extremityDeadZone = sane(extremityDeadZone, 0.f, 1.f);
}

float AxisCurve::apply(float in) const {
    float x = std::isnan(in) ? 0.f : clamp1(in);

    // Neutral: shift the zero point, then stretch each half back to full
    // length so the pedal stops still reach +-1.
    x -= neutral;
    const float span = (x >= 0.f) ? std::max(kMinSpan, 1.f - neutral)
                                  : std::max(kMinSpan, 1.f + neutral);
    x = clamp1(x / span);

    const float sign = (x < 0.f) ? -1.f : 1.f;
    float m = std::fabs(x);

    // Dead zone around the centre, rescaled so there is no jump at the edge.
    const float dz = std::min(deadZone, 1.f - kMinSpan);
    m = (m <= dz) ? 0.f : (m - dz) / (1.f - dz);

    // Extremity dead zone: full deflection is reached before the stop.
    const float ext = std::min(extremityDeadZone, 1.f - kMinSpan);
    m = (m >= 1.f - ext) ? 1.f : m / (1.f - ext);

    // Sensitivity: power curve, p = 3^(-s). s=0 linear, s=-1 cubic, s=+1 cube root.
    const float s = (sign > 0.f) ? sensPos : sensNeg;
    m = std::pow(m, std::pow(3.f, -s));

    return clamp1(sign * m);
}
```

- [ ] **Step 5: Run the tests**

```bash
cmake -S tests -B tests/build && cmake --build tests/build && ctest --test-dir tests/build --output-on-failure
```
Expected: `1/1 Test #1: axiscurve ... Passed`, output ends with `N checks, 0 failures`.

- [ ] **Step 6: Add the sources to the plugin build**

In `DCUProviderPlugin/CMakeLists.txt`, extend the two lists:

```cmake
set(SOURCES
    ...
    src/ConfigUtils.cpp
    src/AxisCurve.cpp
)

set(HEADERS
    ...
    src/ConfigUtils.h
    src/AxisCurve.h
)
```

Then from `DCUProviderPlugin/`:
```bash
./build-macos.sh
```
Expected: build succeeds, plugin copied to the X-Plane plugins dir.

- [ ] **Step 7: Commit**

```bash
git add src/AxisCurve.h src/AxisCurve.cpp tests/CMakeLists.txt tests/test_axiscurve.cpp CMakeLists.txt
git commit -m "feat(plugin): AxisCurve response-curve math with native tests

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 2: Persist the curve and the window visibility in `ConfigUtils`

**Files:**
- Modify: `DCUProviderPlugin/src/ConfigUtils.h`
- Modify: `DCUProviderPlugin/src/ConfigUtils.cpp`
- Create: `DCUProviderPlugin/tests/test_config.cpp`
- Modify: `DCUProviderPlugin/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `AxisCurve` from Task 1.
- Produces:
  ```cpp
  AxisCurve loadAxisCurve(const std::string& axis);
  void      saveAxisCurve(const std::string& axis, const AxisCurve& curve);
  bool      loadCurveWindowVisible();   // default false
  void      saveCurveWindowVisible(bool visible);
  ```

- [ ] **Step 1: Write the failing test**

`ConfigUtils` resolves the file from `$HOME` on every call, so the test points `HOME` at a fresh temp dir. POSIX only (`setenv`, `mkdtemp`); the test harness is macOS/Linux-only anyway.

`DCUProviderPlugin/tests/test_config.cpp`:

```cpp
#include "ConfigUtils.h"
#include "AxisCurve.h"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w) { ++g_checks; if (!c) { ++g_failures; std::printf("  FAIL: %s\n", w); } }

static void writeConfig(const std::string& body) {
    std::ofstream f(getConfigFilePath(), std::ios::trunc);
    f << body;
}

int main() {
    char tmpl[] = "/tmp/dcucfgXXXXXX";
    const char* dir = mkdtemp(tmpl);
    if (!dir) { std::printf("mkdtemp failed\n"); return 1; }
    setenv("HOME", dir, 1);

    // No file at all -> defaults.
    {
        std::remove(getConfigFilePath().c_str());
        AxisCurve c = loadAxisCurve("rudder");
        check(c.sensPos == 0.f && c.sensNeg == 0.f && c.neutral == 0.f && c.deadZone == 0.f && c.extremityDeadZone == 0.f,
              "missing file -> all zero");
        check(loadCurveWindowVisible() == false, "missing file -> curve window hidden");
    }

    // Round trip.
    {
        AxisCurve c; c.sensPos = -0.35f; c.sensNeg = 0.25f; c.neutral = 0.1f; c.deadZone = 0.05f; c.extremityDeadZone = 0.2f;
        saveAxisCurve("rudder", c);
        AxisCurve r = loadAxisCurve("rudder");
        check(r.sensPos == -0.35f && r.sensNeg == 0.25f, "round trip: sensitivities");
        check(r.neutral == 0.1f && r.deadZone == 0.05f && r.extremityDeadZone == 0.2f, "round trip: neutral/dz/ext");
    }

    // Saving the curve keeps unrelated keys.
    {
        saveLastUsedPort("/dev/tty.fake");
        AxisCurve c; c.sensPos = 0.5f;
        saveAxisCurve("rudder", c);
        check(loadLastUsedPort() == "/dev/tty.fake", "saveAxisCurve keeps other keys");
        check(loadAxisCurve("rudder").sensPos == 0.5f, "...and writes its own");
    }

    // Axis prefix isolates axes.
    {
        AxisCurve a; a.neutral = 0.3f;
        saveAxisCurve("left_brake", a);
        check(loadAxisCurve("rudder").neutral == 0.f,
              "left_brake keys do not touch rudder");
        check(loadAxisCurve("left_brake").neutral == 0.3f, "left_brake loads its own value");
    }

    // Partial file: present keys kept, missing ones default (Review Focus 5).
    {
        writeConfig("curve.rudder.sens_pos=-0.5\ncurve.rudder.dead_zone=0.1\n");
        AxisCurve c = loadAxisCurve("rudder");
        check(c.sensPos == -0.5f && c.deadZone == 0.1f, "partial: present keys loaded");
        check(c.sensNeg == 0.f && c.neutral == 0.f && c.extremityDeadZone == 0.f, "partial: missing keys default to 0");
    }

    // Garbage values: no exception, clamped or zero (Review Focus 2).
    {
        writeConfig("curve.rudder.sens_pos=abc\ncurve.rudder.sens_neg=nan\ncurve.rudder.neutral=5\ncurve.rudder.dead_zone=-3\ncurve.rudder.extremity=\n");
        AxisCurve c = loadAxisCurve("rudder");
        check(c.sensPos == 0.f, "garbage 'abc' -> 0");
        check(c.sensNeg == 0.f, "garbage 'nan' -> 0");
        check(c.neutral == 1.f, "out of range 5 -> clamped to 1");
        check(c.deadZone == 0.f, "out of range -3 -> clamped to 0");
        check(c.extremityDeadZone == 0.f, "empty value -> 0");
    }

    // Window visibility round trip.
    {
        saveCurveWindowVisible(true);
        check(loadCurveWindowVisible() == true, "curve window visible round trip (true)");
        saveCurveWindowVisible(false);
        check(loadCurveWindowVisible() == false, "curve window visible round trip (false)");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
```

Add to `DCUProviderPlugin/tests/CMakeLists.txt` (before `enable_testing()`):

```cmake
add_executable(test_config test_config.cpp ../src/ConfigUtils.cpp ../src/AxisCurve.cpp)
target_include_directories(test_config PRIVATE ../src)
```
and after the existing `add_test`:
```cmake
add_test(NAME config COMMAND test_config)
```

- [ ] **Step 2: Run to verify it fails**

```bash
cmake -S tests -B tests/build && cmake --build tests/build
```
Expected: FAIL — `use of undeclared identifier 'loadAxisCurve'`.

- [ ] **Step 3: Implement**

`DCUProviderPlugin/src/ConfigUtils.h` — append:

```cpp
#include "AxisCurve.h"

// Response curve for one axis, stored as curve.<axis>.<param> keys.
// Missing or unparseable keys load as 0; values are clamped to range.
AxisCurve loadAxisCurve(const std::string& axis);
void saveAxisCurve(const std::string& axis, const AxisCurve& curve);

// Loads the curve window visibility. Default: hidden.
bool loadCurveWindowVisible();
void saveCurveWindowVisible(bool visible);
```

`DCUProviderPlugin/src/ConfigUtils.cpp` — add `#include <cstdio>` to the includes and append:

```cpp
namespace {

const char* const kCurveParams[] = { "sens_pos", "sens_neg", "neutral", "dead_zone", "extremity" };

float* curveField(AxisCurve& c, int i) {
    switch (i) {
        case 0: return &c.sensPos;
        case 1: return &c.sensNeg;
        case 2: return &c.neutral;
        case 3: return &c.deadZone;
        default: return &c.extremityDeadZone;
    }
}

// strtof accepts "nan"/"inf" and returns 0 on garbage; clampToRange() below
// turns NaN into 0 and clamps the rest, so nothing unsafe reaches the sim.
float parseFloat(const std::string& s) {
    return std::strtof(s.c_str(), nullptr);
}

} // namespace

AxisCurve loadAxisCurve(const std::string& axis) {
    auto config = loadConfig();
    AxisCurve c;
    for (int i = 0; i < 5; ++i) {
        auto it = config.find("curve." + axis + "." + kCurveParams[i]);
        if (it != config.end())
            *curveField(c, i) = parseFloat(it->second);
    }
    c.clampToRange();
    return c;
}

void saveAxisCurve(const std::string& axis, const AxisCurve& curve) {
    auto config = loadConfig();
    AxisCurve c = curve;
    for (int i = 0; i < 5; ++i) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.4f", *curveField(c, i));
        config["curve." + axis + "." + kCurveParams[i]] = buf;
    }
    saveConfig(config);
}

bool loadCurveWindowVisible() {
    auto config = loadConfig();
    auto it = config.find("curve_window_visible");
    if (it == config.end()) return false;
    return it->second == "1" || it->second == "true";
}

void saveCurveWindowVisible(bool visible) {
    auto config = loadConfig();
    config["curve_window_visible"] = visible ? "1" : "0";
    saveConfig(config);
}
```

- [ ] **Step 4: Run the tests**

```bash
cmake -S tests -B tests/build && cmake --build tests/build && ctest --test-dir tests/build --output-on-failure
```
Expected: both `axiscurve` and `config` pass, `0 failures`.

Note on the round-trip checks: `%.4f` reproduces `-0.35f`, `0.25f` etc. exactly after `strtof` because those literals are already the nearest float to a 4-decimal value. If a check fails by one ulp, compare with `std::fabs(a-b) < 1e-4f` instead of `==`.

- [ ] **Step 5: Build the plugin**

```bash
./build-macos.sh
```
Expected: green.

- [ ] **Step 6: Commit**

```bash
git add src/ConfigUtils.h src/ConfigUtils.cpp tests/test_config.cpp tests/CMakeLists.txt
git commit -m "feat(plugin): persist per-axis response curves in the config file

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 3: Generic menu items on `StatusWindow`

**Files:**
- Modify: `DCUProviderPlugin/src/StatusWindow.h` (public API + private members)
- Modify: `DCUProviderPlugin/src/StatusWindow.cpp` (`initialize()` menu block ~lines 78-93, `menuCallback` ~line 258)

**Interfaces:**
- Produces: `void StatusWindow::appendMenuItem(const std::string& label, std::function<void()> onSelect);` — must be called after `initialize()`.

- [ ] **Step 1: Declare the API**

In `StatusWindow.h`, in the public section after `setCommandCallback`:

```cpp
    // Append an item to the plugin's "DCU Provider" submenu. Call after
    // initialize(). The callback runs on X-Plane's main thread.
    void appendMenuItem(const std::string& label, std::function<void()> onSelect);
```

In the private members, after `XPLMMenuID pluginMenuId_;`:

```cpp
    // One entry per appended menu item; the XPLM item refcon is the index.
    std::vector<std::function<void()>> menuActions_;
```

- [ ] **Step 2: Implement**

In `StatusWindow.cpp`, replace the menu block inside `initialize()`:

```cpp
    // Create menu
    pluginMenuId_ = XPLMFindPluginsMenu();

    if (pluginMenuId_) {
        // Create a submenu for our plugin
        int subMenuIdx = XPLMAppendMenuItem(pluginMenuId_, "DCU Provider", nullptr, 1);
        XPLMMenuID ourMenuId = XPLMCreateMenu("DCU Provider", pluginMenuId_, subMenuIdx,
                                               menuCallback, this);

        // Store reference for cleanup
        menuItemIdx_ = subMenuIdx;
        pluginMenuId_ = ourMenuId;

        appendMenuItem("Show Status Window", [this]() { setVisible(!isVisible()); });
    }
```

Add the method (anywhere after `destroy()`):

```cpp
void StatusWindow::appendMenuItem(const std::string& label, std::function<void()> onSelect) {
    if (!pluginMenuId_) return;
    menuActions_.push_back(std::move(onSelect));
    void* refcon = reinterpret_cast<void*>(static_cast<intptr_t>(menuActions_.size() - 1));
    XPLMAppendMenuItem(pluginMenuId_, label.c_str(), refcon, 1);
}
```

Replace `menuCallback`:

```cpp
void StatusWindow::menuCallback(void* inMenuRef, void* inItemRef) {
    StatusWindow* self = static_cast<StatusWindow*>(inMenuRef);
    if (!self) return;
    const size_t idx = static_cast<size_t>(reinterpret_cast<intptr_t>(inItemRef));
    if (idx < self->menuActions_.size() && self->menuActions_[idx])
        self->menuActions_[idx]();
}
```

Add `#include <cstdint>` to `StatusWindow.cpp` if `intptr_t` is not already visible (the header pulls in `<cstdint>`, so it should be).

- [ ] **Step 3: Build**

```bash
./build-macos.sh
```
Expected: green. Manual check (optional now, required in Task 6): "Plugins → DCU Provider → Show Status Window" still toggles the status window.

- [ ] **Step 4: Commit**

```bash
git add src/StatusWindow.h src/StatusWindow.cpp
git commit -m "refactor(plugin): generic appendMenuItem on the DCU Provider menu

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 4: `CurveWindow` — plot, sliders, mouse

**Files:**
- Create: `DCUProviderPlugin/src/CurveWindow.h`
- Create: `DCUProviderPlugin/src/CurveWindow.cpp`
- Modify: `DCUProviderPlugin/CMakeLists.txt` (SOURCES/HEADERS)

**Interfaces:**
- Consumes: `AxisCurve` (Task 1), `loadCurveWindowVisible`/`saveCurveWindowVisible` (Task 2).
- Produces:
  ```cpp
  class CurveWindow {
      CurveWindow(std::string axisLabel, AxisCurve* curve);
      void initialize(); void destroy();
      void setVisible(bool); bool isVisible() const;
      void update();                      // ~1 Hz
      void setLiveInput(float raw);       // per rudder frame
      void setChangedCallback(std::function<void()> cb);
  };
  ```

No native test: the whole class is XPLM/GL-bound. The one pure helper (`valueFromX`) is three lines and is exercised by the manual checklist in Task 6.

- [ ] **Step 1: Header**

`DCUProviderPlugin/src/CurveWindow.h`:

```cpp
#pragma once
#include "XPLMDisplay.h"
#include "AxisCurve.h"
#include <functional>
#include <string>
#include <vector>

// Floating window that edits one AxisCurve in place: a plot of the curve with
// a live marker of the current axis input on the left, five draggable sliders
// (one per AxisCurve parameter) with reset buttons on the right.
//
// The curve pointer is written directly on every drag, so the change is live
// on the next axis frame. changedCallback_ fires on slider release and reset
// so the owner can persist it.
class CurveWindow {
public:
    CurveWindow(std::string axisLabel, AxisCurve* curve);
    ~CurveWindow();

    CurveWindow(const CurveWindow&) = delete;
    CurveWindow& operator=(const CurveWindow&) = delete;

    // Create the XPLM window. Visibility is restored from the config file.
    void initialize();
    void destroy();

    void setVisible(bool visible);
    bool isVisible() const;

    // Call ~1 Hz: detects the native close button and persists visibility.
    void update();

    // Current raw axis input (-1..1) for the live marker. Call per axis frame.
    void setLiveInput(float raw);

    // Fired on slider release and on reset. Owner persists the curve here.
    void setChangedCallback(std::function<void()> cb);

private:
    enum Param { SENS_POS = 0, SENS_NEG, NEUTRAL, DEAD_ZONE, EXTREMITY, PARAM_COUNT };

    struct Row { const char* label; float min; float max; };
    static const Row kRows[PARAM_COUNT];

    // Clickable region, rebuilt every draw() frame. reset=false is a slider
    // track, reset=true is that row's [R] button.
    struct Hit { int left, top, right, bottom; int param; bool reset; };

    float getParam(int p) const;
    void  setParam(int p, float v);
    static float valueFromX(int x, int trackLeft, int trackWidth, float min, float max);

    static void drawCallback(XPLMWindowID inWindowID, void* inRefcon);
    static void keyCallback(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags,
                            char inVirtualKey, void* inRefcon, int losingFocus);
    static int  mouseCallback(XPLMWindowID inWindowID, int x, int y,
                              XPLMMouseStatus inMouse, void* inRefcon);

    void draw();
    void drawPlot(int left, int top, int size);
    void drawSliders(int x, int y);
    void drawString(int x, int y, const std::string& text, float r, float g, float b);

    std::string axisLabel_;
    AxisCurve* curve_;
    std::function<void()> changedCallback_;
    XPLMWindowID windowId_ = nullptr;
    std::vector<Hit> hits_;

    // Slider currently being dragged, -1 when none; track geometry captured
    // at MouseDown so MouseDrag doesn't depend on hits_ being rebuilt.
    int grabbed_ = -1;
    int grabLeft_ = 0;
    int grabWidth_ = 1;

    float liveInput_ = 0.f;
    bool lastKnownVisible_ = false;
};
```

- [ ] **Step 2: Implementation**

`DCUProviderPlugin/src/CurveWindow.cpp`:

```cpp
#include "CurveWindow.h"
#include "ConfigUtils.h"
#include "XPLMGraphics.h"
#include "XPLMUtilities.h"
#include <algorithm>
#include <cstdio>

#if APL
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl.h>
#elif IBM
#include <windows.h>
#include <GL/gl.h>
#else
#include <GL/gl.h>
#endif

namespace {
constexpr int kWinW = 620, kWinH = 320;
constexpr int kPlotSize = 240;
constexpr int kTrackW = 150, kTrackH = 10;
constexpr int kRowStep = 30;
constexpr int kCurveSamples = 65;
}

const CurveWindow::Row CurveWindow::kRows[CurveWindow::PARAM_COUNT] = {
    { "Sensitivity +", -1.f, 1.f },
    { "Sensitivity -", -1.f, 1.f },
    { "Neutral",       -1.f, 1.f },
    { "Dead zone",      0.f, 0.5f },
    { "Extremity DZ",   0.f, 0.5f },
};

CurveWindow::CurveWindow(std::string axisLabel, AxisCurve* curve)
    : axisLabel_(std::move(axisLabel)), curve_(curve) {}

CurveWindow::~CurveWindow() { destroy(); }

void CurveWindow::setChangedCallback(std::function<void()> cb) { changedCallback_ = std::move(cb); }
void CurveWindow::setLiveInput(float raw) { liveInput_ = raw; }

float CurveWindow::getParam(int p) const {
    switch (p) {
        case SENS_POS:  return curve_->sensPos;
        case SENS_NEG:  return curve_->sensNeg;
        case NEUTRAL:   return curve_->neutral;
        case DEAD_ZONE: return curve_->deadZone;
        default:        return curve_->extremityDeadZone;
    }
}

void CurveWindow::setParam(int p, float v) {
    v = std::max(kRows[p].min, std::min(kRows[p].max, v));
    switch (p) {
        case SENS_POS:  curve_->sensPos = v; break;
        case SENS_NEG:  curve_->sensNeg = v; break;
        case NEUTRAL:   curve_->neutral = v; break;
        case DEAD_ZONE: curve_->deadZone = v; break;
        default:        curve_->extremityDeadZone = v; break;
    }
}

float CurveWindow::valueFromX(int x, int trackLeft, int trackWidth, float min, float max) {
    float t = static_cast<float>(x - trackLeft) / static_cast<float>(std::max(1, trackWidth));
    t = std::max(0.f, std::min(1.f, t));
    return min + t * (max - min);
}

void CurveWindow::initialize() {
    const bool shouldBeVisible = loadCurveWindowVisible();

    XPLMCreateWindow_t params = {};
    params.structSize = sizeof(XPLMCreateWindow_t);
    params.left = 150;
    params.top = 600;
    params.right = 150 + kWinW;
    params.bottom = 600 - kWinH;
    params.visible = shouldBeVisible ? 1 : 0;
    params.drawWindowFunc = drawCallback;
    params.handleKeyFunc = keyCallback;
    params.handleMouseClickFunc = mouseCallback;
    params.handleCursorFunc = nullptr;
    params.handleMouseWheelFunc = nullptr;
    params.handleRightClickFunc = nullptr;
    params.refcon = this;
    params.layer = xplm_WindowLayerFloatingWindows;
    params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;

    windowId_ = XPLMCreateWindowEx(&params);
    if (!windowId_) {
        XPLMDebugString("DCUProvider: Failed to create curve window\n");
        return;
    }
    XPLMSetWindowTitle(windowId_, (axisLabel_ + " Curve").c_str());
    lastKnownVisible_ = shouldBeVisible;
    XPLMDebugString("DCUProvider: Curve window initialized\n");
}

void CurveWindow::destroy() {
    if (windowId_) {
        XPLMDestroyWindow(windowId_);
        windowId_ = nullptr;
    }
}

void CurveWindow::setVisible(bool visible) {
    if (!windowId_) return;
    XPLMSetWindowIsVisible(windowId_, visible ? 1 : 0);
    lastKnownVisible_ = visible;
    saveCurveWindowVisible(visible);
}

bool CurveWindow::isVisible() const {
    return windowId_ && XPLMGetWindowIsVisible(windowId_) != 0;
}

void CurveWindow::update() {
    if (!windowId_) return;
    // The native close button has no callback; poll for it like StatusWindow does.
    const bool nowVisible = isVisible();
    if (nowVisible != lastKnownVisible_) {
        lastKnownVisible_ = nowVisible;
        saveCurveWindowVisible(nowVisible);
    }
}

// ---------------- X-Plane callbacks ----------------

void CurveWindow::drawCallback(XPLMWindowID, void* inRefcon) {
    if (auto* self = static_cast<CurveWindow*>(inRefcon)) self->draw();
}

void CurveWindow::keyCallback(XPLMWindowID, char inKey, XPLMKeyFlags, char, void* inRefcon, int) {
    auto* self = static_cast<CurveWindow*>(inRefcon);
    if (self && inKey == 27) self->setVisible(false);   // ESC hides
}

int CurveWindow::mouseCallback(XPLMWindowID, int x, int y, XPLMMouseStatus inMouse, void* inRefcon) {
    auto* self = static_cast<CurveWindow*>(inRefcon);
    if (!self) return 1;

    if (inMouse == xplm_MouseDown) {
        for (const Hit& h : self->hits_) {
            if (x < h.left || x > h.right || y < h.bottom || y > h.top) continue;
            if (h.reset) {
                self->setParam(h.param, 0.f);
                if (self->changedCallback_) self->changedCallback_();
            } else {
                self->grabbed_ = h.param;
                self->grabLeft_ = h.left;
                self->grabWidth_ = h.right - h.left;
                self->setParam(h.param, valueFromX(x, h.left, h.right - h.left,
                                                   kRows[h.param].min, kRows[h.param].max));
            }
            return 1;
        }
    } else if (inMouse == xplm_MouseDrag) {
        if (self->grabbed_ >= 0) {
            const int p = self->grabbed_;
            self->setParam(p, valueFromX(x, self->grabLeft_, self->grabWidth_, kRows[p].min, kRows[p].max));
        }
    } else if (inMouse == xplm_MouseUp) {
        if (self->grabbed_ >= 0) {
            self->grabbed_ = -1;
            if (self->changedCallback_) self->changedCallback_();
        }
    }
    return 1; // consume, so drags keep coming to this window
}

// ---------------- drawing ----------------

void CurveWindow::draw() {
    if (!windowId_) return;
    hits_.clear();

    int left, top, right, bottom;
    XPLMGetWindowGeometry(windowId_, &left, &top, &right, &bottom);
    XPLMDrawTranslucentDarkBox(left, top, right, bottom);

    drawString(left + 10, top - 20, axisLabel_ + " axis curve", 0.8f, 1.0f, 0.8f);
    drawPlot(left + 10, top - 35, kPlotSize);
    drawSliders(left + 10 + kPlotSize + 20, top - 50);

    char line[64];
    std::snprintf(line, sizeof(line), "in %+.2f -> out %+.2f", liveInput_, curve_->apply(liveInput_));
    drawString(left + 10, bottom + 12, line, 0.7f, 0.9f, 1.0f);
}

void CurveWindow::drawPlot(int left, int top, int size) {
    const int bottom = top - size;
    const int right = left + size;
    auto px = [&](float v) { return left + (v + 1.f) * 0.5f * size; };
    auto py = [&](float v) { return bottom + (v + 1.f) * 0.5f * size; };

    XPLMSetGraphicsState(0, 0, 0, 0, 0, 0, 0);

    // Frame + axis cross
    glColor3f(0.35f, 0.35f, 0.35f);
    glBegin(GL_LINE_LOOP);
    glVertex2i(left, bottom); glVertex2i(right, bottom); glVertex2i(right, top); glVertex2i(left, top);
    glEnd();
    glBegin(GL_LINES);
    glVertex2f(px(-1.f), py(0.f)); glVertex2f(px(1.f), py(0.f));
    glVertex2f(px(0.f), py(-1.f)); glVertex2f(px(0.f), py(1.f));
    glEnd();

    // Curve
    glColor3f(0.4f, 0.8f, 1.0f);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < kCurveSamples; ++i) {
        const float x = -1.f + 2.f * i / (kCurveSamples - 1);
        glVertex2f(px(x), py(curve_->apply(x)));
    }
    glEnd();

    // Live marker
    const float mx = px(std::max(-1.f, std::min(1.f, liveInput_)));
    const float my = py(curve_->apply(liveInput_));
    glColor3f(1.0f, 0.9f, 0.3f);
    glBegin(GL_QUADS);
    glVertex2f(mx - 3, my - 3); glVertex2f(mx + 3, my - 3); glVertex2f(mx + 3, my + 3); glVertex2f(mx - 3, my + 3);
    glEnd();
}

void CurveWindow::drawSliders(int x, int y) {
    const int labelW = 100;
    for (int p = 0; p < PARAM_COUNT; ++p) {
        const Row& row = kRows[p];
        const float v = getParam(p);
        const int trackLeft = x + labelW;
        const int trackBottom = y - 2;
        const int trackTop = trackBottom + kTrackH;

        drawString(x, y, row.label, 0.8f, 0.8f, 0.8f);

        XPLMSetGraphicsState(0, 0, 0, 0, 0, 0, 0);
        glColor3f(0.25f, 0.3f, 0.4f);
        glBegin(GL_QUADS);
        glVertex2i(trackLeft, trackBottom); glVertex2i(trackLeft + kTrackW, trackBottom);
        glVertex2i(trackLeft + kTrackW, trackTop); glVertex2i(trackLeft, trackTop);
        glEnd();

        const float t = (v - row.min) / (row.max - row.min);
        const int knobX = trackLeft + static_cast<int>(t * kTrackW);
        glColor3f(0.4f, 0.7f, 1.0f);
        glBegin(GL_QUADS);
        glVertex2i(knobX - 3, trackBottom - 3); glVertex2i(knobX + 3, trackBottom - 3);
        glVertex2i(knobX + 3, trackTop + 3);    glVertex2i(knobX - 3, trackTop + 3);
        glEnd();

        // Generous hitbox so the thin track is easy to grab.
        hits_.push_back({ trackLeft - 4, trackTop + 6, trackLeft + kTrackW + 4, trackBottom - 6, p, false });

        char val[16];
        std::snprintf(val, sizeof(val), "%+.2f", v);
        drawString(trackLeft + kTrackW + 10, y, val, 1.0f, 1.0f, 1.0f);

        const int rx = trackLeft + kTrackW + 60;
        drawString(rx, y, "[R]", 0.7f, 0.8f, 0.9f);
        hits_.push_back({ rx, y + 12, rx + 24, y - 3, p, true });

        y -= kRowStep;
    }
}

void CurveWindow::drawString(int x, int y, const std::string& text, float r, float g, float b) {
    float color[3] = { r, g, b };
    XPLMDrawString(color, x, y, const_cast<char*>(text.c_str()), nullptr, xplmFont_Basic);
}
```

- [ ] **Step 3: Add to the plugin build**

`DCUProviderPlugin/CMakeLists.txt`: add `src/CurveWindow.cpp` to `SOURCES` and `src/CurveWindow.h` to `HEADERS`.

```bash
./build-macos.sh
```
Expected: green. If macOS complains about `glBegin` being unavailable, confirm `GL_SILENCE_DEPRECATION` is defined before the include (it is in the snippet above). If `XPLMSetWindowTitle` is reported undeclared, the XPLM300 define is missing — it is set in `CMakeLists.txt`, so this should not happen.

- [ ] **Step 4: Commit**

```bash
git add src/CurveWindow.h src/CurveWindow.cpp CMakeLists.txt
git commit -m "feat(plugin): curve window with plot and draggable sliders

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 5: Wire the curve into `DCUProvider`

**Files:**
- Modify: `DCUProviderPlugin/src/DCUProvider.h` (includes, private members)
- Modify: `DCUProviderPlugin/src/DCUProvider.cpp` (`initialize()` ~line 53-65, `shutdown()` ~line 200, `onFlightLoopTick()` 1 Hz block ~line 262, `updateUplink()` rudder case ~line 642)

**Interfaces:**
- Consumes: `AxisCurve`, `loadAxisCurve`/`saveAxisCurve`, `CurveWindow`, `StatusWindow::appendMenuItem`.
- Produces: nothing new outward.

- [ ] **Step 1: Header changes**

In `DCUProvider.h`, add to the includes:

```cpp
#include "CurveWindow.h"
#include "AxisCurve.h"
```

In the `// ============ Components ============` block after `std::unique_ptr<StatusWindow> statusWin_;`:

```cpp
    // Rudder pedal response curve, edited live by curveWin_ and persisted
    // under curve.rudder.* in the config file. Toe brakes stay linear.
    AxisCurve rudderCurve_;
    std::unique_ptr<CurveWindow> curveWin_;
```

- [ ] **Step 2: `initialize()` — load, create window, menu item**

In `DCUProvider.cpp` `initialize()`, directly after the `statusWin_->setCommandCallback(...)` call:

```cpp
    rudderCurve_ = loadAxisCurve("rudder");
    curveWin_ = std::make_unique<CurveWindow>("Rudder", &rudderCurve_);
    curveWin_->initialize();
    curveWin_->setChangedCallback([this]()
                                  { saveAxisCurve("rudder", rudderCurve_); });
    statusWin_->appendMenuItem("Rudder Curve...", [this]()
                               { if (curveWin_) curveWin_->setVisible(!curveWin_->isVisible()); });
```

- [ ] **Step 3: `shutdown()` — destroy before the status window**

In `shutdown()`, before the `if (statusWin_)` block:

```cpp
    if (curveWin_)
    {
        curveWin_->destroy();
        curveWin_.reset();
    }
```

- [ ] **Step 4: 1 Hz update**

In `onFlightLoopTick()`, inside the `if (statusUpdateAccum >= 1.0f)` block, after `updateStatusWindow();`:

```cpp
        if (curveWin_)
            curveWin_->update();
```

- [ ] **Step 5: Apply the curve in the rudder case**

In `updateUplink()`, case `MessageType::SerialMessageRudder`, replace

```cpp
                dataRefMgr_->setRudderRatio(static_cast<float>(message->rudder) / 1000.0f);
```
with
```cpp
                const float rudderRaw = static_cast<float>(message->rudder) / 1000.0f;
                if (curveWin_)
                    curveWin_->setLiveInput(rudderRaw);
                dataRefMgr_->setRudderRatio(rudderCurve_.apply(rudderRaw));
```

The two brake lines stay as they are.

- [ ] **Step 6: Build and run the native tests once more**

```bash
./build-macos.sh
cmake -S tests -B tests/build && cmake --build tests/build && ctest --test-dir tests/build --output-on-failure
```
Expected: plugin green, both tests pass.

- [ ] **Step 7: Commit**

```bash
git add src/DCUProvider.h src/DCUProvider.cpp
git commit -m "feat(plugin): apply the tunable response curve to the rudder axis

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 6: Docs + manual bench checklist

**Files:**
- Modify: `DCUProviderPlugin/CLAUDE.md` (Commands section: tests; Architecture: data flow item 4, config persistence)
- Modify: `DCUProviderPlugin/src/StatusWindow.cpp` (title string, optional version bump)

- [ ] **Step 1: Update `DCUProviderPlugin/CLAUDE.md`**

Replace the paragraph starting `No automated test suite currently exists` with:

```markdown
Native unit tests (no X-Plane needed) live in `tests/`, framework-free `check()` executables
driven by CTest, same shape as `MotionProviderPlugin/tests`:

```bash
cmake -S tests -B tests/build && cmake --build tests/build && ctest --test-dir tests/build --output-on-failure
```

They cover the XPLM-free pieces only: `AxisCurve` (rudder response curve math) and
`ConfigUtils` (key=value persistence, including the per-axis curve keys). Windows,
datarefs and the serial path are verified manually.
```

In the data-flow list, item 4, append to the sentence about the rudder axes:

```markdown
   The rudder value passes through `AxisCurve rudderCurve_` (MSFS-style sensitivity /
   neutral / dead-zone curve, see `docs/superpowers/specs/2026-10-11-rudder-axis-curve-design.md`)
   before `setRudderRatio()`; toe brakes stay linear.
```

Add after the `StatusWindow` paragraph:

```markdown
`CurveWindow` is the second floating window ("Plugins → DCU Provider → Rudder Curve..."): it
edits `rudderCurve_` in place through a pointer, so slider drags are live on the next rudder
frame, and fires a changed-callback on release/reset that `DCUProvider` answers with
`saveAxisCurve("rudder", ...)`. `StatusWindow` owns the plugin submenu and exposes
`appendMenuItem()` for anything else that wants an entry there.
```

In the config-persistence paragraph, replace `currently just last-used serial port and status window visibility` with:

```markdown
last-used serial port, both window visibilities (`window_visible`, `curve_window_visible`)
and the per-axis response curve under `curve.<axis>.{sens_pos,sens_neg,neutral,dead_zone,extremity}`
(only `rudder` today; a second axis is another prefix plus an `apply()` call at its setter).
```

- [ ] **Step 2: Bump the status window title**

In `StatusWindow.cpp` `drawText()`, change `"DCU Provider Status v0.4"` to `"DCU Provider Status v0.5"`.

- [ ] **Step 3: Manual bench checklist (run with X-Plane + rig, record results in the commit or a follow-up note)**

1. Fresh config: delete or back up `~/.dcuprovider.cfg`. Start X-Plane. Curve window hidden, status window state as before.
2. "Plugins → DCU Provider → Rudder Curve..." opens the window. Plot shows a straight diagonal, all sliders at 0 / `+0.00`.
3. Press pedals: yellow marker rides the diagonal, bottom line shows `in x -> out x`.
4. Drag "Sensitivity +" to −1: right half of the plot flattens around the centre; pedal right half-way yields a visibly smaller yoke deflection in the sim (check `sim/joystick/yoke_heading_ratio` in DataRefTool or the rudder animation).
5. Drag "Dead zone" to 0.2: marker stays at out 0 for small pedal inputs.
6. Drag "Extremity DZ" to 0.3: marker hits ±1 well before the pedal stops.
7. Drag "Neutral" to +0.3: zero crossing moves right on the plot.
8. Each `[R]` resets its row to 0.
9. After each release `cat ~/.dcuprovider.cfg` shows updated `curve.rudder.*` lines; `curve_window_visible=1`.
10. Close via the window's native X: `curve_window_visible=0` within ~1 s. Reopen via menu, ESC closes it.
11. Restart X-Plane: slider values and window visibility restored.
12. Status window: unchanged, "Show Status Window" still toggles it.
13. Unplug the DCU: rudder override releases after 3 s as before (watchdog untouched).

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md src/StatusWindow.cpp
git commit -m "docs(plugin): rudder curve window, native test harness

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

- [ ] **Step 5 (optional, if MinGW toolchain is installed): Windows cross build**

```bash
./build-xc-windows.sh
```
Expected: green. The only platform-sensitive code is the GL include block in `CurveWindow.cpp` (`<windows.h>` before `<GL/gl.h>` under `IBM`).
