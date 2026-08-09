# Rudder Axis Smoothing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop the RudderCAN axis values from jittering during pedal movement, without adding perceptible lag.

**Architecture:** Sampling is decoupled from CAN reporting. Each of the three analog axes is oversampled at 500 Hz into a speed-adaptive exponential moving average whose state is kept in Q6 fixed point. That sub-LSB resolution is carried into the axis mapping, which lets the change threshold drop from 8 to 2. Both the filter and the mapping become header-only, Arduino-free units so they can be unit tested natively, mirroring the `DCU/include/` pattern.

**Tech Stack:** C++ (Arduino / AVR, ATmega328), PlatformIO, Unity test framework, integer-only fixed-point math (no float, no heap).

Spec: `docs/superpowers/specs/2026-08-09-rudder-smoothing-design.md`

## Global Constraints

- Target is an ATmega328 (`env:nano`, `env:diecimilaatmega328`). No `float`, no `String`, no dynamic allocation in the signal path — heap churn and stack depth are what froze the CAN link on other AVR nodes in this repo.
- Fixed point format is **Q6**: one raw ADC count equals 64 units. Raw range is 0..1023, so the maximum Q6 value is `1023 << 6 = 65472`. All fixed-point arithmetic uses `int32_t`; the worst-case intermediate is `65472 * 1000 = 65,472,000`, well inside `INT32_MAX`.
- Filter tuning constants: `alphaMin = 32` (Q8), `slope = 12`, sample interval `2` ms.
- Change thresholds after this work: `kRudderChangeThreshold = 2`, `kBrakeChangeThreshold = 2`.
- `kRudderCenterDeadbandRaw = 12` (raw LSB) keeps its value — it covers mechanical slop in the pedal linkage, not electrical noise. It is only rescaled to Q6 at the point of use.
- The CAN wire format (`CanMessageId::rudder`, big-endian, `-1000..1000` / `0..1000`) does not change. Neither DCU nor `DCUProviderPlugin` is touched.
- Calibration paths (`sampleAverage`, every `calibrateXxx`, `getRawRudder`/`getRawLeftBrake`/`getRawRightBrake`) keep reading the ADC **directly and unfiltered**.
- Comments in this repo are written in English. Match the density and tone of the surrounding code.
- `pio` lives at `~/.platformio/penv/bin/pio`.
- Work happens on branch `feature/rudder-smoothing`, which already exists and holds the spec commit.

## File Structure

| File | Responsibility |
| --- | --- |
| `RudderCAN/include/AdaptiveFilter.h` (create) | Speed-adaptive EMA over one raw ADC channel. No Arduino dependency. |
| `RudderCAN/include/AxisMapping.h` (create) | Pure Q6 → wire-scale mapping for the bipolar rudder half-axes and the unipolar brake axes. No Arduino dependency. |
| `RudderCAN/test/test_adaptive_filter/test_adaptive_filter.cpp` (create) | Unity suite for the filter. |
| `RudderCAN/test/test_axis_mapping/test_axis_mapping.cpp` (create) | Unity suite for the mapping. |
| `RudderCAN/platformio.ini` (modify) | Add a `native` env; move AVR-common settings out of `[env]` so `native` does not inherit them. |
| `RudderCAN/src/Rudder.h` / `Rudder.cpp` (modify) | Own the three filters, expose `sample()`, map from filter state, retune thresholds. |
| `RudderCAN/src/CAN.cpp` (modify) | Call `sample()` every loop pass. |
| `RudderCAN/src/BenchDebug.cpp` (modify) | Call `sample()` every loop pass; print the filtered Q6 value. |

---

### Task 1: `AdaptiveFilter` + native test environment

**Files:**
- Create: `RudderCAN/include/AdaptiveFilter.h`
- Create: `RudderCAN/test/test_adaptive_filter/test_adaptive_filter.cpp`
- Modify: `RudderCAN/platformio.ini`

**Interfaces:**
- Consumes: nothing.
- Produces: `class AdaptiveFilter` with `AdaptiveFilter(int32_t alphaMin = 32, int32_t slope = 12)`, `void reset(uint16_t raw)`, `void update(uint16_t raw)`, `bool seeded() const`, `int32_t valueQ6() const`. Also a `native` PlatformIO env that later tasks reuse.

The build environment is folded into this task because the first test cannot run without it.

- [ ] **Step 1: Restructure `RudderCAN/platformio.ini`**

The existing `[env]` section propagates `platform = atmelavr` and `framework = arduino` to *every* environment, which would break a native env. Move those settings into a plain `[avr]` section that only the board envs extend. Replace the whole file body below the comment header with:

```ini
[avr]
framework = arduino
platform = atmelavr
monitor_speed = 115200
lib_deps =
    coryjfowler/mcp_can@^1.5.1
lib_extra_dirs =
  ../shared

[env:nano]
extends = avr
board = nanoatmega328new

[env:diecimilaatmega328]
extends = avr
board = diecimilaatmega328

[env:native]
platform = native
test_framework = unity
build_src_filter = -<*>
```

`build_src_filter = -<*>` keeps the Arduino sources out of the host build; PlatformIO still puts `RudderCAN/include/` on the include path. This mirrors `DCU/platformio.ini:20-24`, minus the `-I../shared/CANBase/include` flag, which is not needed here.

- [ ] **Step 2: Verify the AVR builds still work after the restructure**

Run from `RudderCAN/`:

```bash
~/.platformio/penv/bin/pio run -e nano
~/.platformio/penv/bin/pio run -e diecimilaatmega328
```

Expected: both SUCCESS. If they fail with a missing-package error, the sandbox has no network — report that and stop rather than working around it.

- [ ] **Step 3: Write the failing test**

Create `RudderCAN/test/test_adaptive_filter/test_adaptive_filter.cpp`:

```cpp
#include <unity.h>
#include "AdaptiveFilter.h"

void setUp(void) {}
void tearDown(void) {}

// One raw ADC count in Q6.
static const int32_t kQ6 = 64;

void test_reset_seeds_exactly(void) {
    AdaptiveFilter f;
    f.reset(500);
    TEST_ASSERT_TRUE(f.seeded());
    TEST_ASSERT_EQUAL_INT32(500 * kQ6, f.valueQ6());
}

void test_first_update_seeds_without_ramp(void) {
    AdaptiveFilter f;
    TEST_ASSERT_FALSE(f.seeded());
    f.update(500);
    TEST_ASSERT_TRUE(f.seeded());
    TEST_ASSERT_EQUAL_INT32(500 * kQ6, f.valueQ6());
}

void test_small_step_converges_without_overshoot(void) {
    // Deliberately a 5 LSB step, small enough that alpha stays off its ceiling
    // and the convergence behaviour is actually exercised.
    AdaptiveFilter f;
    f.reset(500);
    int32_t previous = f.valueQ6();
    for (int i = 0; i < 200; i++) {
        f.update(505);
        // Monotonic rise, and never past the target.
        TEST_ASSERT_TRUE(f.valueQ6() >= previous);
        TEST_ASSERT_TRUE(f.valueQ6() <= 505 * kQ6);
        previous = f.valueQ6();
    }
    // Settled to within one LSB of the target.
    TEST_ASSERT_TRUE((505 * kQ6) - f.valueQ6() < kQ6);
}

void test_alpha_saturates_on_full_scale_jump(void) {
    // A 0 -> 1023 step drives alpha to its 256/256 ceiling, so the filter is
    // transparent: the estimate must land exactly on the input, with no overflow.
    AdaptiveFilter f;
    f.reset(0);
    f.update(1023);
    TEST_ASSERT_EQUAL_INT32(1023 * kQ6, f.valueQ6());
}

void test_noise_is_attenuated_below_one_lsb(void) {
    AdaptiveFilter f;
    f.reset(512);
    for (int i = 0; i < 100; i++) {
        f.update((i % 2) ? 514 : 510);
    }
    int32_t lo = f.valueQ6();
    int32_t hi = f.valueQ6();
    for (int i = 0; i < 100; i++) {
        f.update((i % 2) ? 514 : 510);
        if (f.valueQ6() < lo) lo = f.valueQ6();
        if (f.valueQ6() > hi) hi = f.valueQ6();
    }
    // +-2 LSB of input dither must collapse to under 1 LSB of output span.
    TEST_ASSERT_TRUE((hi - lo) < kQ6);
}

void test_ramp_tracking_error_stays_small(void) {
    // 4 LSB per sample at 500 Hz is a full-travel sweep in about half a second.
    AdaptiveFilter f;
    f.reset(0);
    uint16_t raw = 0;
    for (int i = 0; i < 50; i++) {
        raw = (uint16_t)(raw + 4);
        f.update(raw);
    }
    const int32_t errorLsb = ((int32_t)raw * kQ6 - f.valueQ6()) / kQ6;
    TEST_ASSERT_TRUE(errorLsb >= 0);
    TEST_ASSERT_TRUE(errorLsb < 12);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_reset_seeds_exactly);
    RUN_TEST(test_first_update_seeds_without_ramp);
    RUN_TEST(test_small_step_converges_without_overshoot);
    RUN_TEST(test_alpha_saturates_on_full_scale_jump);
    RUN_TEST(test_noise_is_attenuated_below_one_lsb);
    RUN_TEST(test_ramp_tracking_error_stays_small);
    return UNITY_END();
}
```

- [ ] **Step 4: Run the test to verify it fails**

Run from `RudderCAN/`:

```bash
~/.platformio/penv/bin/pio test -e native
```

Expected: FAIL — compile error, `AdaptiveFilter.h: No such file or directory`.

- [ ] **Step 5: Write the implementation**

Create `RudderCAN/include/AdaptiveFilter.h`:

```cpp
#ifndef ADAPTIVE_FILTER_H
#define ADAPTIVE_FILTER_H

#include <stdint.h>

// Speed-adaptive exponential moving average over one raw ADC channel.
//
// The estimate is kept in Q6 fixed point (raw << 6) so it carries sub-LSB
// resolution into the axis mapping instead of being rounded back to whole ADC
// counts. The smoothing factor rises with the distance between the incoming
// sample and the current estimate: standing still it stays at `alphaMin` and
// smooths hard, on fast movement it saturates at 256/256 and the filter becomes
// transparent. That is what keeps the axis quiet at rest without adding lag
// when a pedal is actually moved.
class AdaptiveFilter
{
public:
    // alphaMin: Q8 smoothing factor applied when the input is not moving.
    // slope:    added to alpha per LSB of deviation between input and estimate.
    AdaptiveFilter(int32_t alphaMin = 32, int32_t slope = 12)
        : _alphaMin(alphaMin), _slope(slope), _q6(0), _seeded(false) {}

    // Jumps the estimate straight to `raw`, skipping the settling ramp.
    void reset(uint16_t raw)
    {
        _q6 = (int32_t)raw << 6;
        _seeded = true;
    }

    void update(uint16_t raw)
    {
        if (!_seeded) {
            reset(raw);
            return;
        }

        const int32_t target = (int32_t)raw << 6;
        const int32_t d      = target - _q6;
        const int32_t dLsb   = ((d < 0) ? -d : d) >> 6;

        int32_t alpha = _alphaMin + _slope * dLsb;
        if (alpha > 256) {
            alpha = 256;
        }

        // Division, not a shift: it truncates towards zero, so the residual
        // sub-step deadband is symmetric. An arithmetic shift would floor and
        // leave the estimate creeping downwards on small negative deltas.
        _q6 += (d * alpha) / 256;
    }

    bool seeded() const { return _seeded; }

    // Filtered value in Q6 (raw << 6).
    int32_t valueQ6() const { return _q6; }

private:
    int32_t _alphaMin;
    int32_t _slope;
    int32_t _q6;
    bool    _seeded;
};

#endif // ADAPTIVE_FILTER_H
```

- [ ] **Step 6: Run the test to verify it passes**

```bash
~/.platformio/penv/bin/pio test -e native
```

Expected: PASS, 6 tests, 0 failures.

- [ ] **Step 7: Commit**

```bash
git add RudderCAN/include/AdaptiveFilter.h RudderCAN/test/test_adaptive_filter/test_adaptive_filter.cpp RudderCAN/platformio.ini
git commit -m "feat(RudderCAN): add speed-adaptive EMA filter"
```

---

### Task 2: Q6 axis mapping

**Files:**
- Create: `RudderCAN/include/AxisMapping.h`
- Create: `RudderCAN/test/test_axis_mapping/test_axis_mapping.cpp`

**Interfaces:**
- Consumes: the `native` env from Task 1.
- Produces: `int32_t rawToQ6(uint16_t raw)`, `int32_t mapToWire(int32_t v, int32_t fromV, int32_t toV)`, `uint16_t mapHalfQ6(int32_t vQ6, int32_t centerQ6, int32_t endQ6, int32_t centerDeadbandQ6)`, `uint16_t mapUnipolarQ6(int32_t vQ6, int32_t loQ6, int32_t hiQ6)`, `int16_t mapRudderQ6(int32_t vQ6, int32_t minQ6, int32_t centerQ6, int32_t maxQ6, int32_t centerDeadbandQ6)`.

This is a move of the existing `static` helpers out of `Rudder.cpp` into a testable header, converted to Q6. The Arduino `map()` and `constrain()` used today are not available on the host, so `mapToWire` replaces them.

- [ ] **Step 1: Write the failing test**

Create `RudderCAN/test/test_axis_mapping/test_axis_mapping.cpp`:

```cpp
#include <unity.h>
#include "AxisMapping.h"

void setUp(void) {}
void tearDown(void) {}

// A representative small-swing hall sensor: 512 centre, +-150 counts of travel.
static const int32_t kCenter   = 512 * 64;
static const int32_t kMax      = 662 * 64;
static const int32_t kMin      = 362 * 64;
static const int32_t kDeadband =  12 * 64;

void test_rawToQ6_scales_by_64(void) {
    TEST_ASSERT_EQUAL_INT32(0, rawToQ6(0));
    TEST_ASSERT_EQUAL_INT32(65472, rawToQ6(1023));
}

void test_centre_deadband_reports_zero(void) {
    TEST_ASSERT_EQUAL_INT16(0, mapRudderQ6(kCenter, kMin, kCenter, kMax, kDeadband));
    TEST_ASSERT_EQUAL_INT16(0, mapRudderQ6(kCenter + kDeadband, kMin, kCenter, kMax, kDeadband));
    TEST_ASSERT_EQUAL_INT16(0, mapRudderQ6(kCenter - kDeadband, kMin, kCenter, kMax, kDeadband));
}

void test_endstops_reach_full_deflection(void) {
    TEST_ASSERT_EQUAL_INT16( 1000, mapRudderQ6(kMax, kMin, kCenter, kMax, kDeadband));
    TEST_ASSERT_EQUAL_INT16(-1000, mapRudderQ6(kMin, kMin, kCenter, kMax, kDeadband));
}

void test_inverted_wiring_still_maps_positive_towards_max(void) {
    // Sensor wired so the raw value falls as the pedal is pushed right.
    const int32_t invMax = 362 * 64;
    const int32_t invMin = 662 * 64;
    TEST_ASSERT_EQUAL_INT16( 1000, mapRudderQ6(invMax, invMin, kCenter, invMax, kDeadband));
    TEST_ASSERT_EQUAL_INT16(-1000, mapRudderQ6(invMin, invMin, kCenter, invMax, kDeadband));
}

void test_unipolar_spans_zero_to_thousand(void) {
    const int32_t lo = 100 * 64;
    const int32_t hi = 900 * 64;
    TEST_ASSERT_EQUAL_UINT16(   0, mapUnipolarQ6(lo, lo, hi));
    TEST_ASSERT_EQUAL_UINT16(1000, mapUnipolarQ6(hi, lo, hi));
    TEST_ASSERT_EQUAL_UINT16( 500, mapUnipolarQ6((lo + hi) / 2, lo, hi));
}

void test_zero_span_calibration_does_not_divide_by_zero(void) {
    TEST_ASSERT_EQUAL_UINT16(0, mapUnipolarQ6(5000, 5000, 5000));
    TEST_ASSERT_EQUAL_UINT16(0, mapHalfQ6(5000, 5000, 5000, kDeadband));
}

void test_sub_lsb_input_change_moves_the_wire_value(void) {
    // A quarter of an ADC count apart. Before the Q6 conversion both inputs
    // rounded to the same raw count and produced an identical wire value.
    const int16_t a = mapRudderQ6(35000, kMin, kCenter, kMax, kDeadband);
    const int16_t b = mapRudderQ6(35016, kMin, kCenter, kMax, kDeadband);
    TEST_ASSERT_NOT_EQUAL(a, b);
    TEST_ASSERT_TRUE(b > a);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_rawToQ6_scales_by_64);
    RUN_TEST(test_centre_deadband_reports_zero);
    RUN_TEST(test_endstops_reach_full_deflection);
    RUN_TEST(test_inverted_wiring_still_maps_positive_towards_max);
    RUN_TEST(test_unipolar_spans_zero_to_thousand);
    RUN_TEST(test_zero_span_calibration_does_not_divide_by_zero);
    RUN_TEST(test_sub_lsb_input_change_moves_the_wire_value);
    return UNITY_END();
}
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
~/.platformio/penv/bin/pio test -e native -f test_axis_mapping
```

Expected: FAIL — compile error, `AxisMapping.h: No such file or directory`.

- [ ] **Step 3: Write the implementation**

Create `RudderCAN/include/AxisMapping.h`:

```cpp
#ifndef AXIS_MAPPING_H
#define AXIS_MAPPING_H

#include <stdint.h>

// Q6 fixed point throughout: one raw ADC count is 64 units. Working at 64x the
// ADC resolution is what lets the filtered value reach the wire without being
// rounded back to whole counts.
inline int32_t rawToQ6(uint16_t raw)
{
    return (int32_t)raw << 6;
}

// Linear interpolation of `v` from [fromV, toV] onto 0..1000, clamped at both
// ends. Works with a reversed interval (toV < fromV) so inverted sensor wiring
// needs no special case.
inline int32_t mapToWire(int32_t v, int32_t fromV, int32_t toV)
{
    if (fromV == toV) {
        return 0;
    }
    int32_t out = ((v - fromV) * 1000) / (toV - fromV);
    if (out < 0) {
        out = 0;
    }
    if (out > 1000) {
        out = 1000;
    }
    return out;
}

// Maps one half of a travel (from `centerQ6` to `endQ6`) onto 0..1000. All
// offsets are derived from the signed (end - center) delta, so an inverted
// sensor — where the raw value falls as the pedal is pushed — works without
// special casing.
inline uint16_t mapHalfQ6(int32_t vQ6, int32_t centerQ6, int32_t endQ6, int32_t centerDeadbandQ6)
{
    const int32_t span = endQ6 - centerQ6;
    if (span == 0) {
        return 0;
    }
    const int32_t from = centerQ6 + ((span > 0) ? centerDeadbandQ6 : -centerDeadbandQ6);
    const int32_t to   = endQ6 - span / 20; // 5% end deadband, guarantees the endpoint is reachable
    return (uint16_t)mapToWire(vQ6, from, to);
}

// Maps a unidirectional axis (brake) between two calibration points onto 0..1000.
inline uint16_t mapUnipolarQ6(int32_t vQ6, int32_t loQ6, int32_t hiQ6)
{
    const int32_t span = hiQ6 - loQ6;
    if (span == 0) {
        return 0;
    }
    const int32_t deadband = span / 20; // 5% at each end
    return (uint16_t)mapToWire(vQ6, loQ6 + deadband, hiQ6 - deadband);
}

// Full bipolar rudder axis: -1000..1000, left negative.
inline int16_t mapRudderQ6(int32_t vQ6, int32_t minQ6, int32_t centerQ6, int32_t maxQ6, int32_t centerDeadbandQ6)
{
    const int32_t d = vQ6 - centerQ6;
    if (d <= centerDeadbandQ6 && -d <= centerDeadbandQ6) {
        return 0;
    }

    // Which half of the travel are we on? Decided against the signed direction
    // of the max endpoint, so it stays correct for inverted wiring too.
    const bool towardsMax = (maxQ6 > centerQ6) ? (vQ6 > centerQ6) : (vQ6 < centerQ6);

    if (towardsMax) {
        return (int16_t)mapHalfQ6(vQ6, centerQ6, maxQ6, centerDeadbandQ6);
    }
    return (int16_t)-(int16_t)mapHalfQ6(vQ6, centerQ6, minQ6, centerDeadbandQ6);
}

#endif // AXIS_MAPPING_H
```

- [ ] **Step 4: Run both suites to verify they pass**

```bash
~/.platformio/penv/bin/pio test -e native
```

Expected: PASS — 6 tests in `test_adaptive_filter`, 7 in `test_axis_mapping`, 0 failures.

- [ ] **Step 5: Commit**

```bash
git add RudderCAN/include/AxisMapping.h RudderCAN/test/test_axis_mapping/test_axis_mapping.cpp
git commit -m "feat(RudderCAN): add Q6 axis mapping helpers"
```

---

### Task 3: Wire the filter into `Rudder` and its callers

**Files:**
- Modify: `RudderCAN/src/Rudder.h`
- Modify: `RudderCAN/src/Rudder.cpp`
- Modify: `RudderCAN/src/CAN.cpp:16-26`
- Modify: `RudderCAN/src/BenchDebug.cpp:27-43`, `RudderCAN/src/BenchDebug.cpp:158-172`

**Interfaces:**
- Consumes: `AdaptiveFilter` (Task 1), `rawToQ6` / `mapUnipolarQ6` / `mapRudderQ6` (Task 2).
- Produces: `void Rudder::sample()`, `int32_t Rudder::getFilteredRudderQ6() const`, `int32_t Rudder::getFilteredLeftBrakeQ6() const`, `int32_t Rudder::getFilteredRightBrakeQ6() const`.

This task has no native test — `Rudder` pulls in `Arduino.h` and `EEPROM.h`. The logic worth testing already lives in the two headers covered by Tasks 1 and 2; verification here is a compile plus a bench run.

- [ ] **Step 1: Update `Rudder.h`**

Add the include, the new public methods, and the filter members. Apply these three edits:

Replace the include block at the top:

```cpp
#ifndef RUDDER_H
#define RUDDER_H
#include <Arduino.h>
#include "Configuration.h"
#include "AdaptiveFilter.h"
```

In the `public:` section, after `RudderStateUpdate getStateUpdate();`, add:

```cpp
    // Reads the ADCs into the axis filters. Call this on every loop() pass; it
    // rate-gates itself to kSampleIntervalMs internally.
    void sample();

    // Filtered raw values in Q6, for bench diagnostics.
    int32_t getFilteredRudderQ6() const;
    int32_t getFilteredLeftBrakeQ6() const;
    int32_t getFilteredRightBrakeQ6() const;
```

In the `private:` section, replace `int16_t mapRudder(uint16_t raw) const;` with the filter state (the mapping now lives in `AxisMapping.h`):

```cpp
    AdaptiveFilter _rudderFilter;
    AdaptiveFilter _leftBrakeFilter;
    AdaptiveFilter _rightBrakeFilter;
    uint32_t _lastSampleMs;
```

- [ ] **Step 2: Replace the mapping helpers and constants in `Rudder.cpp`**

Change the include block at the top of the file to:

```cpp
#include "Rudder.h"
#include "AxisMapping.h"
#include "DebugLog.h"
#include <EEPROM.h>
```

Replace the constant block at `Rudder.cpp:10-18` with:

```cpp
// Raw ADC counts around the calibrated center that still report 0. Rudder pedals
// have mechanical slop, so without this the aircraft would never track straight.
// Expressed in Q6, like everything else downstream of the filter.
static const int32_t kRudderCenterDeadbandQ6 = 12 * 64;

// The axis filters run far faster than the CAN send cadence: 10 samples per
// 20ms frame. Three ADC conversions at ~112us every 2ms is about 17% CPU, which
// leaves the MCP2515 SPI traffic plenty of room.
static const uint32_t kSampleIntervalMs = 2;
static const int32_t  kFilterAlphaMin   = 32; // Q8: tau ~16ms at rest, ~4x noise reduction
static const int32_t  kFilterSlope      = 12; // fully transparent from ~19 LSB of deviation

// Report thresholds on the 0..1000 wire scale. The filtered value carries
// sub-LSB resolution, so the wire quantum is about 1 unit and a threshold of 2
// suppresses nothing real. Send rate stays capped at 50Hz by kMinSendIntervalMs.
static const int16_t kRudderChangeThreshold = 2;
static const int16_t kBrakeChangeThreshold  = 2;
```

Then delete the now-unused `mapHalf` (`Rudder.cpp:23-35`) and `mapUnipolar` (`Rudder.cpp:38-51`) static functions, and delete `Rudder::mapRudder` (`Rudder.cpp:158-174`) — all three moved to `AxisMapping.h` in Task 2.

- [ ] **Step 3: Initialise the filters in the constructor**

In `Rudder::Rudder()`, replace the member setup so the filters get their tuning constants. The constructor body keeps the `pinMode` calls and `loadConfig()`; add an initialiser list before the opening brace:

```cpp
Rudder::Rudder()
    : _rudderFilter(kFilterAlphaMin, kFilterSlope),
      _leftBrakeFilter(kFilterAlphaMin, kFilterSlope),
      _rightBrakeFilter(kFilterAlphaMin, kFilterSlope),
      _lastSampleMs(0)
{
    pinMode(kRudderPin, INPUT);
    pinMode(kLeftBrakePin, INPUT);
    pinMode(kRightBrakePin, INPUT);

    _hasLastReportedState = false;
    _lastReportedState.rudder     = 0;
    _lastReportedState.leftBrake  = 0;
    _lastReportedState.rightBrake = 0;

    loadConfig();
}
```

- [ ] **Step 4: Add `sample()` and the diagnostic accessors**

Insert after the `getRawXxx()` accessors (`Rudder.cpp:108-110`):

```cpp
void Rudder::sample()
{
    const uint32_t now = millis();
    // Unsigned subtraction, so this stays correct across the millis() rollover.
    if (_rudderFilter.seeded() && (now - _lastSampleMs) < kSampleIntervalMs) {
        return;
    }
    _lastSampleMs = now;

    _rudderFilter.update(analogRead(kRudderPin));
    _leftBrakeFilter.update(analogRead(kLeftBrakePin));
    _rightBrakeFilter.update(analogRead(kRightBrakePin));
}

int32_t Rudder::getFilteredRudderQ6() const     { return _rudderFilter.valueQ6(); }
int32_t Rudder::getFilteredLeftBrakeQ6() const  { return _leftBrakeFilter.valueQ6(); }
int32_t Rudder::getFilteredRightBrakeQ6() const { return _rightBrakeFilter.valueQ6(); }
```

- [ ] **Step 5: Map from filter state instead of a fresh ADC read**

Replace `Rudder::getState()` (`Rudder.cpp:176-182`) with:

```cpp
RudderState Rudder::getState()
{
    // Rate-gated, so this is a no-op on the normal path; it only matters if
    // getState() is reached before loop() has ever sampled.
    sample();

    RudderState state;
    state.rudder = mapRudderQ6(_rudderFilter.valueQ6(),
                               rawToQ6(_config.rudderMin),
                               rawToQ6(_config.rudderCenter),
                               rawToQ6(_config.rudderMax),
                               kRudderCenterDeadbandQ6);
    state.leftBrake = mapUnipolarQ6(_leftBrakeFilter.valueQ6(),
                                    rawToQ6(_config.leftBrakeMin),
                                    rawToQ6(_config.leftBrakeMax));
    state.rightBrake = mapUnipolarQ6(_rightBrakeFilter.valueQ6(),
                                     rawToQ6(_config.rightBrakeMin),
                                     rawToQ6(_config.rightBrakeMax));
    return state;
}
```

`getStateUpdate()` is unchanged — it already works off `getState()` and the two threshold constants.

- [ ] **Step 6: Sample from `CAN::loop()`**

In `RudderCAN/src/CAN.cpp`, add the sampling call so it runs on every pass, before the 20 ms send gate returns early:

```cpp
void CAN::loop()
{
    InstrumentCAN::loop();

    // Must run on every pass — the filters need their 500Hz sample rate, which
    // the send gate below would otherwise throttle to 50Hz.
    rudder->sample();

    const uint32_t now = millis();
    if (now - lastSendTime < kMinSendIntervalMs)
    {
        return;
    }
```

- [ ] **Step 7: Sample from `BenchDebug::loop()` and print the filtered value**

In `RudderCAN/src/BenchDebug.cpp`, add the same call inside `loop()`, right after the heartbeat LED block and before `handleUserInput()`:

```cpp
    rudder->sample();
```

Then extend `printState()` so the smoothing is observable on the bench. Replace the rudder portion of the print with:

```cpp
    Serial.print(F("rud "));
    Serial.print(state.rudder);
    Serial.print(F(" (raw "));
    Serial.print(rudder->getRawRudder());
    Serial.print(F(" q6 "));
    Serial.print(rudder->getFilteredRudderQ6());
    Serial.print(F(")  lBrk "));
```

The `q6` figure is the filtered raw value times 64; divide by 64 to read it back as ADC counts.

- [ ] **Step 8: Verify both AVR builds**

Run from `RudderCAN/`:

```bash
~/.platformio/penv/bin/pio run -e nano
~/.platformio/penv/bin/pio run -e diecimilaatmega328
```

Expected: both SUCCESS. Note the reported RAM figure — the three filters add well under 20 bytes, so a jump larger than that means something else changed. PlatformIO's RAM percentage does not account for heap or stack, so treat it as a floor, not a budget.

- [ ] **Step 9: Verify the native tests still pass**

```bash
~/.platformio/penv/bin/pio test -e native
```

Expected: PASS, 13 tests, 0 failures.

- [ ] **Step 10: Commit**

```bash
git add RudderCAN/src/Rudder.h RudderCAN/src/Rudder.cpp RudderCAN/src/CAN.cpp RudderCAN/src/BenchDebug.cpp
git commit -m "feat(RudderCAN): oversample and adaptively filter the pedal axes"
```

---

### What the final review changed about Task 4

The whole-branch review (after Tasks 1–3 landed) sharpened this task considerably. Read this
before running it.

**The tuning's viability depends on a number nobody has yet.** Filtered noise reaches the wire
scaled by the mapping gain, which is inversely proportional to the sensor span:

| half-span (raw LSB) | gain (wire units / LSB) | centre deadband as % of half-travel |
| --- | --- | --- |
| 490 | 2.2 | 2.4 % |
| 300 | 3.7 | 4 % |
| 150 | 7.7 | 8 % |
| 80 | 15.6 | 15 % |

Decision rule once Step 1 gives the real number: **≥300** → the shipped constants are fine;
**150–300** → borderline, watch mid-travel; **<150** → re-tune before judging the fix
(`kDefaultAlphaMin` 32 → 16, `kDefaultSlope` 12 → 2–4) and make `kRudderCenterDeadbandQ6`
proportional. Do not conclude "the smoothing didn't work" from a small-span board still running
32/12.

**The filter is near-transparent to foot tremor, by construction.** A first-order EMA at
`alphaMin = 32` has its corner at ~10 Hz, and physiological foot tremor sits at 8–12 Hz — so it
removes only about a quarter of it, and the adaptive `slope` term makes that worse rather than
better (±3 LSB of tremor drives alpha to ~56–68, at which point the filter passes ~92 % of it).
What the filter does remove decisively is broadband electrical and ADC noise above ~50 Hz, where
attenuation is roughly 8×. `test_low_frequency_noise_is_attenuated` documents the tremor-band
behaviour as a measured characterisation (3.75 LSB span at 10 Hz), so it is recorded rather than
assumed.

The practical consequence: **Step 3 of the hardware session is a real gate, not a formality.**
Which of the two noise sources dominates decides the remedy, and they call for opposite fixes —
see Step 3a below.

**Lowering the threshold does not by itself reduce jitter.** The change threshold is a deadzone
of half-width T; the reported value re-latches back and forth whenever the input excursion
exceeds 2T. Going from T=8 to T=2 makes that condition *easier* to meet. The design's bet is
that the filter shrinks the excursion by more than the threshold shrank. That bet is what the
bench session tests.

**Look for jitter at mid-travel.** The centre deadband pins the output to 0 near neutral and the
5 % end deadband pins it to ±1000 at the stops, so the only region where re-latching can happen
is between them.

**The existing EEPROM calibration stays valid.** The Q6 port changes mapped values by well under
1 %, so there is no need to re-calibrate at cutover.

### Task 4: Bench and end-to-end verification

**Files:** none — this task changes no code. It exists because every claim in the spec about jitter, lag, and CAN health is unverified until it runs on real hardware.

**Interfaces:**
- Consumes: the firmware built in Task 3.

Do not mark this task complete from a build log. Each step needs an observed result.

- [ ] **Step 1: Record the actual sensor span**

Set `BENCHDEBUG` to `1` in `RudderCAN/src/Configuration.h:7`, flash, and open the monitor:

```bash
~/.platformio/penv/bin/pio run -e nano -t upload
~/.platformio/penv/bin/pio device monitor -b 115200
```

Type `s` with the pedals centred, then at full left and full right. Record all three `raw` values. This is the number the spec's measurement step calls for — it decides whether the resolution gain or the threshold change did the work, and it belongs in the final report.

- [ ] **Step 2: Check the filter at rest**

With the pedals untouched, watch the `q6` figure for 30 seconds. Expected: it settles and drifts by well under 64 (one ADC count). If it swings by hundreds, the sensor noise is far larger than assumed and `kFilterAlphaMin` needs lowering.

- [ ] **Step 3: Check for lag on real movement**

Sweep the pedals from stop to stop at normal speed and watch `rud`. Expected: the value tracks the pedal with no perceptible delay and no staircase. If it feels sluggish, raise `kDefaultSlope`; if it still steps, the change threshold is the remaining cause.

Use the `s` command for endpoint checks rather than trusting the auto-print: it is rate-gated to
100 ms, so a fast in-and-out excursion to a stop can land and reverse entirely inside one closed
window and never appear on the console.

- [ ] **Step 3a: Separate the two noise sources — they need opposite fixes**

Fit the 100 nF A0→GND capacitor as a *separate trial*, not at the same time as flashing.

- Jitter largely gone after the cap → the noise was electrical, and the filter is doing its job.
- Jitter survives the cap → it is 8–12 Hz foot tremor, and the answer is a **lower**
  `kDefaultAlphaMin` with a near-zero `kDefaultSlope`. More oversampling will not help.

Watching `q6` while holding the pedal still distinguishes them directly: fast dither is
electrical, slow wander is tremor.

- [ ] **Step 3b: Rule out ADC mux crosstalk**

Three back-to-back `analogRead`s on A0/A1/A2 now run 500×/s instead of 50×/s, with no settling
delay between channels. Press one toe brake hard and watch whether the rudder `q6` shifts.
Low-impedance hall outputs should be immune; this is the cheapest way to confirm it, and it was
never exercised at this rate before.

- [ ] **Step 3c: If you re-tune, fix the constants' single source of truth first**

`kDefaultAlphaMin` / `kDefaultSlope` live in `RudderCAN/include/AdaptiveFilter.h` and are used
both as the constructor defaults and by `Rudder.cpp`, and the native tests construct the filter
with them explicitly. Change them there and the tests follow. `test_ramp_tracking_error_stays_small`
and `test_low_frequency_noise_is_attenuated` assert tight bounds around the shipped values, so
expect them to fail after a re-tune — re-measure and update the bounds deliberately rather than
widening them.

- [ ] **Step 4: Confirm CAN is healthy under the added ADC load**

Set `BENCHDEBUG` back to `0`, flash, and run with the DCU attached. Watch the DCU bench console `rw` (rudder watch) output, and confirm the CAN alarm LED on the DCU stays dark. Expected: rudder frames arrive continuously, no heartbeat timeouts. This is the risk the spec flags — the 17 % ADC duty must not starve the MCP2515 RX path.

- [ ] **Step 5: End-to-end check in X-Plane**

Run X-Plane with `DCUProviderPlugin` loaded. Confirm the original complaint is gone: the rudder no longer jitters while the pedals are moving, and response still feels immediate. Check `Log.txt` for plugin errors.

- [ ] **Step 6: Commit any tuning changes**

If Steps 2–5 required changing `kFilterAlphaMin`, `kFilterSlope`, `kSampleIntervalMs`, or the thresholds, commit the tuned values with the measured reason:

```bash
git add RudderCAN/src/Rudder.cpp
git commit -m "tune(RudderCAN): adjust filter constants from bench measurements"
```

If nothing needed changing, there is nothing to commit — say so explicitly in the report rather than inventing a commit.

- [ ] **Step 7: Hardware note (optional, independent of everything above)**

If Step 2 shows more noise at rest than the filter comfortably removes, solder a 100 nF capacitor from A0 to GND at the Arduino end. It suppresses pickup on the pedal wiring and costs nothing. This is a hardware change and is the user's call — do not treat it as part of the software deliverable.
