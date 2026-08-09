# RudderCAN Oversampling + Percent Deadzones Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the RudderCAN board 13-bit effective analog resolution without blocking the CAN loop, and make all axis deadzones tunable percentages.

**Architecture:** A new `OversampledInput` class takes exactly one `analogRead()` per axis per `loop()` iteration and feeds a fixed-point exponential moving average (shift 6 → ~64-sample window → 3 extra bits). All pure integer math — EMA plus the deadzone/rescale mapping — moves into a header-only, Arduino-free `RudderMapping.h` so it can be unit-tested natively. `Rudder` keeps three `OversampledInput` members and gains a `poll()` that `main.cpp` calls each iteration.

**Tech Stack:** PlatformIO, Arduino framework, ATmega328 (`nanoatmega328new`, `diecimilaatmega328`), Unity test framework on a new `native` env.

## Global Constraints

- Target MCU is ATmega328: 16 MHz, 2 KB SRAM, `long` is 32-bit. Keep SRAM growth minimal — past CAN freezes on sibling AVR nodes were stack/heap collisions.
- **No `float` anywhere.** Integer math only, on both the hot path and in tests.
- **No blocking reads at runtime.** `analogRead()` costs ~112 µs; only calibration may block.
- `RudderMapping.h` must NOT include `Arduino.h`, `Configuration.h`, or anything else Arduino-specific — the native test env has no Arduino framework.
- The CAN wire format and the `RudderState` 0..1000 scale are unchanged. No gateway or `DCUProviderPlugin` change.
- The BenchDebug command set (`r-`, `r0`, `r+`, `l0`, `l1`, `b0`, `b1`, `s`, `?`) is unchanged.
- Exact constants, copied verbatim from the spec:
  - `kAdcEmaShift = 6`
  - `kAdcExtraBits = 3`
  - `kAdcCalibrationShift = 8`
  - `kAdcCalibrationSamples = 1U << kAdcCalibrationShift` (256)
  - `kAdcMax = (uint16_t)(1023U << kAdcExtraBits)` (8184)
  - `kRudderCenterDeadzonePercent = 2`
  - `kRudderEndDeadzonePercent = 5`
  - `kBrakeDeadzonePercent = 5`
  - `kRudderConfigVersion = 2`
  - `kRudderChangeThreshold = 4`, `kBrakeChangeThreshold = 4`
- Comments in code are English, matching the rest of the repo.
- `git` may fail in this sandbox (`unable to access '/Users/zaggo/.gitconfig'`). If a commit step errors that way, report it and continue — do not retry with alternative git configurations.

## File Structure

| File | Status | Responsibility |
|---|---|---|
| `src/RudderMapping.h` | create | Pure integer math: clamp, linear map, EMA step/prime/decimate, deadzone width, axis mappers. Header-only, no Arduino. |
| `src/OversampledInput.h` / `.cpp` | create | One analog axis: non-blocking EMA sampling + a blocking burst for calibration. |
| `src/Configuration.h` | modify | ADC oversampling constants + percent deadzone constants. |
| `src/Rudder.h` / `.cpp` | modify | Owns three `OversampledInput`s, exposes `poll()`, maps via `RudderMapping.h`, migrates EEPROM v1→v2. |
| `src/main.cpp` | modify | Calls `rudder->poll()` once per `loop()`. |
| `platformio.ini` | modify | Shared `[avr]` base + new `[env:native]` for Unity tests. |
| `test/test_mapping/test_mapping.cpp` | create | Native Unity tests for `RudderMapping.h`. |

`src/CAN.cpp/h` and `src/BenchDebug.cpp/h` need **no** changes — they call `getState()` / `getStateUpdate()`, whose signatures are untouched.

---

### Task 1: Mapping primitives + native test environment

**Files:**
- Create: `src/RudderMapping.h`
- Create: `test/test_mapping/test_mapping.cpp`
- Modify: `platformio.ini`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `long clampLong(long v, long lo, long hi)`
  - `long mapLinear(long x, long inMin, long inMax, long outMin, long outMax)`
  - `int32_t emaPrime(uint16_t raw, uint8_t shift)`
  - `int32_t emaStep(int32_t acc, uint16_t raw, uint8_t shift)`
  - `uint16_t emaValue(int32_t acc, uint8_t shift, uint8_t extraBits)`
  - `long deadzoneOf(long span, uint8_t percent)`

- [ ] **Step 1: Restructure `platformio.ini` so `[env]` no longer leaks into the native env**

The current file has a global `[env]` section that applies `framework = arduino` and `platform = atmelavr` to *every* env, including a native one. Replace the whole file with:

```ini
; PlatformIO Project Configuration File
;
;   Build options: build flags, source filter
;   Upload options: custom upload port, speed and extra flags
;   Library options: dependencies, extra library storages
;   Advanced options: extra scripting
;
; Please visit documentation for the other options and examples
; https://docs.platformio.org/page/projectconf.html

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

; Host-side unit tests for the pure math in src/RudderMapping.h.
; build_src_filter excludes src/ so nothing Arduino-specific gets compiled.
[env:native]
platform = native
test_framework = unity
build_src_filter = -<*>
build_flags = -Isrc
```

- [ ] **Step 2: Write the failing test**

Create `test/test_mapping/test_mapping.cpp`:

```cpp
#include <unity.h>
#include "RudderMapping.h"

// Matches Configuration.h: 10-bit ADC, ~64-sample EMA window, 3 extra bits.
static const uint8_t kShift = 6;
static const uint8_t kExtra = 3;

void setUp(void) {}
void tearDown(void) {}

void test_clamp_bounds(void) {
    TEST_ASSERT_EQUAL_INT32(0, clampLong(-5, 0, 1000));
    TEST_ASSERT_EQUAL_INT32(1000, clampLong(1500, 0, 1000));
    TEST_ASSERT_EQUAL_INT32(500, clampLong(500, 0, 1000));
}

void test_map_linear_endpoints_and_midpoint(void) {
    TEST_ASSERT_EQUAL_INT32(0, mapLinear(100, 100, 900, 0, 1000));
    TEST_ASSERT_EQUAL_INT32(1000, mapLinear(900, 100, 900, 0, 1000));
    TEST_ASSERT_EQUAL_INT32(500, mapLinear(500, 100, 900, 0, 1000));
}

void test_map_linear_degenerate_input_range(void) {
    TEST_ASSERT_EQUAL_INT32(0, mapLinear(42, 100, 100, 0, 1000));
}

void test_map_linear_inverted_input_range(void) {
    // Raw falls as the pedal is pushed — must still rise 0..1000.
    TEST_ASSERT_EQUAL_INT32(0, mapLinear(900, 900, 100, 0, 1000));
    TEST_ASSERT_EQUAL_INT32(1000, mapLinear(100, 900, 100, 0, 1000));
}

void test_ema_primed_value_is_stable(void) {
    const int32_t acc = emaPrime(500, kShift);
    TEST_ASSERT_EQUAL_INT32(acc, emaStep(acc, 500, kShift));
    TEST_ASSERT_EQUAL_UINT16(500 << kExtra, emaValue(acc, kShift, kExtra));
}

void test_ema_converges_from_below(void) {
    int32_t acc = emaPrime(0, kShift);
    for (int i = 0; i < 2000; i++) {
        acc = emaStep(acc, 500, kShift);
    }
    // Integer EMA settles on the first accumulator whose decimation equals raw.
    TEST_ASSERT_EQUAL_INT32((int32_t)500 << kShift, acc);
    TEST_ASSERT_EQUAL_UINT16(500 << kExtra, emaValue(acc, kShift, kExtra));
}

void test_ema_converges_from_above(void) {
    int32_t acc = emaPrime(1023, kShift);
    for (int i = 0; i < 5000; i++) {
        acc = emaStep(acc, 500, kShift);
    }
    // From above it stops at the top of the equal-decimation band.
    TEST_ASSERT_UINT16_WITHIN(8, 500 << kExtra, emaValue(acc, kShift, kExtra));
}

void test_ema_is_monotone_while_rising(void) {
    int32_t acc = emaPrime(0, kShift);
    uint16_t previous = emaValue(acc, kShift, kExtra);
    for (int i = 0; i < 500; i++) {
        acc = emaStep(acc, 800, kShift);
        const uint16_t current = emaValue(acc, kShift, kExtra);
        TEST_ASSERT_TRUE(current >= previous);
        previous = current;
    }
}

void test_ema_never_exceeds_full_scale(void) {
    int32_t acc = emaPrime(1023, kShift);
    for (int i = 0; i < 2000; i++) {
        acc = emaStep(acc, 1023, kShift);
        TEST_ASSERT_TRUE(acc <= ((int32_t)1023 << kShift));
    }
    TEST_ASSERT_EQUAL_UINT16(1023 << kExtra, emaValue(acc, kShift, kExtra));
}

void test_deadzone_of_span_uses_magnitude(void) {
    TEST_ASSERT_EQUAL_INT32(81, deadzoneOf(4092, 2));
    TEST_ASSERT_EQUAL_INT32(81, deadzoneOf(-4092, 2));
    TEST_ASSERT_EQUAL_INT32(204, deadzoneOf(4092, 5));
    TEST_ASSERT_EQUAL_INT32(0, deadzoneOf(4092, 0));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_clamp_bounds);
    RUN_TEST(test_map_linear_endpoints_and_midpoint);
    RUN_TEST(test_map_linear_degenerate_input_range);
    RUN_TEST(test_map_linear_inverted_input_range);
    RUN_TEST(test_ema_primed_value_is_stable);
    RUN_TEST(test_ema_converges_from_below);
    RUN_TEST(test_ema_converges_from_above);
    RUN_TEST(test_ema_is_monotone_while_rising);
    RUN_TEST(test_ema_never_exceeds_full_scale);
    RUN_TEST(test_deadzone_of_span_uses_magnitude);
    return UNITY_END();
}
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `~/.platformio/penv/bin/pio test -e native`
Expected: FAIL — compile error, `RudderMapping.h: No such file or directory`.

- [ ] **Step 4: Write the minimal implementation**

Create `src/RudderMapping.h`:

```cpp
#ifndef RUDDER_MAPPING_H
#define RUDDER_MAPPING_H

// Pure integer math for the rudder and brake axes. Deliberately free of
// Arduino.h (and of Configuration.h) so it can be unit tested on the host with
// `pio test -e native`. All tunables arrive as parameters.

#include <stdint.h>

inline long clampLong(long v, long lo, long hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

// Linear interpolation. An inverted input range (inMax < inMin) is fine — the
// signed division keeps the output rising.
inline long mapLinear(long x, long inMin, long inMax, long outMin, long outMax)
{
    if (inMax == inMin) {
        return outMin;
    }
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// The accumulator holds the smoothed value scaled by 2^shift.
inline int32_t emaPrime(uint16_t raw, uint8_t shift)
{
    return (int32_t)raw << shift;
}

// One step of a fixed-point exponential moving average with a time constant of
// 2^shift samples. Averaging over 4^n samples buys n bits, so shift 6 (~64
// samples) is worth 3 extra bits over the raw ADC width.
inline int32_t emaStep(int32_t acc, uint16_t raw, uint8_t shift)
{
    return acc + (int32_t)raw - (acc >> shift);
}

// Decimates the accumulator down to extraBits more than the raw ADC width.
inline uint16_t emaValue(int32_t acc, uint8_t shift, uint8_t extraBits)
{
    return (uint16_t)(acc >> (shift - extraBits));
}

// Deadzone width in raw counts, taken from the magnitude of a signed travel.
inline long deadzoneOf(long span, uint8_t percent)
{
    const long magnitude = (span < 0) ? -span : span;
    return magnitude * (long)percent / 100L;
}

#endif // RUDDER_MAPPING_H
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `~/.platformio/penv/bin/pio test -e native`
Expected: PASS, 10 tests, 0 failures.

- [ ] **Step 6: Verify the AVR envs still build after the platformio.ini restructure**

Run: `~/.platformio/penv/bin/pio run -e nano && ~/.platformio/penv/bin/pio run -e diecimilaatmega328`
Expected: both SUCCESS. Record the reported RAM/Flash figures — later tasks compare against them.

- [ ] **Step 7: Commit**

```bash
git add platformio.ini src/RudderMapping.h test/test_mapping/test_mapping.cpp
git commit -m "test(RudderCAN): native Unity env + mapping primitives

Adds clamp/map/EMA/deadzone-width helpers in an Arduino-free header so the
axis math can be tested on the host. platformio.ini gets a shared [avr] base
so [env] no longer forces the Arduino framework onto the native env."
```

---

### Task 2: Axis mappers with percent deadzones

**Files:**
- Modify: `src/RudderMapping.h`
- Modify: `test/test_mapping/test_mapping.cpp`

**Interfaces:**
- Consumes: `clampLong`, `mapLinear`, `deadzoneOf` from Task 1.
- Produces:
  - `uint16_t mapHalf(long raw, long center, long end, uint8_t centerPercent, uint8_t endPercent)` → 0..1000
  - `uint16_t mapUnipolar(long raw, long lo, long hi, uint8_t deadzonePercent)` → 0..1000
  - `int16_t mapRudderAxis(long raw, long minRaw, long center, long maxRaw, uint8_t centerPercent, uint8_t endPercent)` → -1000..1000

Background for the implementer: these replace the `static` functions of the same
names currently in `src/Rudder.cpp`. The behaviour is unchanged except that the
center zone becomes a percentage of the half-travel instead of a fixed 12 raw
ADC counts, and the end zone becomes a percentage instead of a hardcoded
`span / 20`. The "snap to the anchor" behaviour is not a separate branch — a raw
value inside the center zone maps below 0 and the clamp pins it to 0.

- [ ] **Step 1: Write the failing tests**

Append these test functions to `test/test_mapping/test_mapping.cpp`, above `main`:

```cpp
// Calibration used throughout: 13-bit scale, symmetric pedals.
static const long kCenter = 4092;
static const long kMax    = 8184;
static const long kMin    = 0;
static const uint8_t kCenterPct = 2;
static const uint8_t kEndPct    = 5;

void test_map_half_snaps_inside_center_zone(void) {
    TEST_ASSERT_EQUAL_UINT16(0, mapHalf(kCenter, kCenter, kMax, kCenterPct, kEndPct));
    // Center deadzone is 2% of 4092 = 81 counts.
    TEST_ASSERT_EQUAL_UINT16(0, mapHalf(kCenter + 80, kCenter, kMax, kCenterPct, kEndPct));
    TEST_ASSERT_EQUAL_UINT16(0, mapHalf(kCenter + 81, kCenter, kMax, kCenterPct, kEndPct));
}

void test_map_half_reaches_full_scale_inside_end_zone(void) {
    // End deadzone is 5% of 4092 = 204 counts, so 1000 is reached at 7980.
    TEST_ASSERT_EQUAL_UINT16(1000, mapHalf(7980, kCenter, kMax, kCenterPct, kEndPct));
    TEST_ASSERT_EQUAL_UINT16(1000, mapHalf(kMax, kCenter, kMax, kCenterPct, kEndPct));
}

void test_map_half_is_monotone_between_zones(void) {
    uint16_t previous = 0;
    for (long raw = kCenter; raw <= kMax; raw += 17) {
        const uint16_t current = mapHalf(raw, kCenter, kMax, kCenterPct, kEndPct);
        TEST_ASSERT_TRUE(current >= previous);
        previous = current;
    }
    TEST_ASSERT_EQUAL_UINT16(1000, previous);
}

void test_map_half_midpoint(void) {
    // Halfway between the zone edges (4173 and 7980) must land near 500.
    TEST_ASSERT_UINT16_WITHIN(2, 500, mapHalf(6076, kCenter, kMax, kCenterPct, kEndPct));
}

void test_map_half_inverted_wiring(void) {
    // end < center: raw falls as the pedal is pushed.
    TEST_ASSERT_EQUAL_UINT16(0, mapHalf(kCenter, kCenter, kMin, kCenterPct, kEndPct));
    TEST_ASSERT_EQUAL_UINT16(0, mapHalf(kCenter - 81, kCenter, kMin, kCenterPct, kEndPct));
    TEST_ASSERT_EQUAL_UINT16(1000, mapHalf(204, kCenter, kMin, kCenterPct, kEndPct));
    TEST_ASSERT_EQUAL_UINT16(1000, mapHalf(kMin, kCenter, kMin, kCenterPct, kEndPct));
}

void test_map_half_degenerate_inputs(void) {
    // Uncalibrated axis: center == end.
    TEST_ASSERT_EQUAL_UINT16(0, mapHalf(500, kCenter, kCenter, kCenterPct, kEndPct));
    // Deadzones that swallow the whole travel must not invert the mapping.
    TEST_ASSERT_EQUAL_UINT16(0, mapHalf(6000, kCenter, kMax, 60, 60));
}

void test_map_unipolar_endpoints_and_midpoint(void) {
    // 5% of 8184 = 409 counts at each end.
    TEST_ASSERT_EQUAL_UINT16(0, mapUnipolar(0, 0, kMax, 5));
    TEST_ASSERT_EQUAL_UINT16(0, mapUnipolar(409, 0, kMax, 5));
    TEST_ASSERT_EQUAL_UINT16(1000, mapUnipolar(7775, 0, kMax, 5));
    TEST_ASSERT_EQUAL_UINT16(1000, mapUnipolar(kMax, 0, kMax, 5));
    TEST_ASSERT_UINT16_WITHIN(2, 500, mapUnipolar(4092, 0, kMax, 5));
}

void test_map_unipolar_inverted_wiring(void) {
    TEST_ASSERT_EQUAL_UINT16(0, mapUnipolar(kMax, kMax, 0, 5));
    TEST_ASSERT_EQUAL_UINT16(1000, mapUnipolar(0, kMax, 0, 5));
    TEST_ASSERT_UINT16_WITHIN(2, 500, mapUnipolar(4092, kMax, 0, 5));
}

void test_map_unipolar_degenerate_inputs(void) {
    TEST_ASSERT_EQUAL_UINT16(0, mapUnipolar(500, 1000, 1000, 5));
    TEST_ASSERT_EQUAL_UINT16(0, mapUnipolar(4092, 0, kMax, 60));
}

void test_map_rudder_axis_signs_and_endpoints(void) {
    TEST_ASSERT_EQUAL_INT16(0, mapRudderAxis(kCenter, kMin, kCenter, kMax, kCenterPct, kEndPct));
    TEST_ASSERT_EQUAL_INT16(1000, mapRudderAxis(kMax, kMin, kCenter, kMax, kCenterPct, kEndPct));
    TEST_ASSERT_EQUAL_INT16(-1000, mapRudderAxis(kMin, kMin, kCenter, kMax, kCenterPct, kEndPct));
    TEST_ASSERT_TRUE(mapRudderAxis(7000, kMin, kCenter, kMax, kCenterPct, kEndPct) > 0);
    TEST_ASSERT_TRUE(mapRudderAxis(1000, kMin, kCenter, kMax, kCenterPct, kEndPct) < 0);
}

void test_map_rudder_axis_inverted_wiring(void) {
    // max below center, min above it.
    TEST_ASSERT_EQUAL_INT16(0, mapRudderAxis(kCenter, kMax, kCenter, kMin, kCenterPct, kEndPct));
    TEST_ASSERT_EQUAL_INT16(1000, mapRudderAxis(kMin, kMax, kCenter, kMin, kCenterPct, kEndPct));
    TEST_ASSERT_EQUAL_INT16(-1000, mapRudderAxis(kMax, kMax, kCenter, kMin, kCenterPct, kEndPct));
}

void test_map_rudder_axis_asymmetric_calibration(void) {
    // Left half is shorter than the right half; both must still reach full scale.
    TEST_ASSERT_EQUAL_INT16(-1000, mapRudderAxis(1000, 1000, kCenter, kMax, kCenterPct, kEndPct));
    TEST_ASSERT_EQUAL_INT16(1000, mapRudderAxis(kMax, 1000, kCenter, kMax, kCenterPct, kEndPct));
    TEST_ASSERT_EQUAL_INT16(0, mapRudderAxis(kCenter, 1000, kCenter, kMax, kCenterPct, kEndPct));
}
```

And register them inside `main`, after the existing `RUN_TEST` lines:

```cpp
    RUN_TEST(test_map_half_snaps_inside_center_zone);
    RUN_TEST(test_map_half_reaches_full_scale_inside_end_zone);
    RUN_TEST(test_map_half_is_monotone_between_zones);
    RUN_TEST(test_map_half_midpoint);
    RUN_TEST(test_map_half_inverted_wiring);
    RUN_TEST(test_map_half_degenerate_inputs);
    RUN_TEST(test_map_unipolar_endpoints_and_midpoint);
    RUN_TEST(test_map_unipolar_inverted_wiring);
    RUN_TEST(test_map_unipolar_degenerate_inputs);
    RUN_TEST(test_map_rudder_axis_signs_and_endpoints);
    RUN_TEST(test_map_rudder_axis_inverted_wiring);
    RUN_TEST(test_map_rudder_axis_asymmetric_calibration);
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `~/.platformio/penv/bin/pio test -e native`
Expected: FAIL — compile error, `mapHalf` / `mapUnipolar` / `mapRudderAxis` not declared.

- [ ] **Step 3: Write the minimal implementation**

Append to `src/RudderMapping.h`, before the closing `#endif`:

```cpp
// Maps one half of a travel (center -> end) onto 0..1000. Every offset derives
// from the signed (end - center) delta, so an inverted potentiometer — where the
// raw value falls as the pedal is pushed — works without special casing. Values
// inside the center deadzone map below 0 and the clamp pins them to 0; that
// clamp is the snap-to-center.
inline uint16_t mapHalf(long raw, long center, long end,
                        uint8_t centerPercent, uint8_t endPercent)
{
    const long span = end - center;
    if (span == 0) {
        return 0;
    }
    const long sign = (span > 0) ? 1L : -1L;
    const long from = center + sign * deadzoneOf(span, centerPercent);
    const long to   = end    - sign * deadzoneOf(span, endPercent);
    // Deadzones wide enough to meet or cross would invert the mapping.
    if ((to - from) * sign <= 0) {
        return 0;
    }
    return (uint16_t)clampLong(mapLinear(raw, from, to, 0L, 1000L), 0L, 1000L);
}

// Maps a unidirectional axis (a toe brake) between two calibration points onto
// 0..1000, with the same deadzone applied at both ends.
inline uint16_t mapUnipolar(long raw, long lo, long hi, uint8_t deadzonePercent)
{
    const long span = hi - lo;
    if (span == 0) {
        return 0;
    }
    const long sign     = (span > 0) ? 1L : -1L;
    const long deadzone = deadzoneOf(span, deadzonePercent);
    const long from     = lo + sign * deadzone;
    const long to       = hi - sign * deadzone;
    if ((to - from) * sign <= 0) {
        return 0;
    }
    return (uint16_t)clampLong(mapLinear(raw, from, to, 0L, 1000L), 0L, 1000L);
}

// Bipolar rudder axis, -1000 (left) .. 0 .. 1000 (right).
inline int16_t mapRudderAxis(long raw, long minRaw, long center, long maxRaw,
                             uint8_t centerPercent, uint8_t endPercent)
{
    // Which half of the travel are we on? Decided against the signed direction
    // of the max endpoint, so it stays correct for inverted wiring too.
    const bool towardsMax = (maxRaw > center) ? (raw > center) : (raw < center);
    if (towardsMax) {
        return (int16_t)mapHalf(raw, center, maxRaw, centerPercent, endPercent);
    }
    return (int16_t) - (int16_t)mapHalf(raw, center, minRaw, centerPercent, endPercent);
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `~/.platformio/penv/bin/pio test -e native`
Expected: PASS, 22 tests, 0 failures.

- [ ] **Step 5: Commit**

```bash
git add src/RudderMapping.h test/test_mapping/test_mapping.cpp
git commit -m "feat(RudderCAN): percent-based deadzones in the axis mappers

mapHalf/mapUnipolar/mapRudderAxis take the center and end deadzones as
percentages of the calibrated travel instead of a fixed raw count and a
hardcoded span/20. Covered by native tests including inverted wiring,
asymmetric calibration and degenerate inputs."
```

---

### Task 3: Oversampling constants + `OversampledInput`

**Files:**
- Modify: `src/Configuration.h`
- Create: `src/OversampledInput.h`
- Create: `src/OversampledInput.cpp`

**Interfaces:**
- Consumes: `emaPrime`, `emaStep`, `emaValue` from Task 1.
- Produces:
  - Constants `kAdcEmaShift`, `kAdcExtraBits`, `kAdcCalibrationShift`, `kAdcCalibrationSamples`, `kAdcMax`, `kRudderCenterDeadzonePercent`, `kRudderEndDeadzonePercent`, `kBrakeDeadzonePercent` in `Configuration.h`.
  - `class OversampledInput` with `explicit OversampledInput(uint8_t pin)`, `void begin()`, `void sample()`, `uint16_t value() const`, `uint16_t sampleBlocking()`.

This task has no unit test: the class is a thin shell around `analogRead()`, and
the math it wraps is already covered by Task 1. Verification is a clean build on
both AVR envs.

- [ ] **Step 1: Add the constants to `Configuration.h`**

Insert after the existing analog pin block (after the `kRightBrakePin` line, before the `MASK_EXACT` comment):

```cpp
// ADC oversampling. Each axis feeds a fixed-point EMA one sample per loop
// iteration; averaging over ~2^kAdcEmaShift samples buys kAdcExtraBits of
// resolution over the raw 10 bits. The averaging window is inherent to the
// resolution gain, so shift 6 (~32ms) trades the 14th bit for pedal response.
const uint8_t  kAdcEmaShift           = 6;   // ~64-sample time constant
const uint8_t  kAdcExtraBits          = 3;   // 10-bit ADC -> 13-bit effective
const uint8_t  kAdcCalibrationShift   = 8;   // blocking burst while calibrating
const uint16_t kAdcCalibrationSamples = 1U << kAdcCalibrationShift;
const uint16_t kAdcMax                = (uint16_t)(1023U << kAdcExtraBits); // 8184

// Deadzones as a percentage of the calibrated travel. The rudder snaps to 0
// near the calibrated center and reaches full deflection before the mechanical
// stop; the brakes get the same zone at both ends.
const uint8_t kRudderCenterDeadzonePercent = 2;
const uint8_t kRudderEndDeadzonePercent    = 5;
const uint8_t kBrakeDeadzonePercent        = 5;
```

- [ ] **Step 2: Create `src/OversampledInput.h`**

```cpp
#ifndef OVERSAMPLED_INPUT_H
#define OVERSAMPLED_INPUT_H
#include <Arduino.h>
#include "Configuration.h"

// One analog axis, oversampled without ever blocking the main loop: sample()
// takes a single ADC reading per call and folds it into a fixed-point EMA, so
// the CAN loop keeps polling while the resolution builds up over the window.
class OversampledInput
{
public:
    explicit OversampledInput(uint8_t pin);

    void begin();
    void sample();
    uint16_t value() const { return _value; }

    // Blocking full-precision burst (~29ms). Calibration only.
    uint16_t sampleBlocking();

private:
    uint8_t  _pin;
    int32_t  _acc;
    uint16_t _value;
};

#endif // OVERSAMPLED_INPUT_H
```

- [ ] **Step 3: Create `src/OversampledInput.cpp`**

```cpp
#include "OversampledInput.h"
#include "RudderMapping.h"

OversampledInput::OversampledInput(uint8_t pin)
    : _pin(pin), _acc(0), _value(0)
{
}

// Primes the accumulator from a single reading so the axis does not ramp up
// from zero over the first window after reset.
void OversampledInput::begin()
{
    pinMode(_pin, INPUT);
    _acc   = emaPrime((uint16_t)analogRead(_pin), kAdcEmaShift);
    _value = emaValue(_acc, kAdcEmaShift, kAdcExtraBits);
}

void OversampledInput::sample()
{
    _acc   = emaStep(_acc, (uint16_t)analogRead(_pin), kAdcEmaShift);
    _value = emaValue(_acc, kAdcEmaShift, kAdcExtraBits);
}

// Decimates 256 raw samples onto exactly the same scale sample()/value()
// produce, then re-primes the EMA from the result.
uint16_t OversampledInput::sampleBlocking()
{
    uint32_t sum = 0;
    for (uint16_t i = 0; i < kAdcCalibrationSamples; i++) {
        sum += (uint32_t)analogRead(_pin);
    }
    _value = (uint16_t)(sum >> (kAdcCalibrationShift - kAdcExtraBits));
    _acc   = (int32_t)_value << (kAdcEmaShift - kAdcExtraBits);
    return _value;
}
```

- [ ] **Step 4: Verify it builds**

Run: `~/.platformio/penv/bin/pio run -e nano`
Expected: SUCCESS. The new file compiles but nothing references it yet, so the linker may drop it — that is fine.

- [ ] **Step 5: Run the native tests to confirm nothing regressed**

Run: `~/.platformio/penv/bin/pio test -e native`
Expected: PASS, 22 tests, 0 failures. (`build_src_filter = -<*>` keeps `OversampledInput.cpp` out of the native build.)

- [ ] **Step 6: Commit**

```bash
git add src/Configuration.h src/OversampledInput.h src/OversampledInput.cpp
git commit -m "feat(RudderCAN): non-blocking oversampled analog input

One analogRead per axis per loop iteration feeds a fixed-point EMA, giving
13-bit effective resolution without the ~86ms stall a blocking 256-sample
burst across three axes would cost the CAN loop. Calibration still uses a
blocking burst, on the same scale."
```

---

### Task 4: Wire `Rudder` to the oversampled inputs

**Files:**
- Modify: `src/Rudder.h`
- Modify: `src/Rudder.cpp`
- Modify: `src/main.cpp`

**Interfaces:**
- Consumes: `OversampledInput` (Task 3), `mapUnipolar` / `mapRudderAxis` (Task 2), the deadzone percent constants (Task 3).
- Produces: `void Rudder::poll()` — must be called once per `loop()` iteration.

- [ ] **Step 1: Update `src/Rudder.h`**

Add the include below the existing ones:

```cpp
#include "OversampledInput.h"
```

In the `public:` section, add `poll()` right after the destructor:

```cpp
    // Takes one ADC sample per axis. Must be called every loop() iteration —
    // the oversampling window is counted in calls, not milliseconds.
    void poll();
```

In the `private:` section, add the three members above `RudderConfig _config;`:

```cpp
    OversampledInput _rudderInput;
    OversampledInput _leftBrakeInput;
    OversampledInput _rightBrakeInput;
```

and delete these two declarations:

```cpp
    uint16_t sampleAverage(uint8_t pin, uint8_t count);
    int16_t mapRudder(uint16_t raw) const;
```

- [ ] **Step 2: Update `src/Rudder.cpp` — includes, constants, constructor**

Add the include below `#include "DebugLog.h"`:

```cpp
#include "RudderMapping.h"
```

Delete the whole `kRudderCenterDeadbandRaw` block (the comment and the constant) — it is superseded by `kRudderCenterDeadzonePercent`.

Replace the change-threshold block with:

```cpp
// Report thresholds on the 0..1000 wire scale. With 13-bit oversampled inputs
// one ADC step is well under one wire unit, so this is purely a "don't flood
// the bus with sub-perceptible motion" limit.
static const int16_t kRudderChangeThreshold = 4;
static const int16_t kBrakeChangeThreshold  = 4;
```

Delete both `static` helper functions `mapHalf` and `mapUnipolar` (they now live in `RudderMapping.h`).

Replace the constructor with:

```cpp
Rudder::Rudder()
    : _rudderInput(kRudderPin),
      _leftBrakeInput(kLeftBrakePin),
      _rightBrakeInput(kRightBrakePin)
{
    _rudderInput.begin();
    _leftBrakeInput.begin();
    _rightBrakeInput.begin();

    _hasLastReportedState = false;
    _lastReportedState.rudder     = 0;
    _lastReportedState.leftBrake  = 0;
    _lastReportedState.rightBrake = 0;

    loadConfig();
}
```

- [ ] **Step 3: Update `src/Rudder.cpp` — poll, raw getters, calibration, getState**

Add `poll()` right after the destructor:

```cpp
void Rudder::poll() {
    _rudderInput.sample();
    _leftBrakeInput.sample();
    _rightBrakeInput.sample();
}
```

Replace the three raw getters:

```cpp
uint16_t Rudder::getRawRudder()     { return _rudderInput.value(); }
uint16_t Rudder::getRawLeftBrake()  { return _leftBrakeInput.value(); }
uint16_t Rudder::getRawRightBrake() { return _rightBrakeInput.value(); }
```

Delete `Rudder::sampleAverage` entirely (comment included) and replace the seven calibration methods with:

```cpp
void Rudder::calibrateRudderMin() {
    _config.rudderMin = _rudderInput.sampleBlocking();
    saveConfig();
}

void Rudder::calibrateRudderCenter() {
    _config.rudderCenter = _rudderInput.sampleBlocking();
    saveConfig();
}

void Rudder::calibrateRudderMax() {
    _config.rudderMax = _rudderInput.sampleBlocking();
    saveConfig();
}

void Rudder::calibrateLeftBrakeMin() {
    _config.leftBrakeMin = _leftBrakeInput.sampleBlocking();
    saveConfig();
}

void Rudder::calibrateLeftBrakeMax() {
    _config.leftBrakeMax = _leftBrakeInput.sampleBlocking();
    saveConfig();
}

void Rudder::calibrateRightBrakeMin() {
    _config.rightBrakeMin = _rightBrakeInput.sampleBlocking();
    saveConfig();
}

void Rudder::calibrateRightBrakeMax() {
    _config.rightBrakeMax = _rightBrakeInput.sampleBlocking();
    saveConfig();
}
```

Delete `Rudder::mapRudder` entirely and replace `Rudder::getState` with:

```cpp
RudderState Rudder::getState() {
    RudderState state;
    state.rudder = mapRudderAxis((long)_rudderInput.value(),
                                 (long)_config.rudderMin,
                                 (long)_config.rudderCenter,
                                 (long)_config.rudderMax,
                                 kRudderCenterDeadzonePercent,
                                 kRudderEndDeadzonePercent);
    state.leftBrake = mapUnipolar((long)_leftBrakeInput.value(),
                                  (long)_config.leftBrakeMin,
                                  (long)_config.leftBrakeMax,
                                  kBrakeDeadzonePercent);
    state.rightBrake = mapUnipolar((long)_rightBrakeInput.value(),
                                   (long)_config.rightBrakeMin,
                                   (long)_config.rightBrakeMax,
                                   kBrakeDeadzonePercent);
    return state;
}
```

`getStateUpdate()` stays exactly as it is.

- [ ] **Step 4: Update `src/main.cpp`**

Replace the body of `loop()` so the axes are sampled in both modes:

```cpp
void loop() {
  // Feeds the oversampling EMA. Must run every iteration, regardless of mode.
  rudder->poll();

  #if BENCHDEBUG
  benchDebug->loop();
  #else
  canBus->loop();
  #endif
}
```

- [ ] **Step 5: Verify it builds on both AVR envs**

Run: `~/.platformio/penv/bin/pio run -e nano && ~/.platformio/penv/bin/pio run -e diecimilaatmega328`
Expected: both SUCCESS, with no warnings about unused functions. Compare the reported RAM figure to the baseline from Task 1 Step 6 — expect roughly +21 bytes of static RAM (7 bytes per `OversampledInput`). If it grew by more than ~64 bytes, stop and investigate before continuing.

- [ ] **Step 6: Run the native tests**

Run: `~/.platformio/penv/bin/pio test -e native`
Expected: PASS, 22 tests, 0 failures.

- [ ] **Step 7: Commit**

```bash
git add src/Rudder.h src/Rudder.cpp src/main.cpp
git commit -m "feat(RudderCAN): read axes through the oversampled inputs

Rudder owns three OversampledInput members and exposes poll(), which main.cpp
calls once per loop iteration in both BenchDebug and CAN mode. Calibration
switches to the blocking burst, dropping the old 16-sample delay(5) loop.
Report thresholds drop from 8 to 4 now that the inputs resolve 13 bits."
```

---

### Task 5: EEPROM config migration v1 → v2

**Files:**
- Modify: `src/Rudder.cpp:6-8` (version constant) and `Rudder::loadConfig`

**Interfaces:**
- Consumes: `kAdcExtraBits`, `kAdcMax` (Task 3).
- Produces: nothing new — `RudderConfig` keeps its layout, only the meaning of the stored values changes.

Background: `RudderConfig` v1 stores raw 10-bit ADC counts. Task 3 moved every
reading onto a 13-bit scale, so stored calibration points are now 8× too small.
The struct layout is unchanged (all `uint16_t`), so a v1 record still
deserializes correctly and only needs rescaling.

- [ ] **Step 1: Bump the version constant**

In `src/Rudder.cpp`, change:

```cpp
static const uint16_t kRudderConfigVersion = 1;
```

to:

```cpp
static const uint16_t kRudderConfigVersion = 2;
```

- [ ] **Step 2: Add the migration to `loadConfig`**

Replace the leading `if / else` of `Rudder::loadConfig` (everything from `EEPROM.get` down to the closing brace of the `else` that logs "EEPROM config loaded") with:

```cpp
    EEPROM.get(kRudderEepromAddress, _config);

    if (_config.magic == kRudderConfigMagic && _config.version == 1) {
        // v1 stored raw 10-bit ADC counts; every reading is now oversampled to
        // kAdcExtraBits more, so the calibration points scale by the same shift.
        DEBUGLOG_PRINTLN(F("Rudder: migrating EEPROM config v1 -> v2"));
        _config.rudderMin     <<= kAdcExtraBits;
        _config.rudderCenter  <<= kAdcExtraBits;
        _config.rudderMax     <<= kAdcExtraBits;
        _config.leftBrakeMin  <<= kAdcExtraBits;
        _config.leftBrakeMax  <<= kAdcExtraBits;
        _config.rightBrakeMin <<= kAdcExtraBits;
        _config.rightBrakeMax <<= kAdcExtraBits;
        _config.version = kRudderConfigVersion;
        EEPROM.put(kRudderEepromAddress, _config);
    } else if (_config.magic != kRudderConfigMagic || _config.version != kRudderConfigVersion) {
        DEBUGLOG_PRINTLN(F("Rudder: No valid EEPROM config, writing defaults"));
        _config.magic         = kRudderConfigMagic;
        _config.version       = kRudderConfigVersion;
        _config.rudderMin     = 0;
        _config.rudderCenter  = kAdcMax / 2;
        _config.rudderMax     = kAdcMax;
        _config.leftBrakeMin  = 0;
        _config.leftBrakeMax  = kAdcMax;
        _config.rightBrakeMin = 0;
        _config.rightBrakeMax = kAdcMax;
        EEPROM.put(kRudderEepromAddress, _config);
    } else {
        DEBUGLOG_PRINTLN(F("Rudder: EEPROM config loaded"));
    }
```

The `DEBUGLOG_PRINT` dump of the loaded values that follows stays unchanged.

- [ ] **Step 3: Verify it builds on both AVR envs**

Run: `~/.platformio/penv/bin/pio run -e nano && ~/.platformio/penv/bin/pio run -e diecimilaatmega328`
Expected: both SUCCESS.

- [ ] **Step 4: Commit**

```bash
git add src/Rudder.cpp
git commit -m "feat(RudderCAN): migrate EEPROM calibration v1 -> v2

v1 records hold raw 10-bit counts; rescale them by kAdcExtraBits and rewrite
as v2 so existing bench calibration survives the switch to oversampled
inputs. Defaults move onto the 13-bit scale too."
```

---

### Task 6: End-to-end verification on hardware

**Files:** none (verification only).

**Interfaces:**
- Consumes: everything from Tasks 1–5.
- Produces: nothing.

This task cannot be completed by an agent without the board attached. If no
board is available, mark it blocked, report exactly which checks are
outstanding, and stop — do not claim the feature is verified.

- [ ] **Step 1: Confirm the full build and test suite**

Run:
```bash
~/.platformio/penv/bin/pio test -e native
~/.platformio/penv/bin/pio run -e nano
~/.platformio/penv/bin/pio run -e diecimilaatmega328
```
Expected: tests PASS, both builds SUCCESS. Record the final RAM/Flash figures.

- [ ] **Step 2: Flash a board with `BENCHDEBUG 1` and check the migration**

`src/Configuration.h` already has `BENCHDEBUG 1` and `DEBUGLOG_ENABLE 1`.

Run: `~/.platformio/penv/bin/pio run -t upload -e nano` then `~/.platformio/penv/bin/pio device monitor -b 115200`

Expected on a board that already had v1 calibration: the line
`Rudder: migrating EEPROM config v1 -> v2`, followed by the dump showing values
8× the previous ones. On a fresh board: `No valid EEPROM config, writing defaults`
and `rud=0/4092/8184`.

- [ ] **Step 3: Check axis behaviour**

With the pedals at rest, type `s`. Expected: `rud 0`, both brakes `0`, and the
raw values in the 0..8184 range. Push each pedal to its stop and confirm the
mapped values reach exactly `1000` / `-1000`. Release and confirm the rudder
returns to exactly `0` and stays there — no dithering around the center.

- [ ] **Step 4: Check the CAN path**

Set `BENCHDEBUG 0` in `src/Configuration.h`, rebuild and upload, and confirm on
the bus that the node still sends its instrument heartbeat on schedule and that
rudder frames are event-driven — moving the pedals produces traffic, holding
them still falls back to the 2 s periodic refresh rather than a continuous
stream at the new finer threshold. If holding still produces continuous traffic,
raise `kRudderChangeThreshold` / `kBrakeChangeThreshold` back toward 8 and note
the value that settles it.

Revert `BENCHDEBUG` to its original value before committing.

- [ ] **Step 5: Commit any threshold adjustment**

Only if Step 4 required a change:

```bash
git add src/Rudder.cpp
git commit -m "fix(RudderCAN): raise report thresholds to match measured noise"
```

---

## Self-Review Notes

- Spec coverage: `OversampledInput` (Task 3), EMA math + N=6/3-extra-bits (Tasks 1, 3), `RudderMapping.h` extraction (Tasks 1, 2), percent deadzone constants (Tasks 2, 3), `Rudder`/`main.cpp` wiring and `poll()` (Task 4), threshold 8→4 (Task 4), EEPROM v1→v2 (Task 5), native test env and the four test areas from the spec (Tasks 1, 2), build/hardware verification (Task 6). No spec section is unclaimed.
- The spec's `percentOf(long span, uint8_t percent)` is implemented under the name `deadzoneOf` — the name says what it computes at the one call site that matters. `mapHalf` / `mapUnipolar` / `mapRudderAxis` keep their spec signatures.
