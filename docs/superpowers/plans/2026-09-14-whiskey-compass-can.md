# WhiskeyCompassCAN Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a new Arduino board project `WhiskeyCompassCAN` that drives the cockpit's standby magnetic compass card with a 28BYJ-48 stepper and up to three panel LEDs, plus the CAN/serial/dataref plumbing that feeds it a heading from X-Plane.

**Architecture:** The board is a pure CAN consumer. `DCUProviderPlugin` reads X-Plane's magnetic compass dataref and sends serial message `0x0B` to the DCU; the DCU repacks it into CAN frame `0x107`; `WhiskeyCompassCAN` turns it into a shortest-path stepper move. Zero reference comes from a Hall sensor found by a non-blocking homing state machine, with the offset between Hall zero and the painted N mark stored in EEPROM.

**Tech Stack:** PlatformIO / Arduino (ATmega328 Nano), `coryjfowler/mcp_can` for the MCP2515, a vendored `CheapStepper` fork for the ULN2003, Unity for native unit tests, C++/CMake for the X-Plane plugin side.

**Spec:** `docs/superpowers/specs/2026-09-14-whiskey-compass-can-design.md`

## Global Constraints

- Node ID: `compassNodeId = 0x0A`. CAN message: `compass = 0x107`. Serial message: `SerialMessageCompass = 0x0B`. These exact values, no others.
- Dataref: `sim/cockpit2/gauges/indicators/compass_heading_deg_mag`.
- CAN wire format for `0x107`: `[0..1]` = `uint16` big-endian, degrees * 100, range `0..35999`; `[2..7]` reserved (zero).
- All multi-byte CAN fields in this repo are big-endian. All multi-byte serial-link payload fields are host byte order.
- Target board: `nanoatmega328new`. `monitor_speed = 115200`.
- Board `lib_deps` must contain `coryjfowler/mcp_can@^1.5.1` and nothing else. No `MCP23017`, no `Servo`.
- Instrument heartbeat timing is fixed by `InstrumentCAN`: send every 500 ms, gateway declared dead after 1500 ms. Nothing in this board may block longer than that.
- Never use `String` inside `CAN::handleFrame` or any function it calls — heap churn plus stack depth in that path caused CAN freezes on other nodes.
- Stepper RPM must stay within 10..14: below ~6 rpm the 28BYJ-48 overheats, above ~23 rpm it skips.
- `pio` lives at `~/.platformio/penv/bin/pio` and is also on `PATH`.

---

### Task 1: Protocol IDs in `shared/CANBase`

The whole rig reads these three headers, so they land first. No logic, no tests of their own — the deliverable is that every existing consumer still builds with the new enum values present.

**Files:**
- Modify: `shared/CANBase/include/CanNodeId.h`
- Modify: `shared/CANBase/include/CanMessageId.h`
- Modify: `shared/CANBase/include/SerialMessageId.h`

**Interfaces:**
- Consumes: nothing.
- Produces: `CanNodeId::compassNodeId` (value `0x0A`), `CanMessageId::compass` (value `0x107`), `MessageType::SerialMessageCompass` (value `0x0B`). Tasks 2, 4, 5 and 6 all depend on these names.

- [ ] **Step 1: Add the node ID**

In `shared/CANBase/include/CanNodeId.h`, extend the enum. Note the trailing comma now needed on `altimeterNodeId`:

```cpp
enum class CanNodeId : uint8_t {
  gatewayNodeId = 0x00,
  debugNodeId = 0x01,
  fuelGaugeNodeId = 0x02,
  transponderNodeId = 0x03,
  handbrakeNodeId = 0x04,
  rpmGaugeNodeId = 0x05,
  rudderNodeId = 0x06,
  asiNodeId = 0x07,
  vsiNodeId = 0x08,
  altimeterNodeId = 0x09,
  compassNodeId = 0x0A
};
```

- [ ] **Step 2: Add the CAN message ID**

In `shared/CANBase/include/CanMessageId.h`, insert after the `altimeterVsi = 0x102,` block and before `rpm = 0x106,`:

```cpp
  // 0x107: Magnetic compass (Gateway -> Instrument, 50Hz)
  // [0..1] heading uint16, degrees * 100, 0..35999
  //        (sim/cockpit2/gauges/indicators/compass_heading_deg_mag)
  // [2..7] reserved
  compass = 0x107,
```

- [ ] **Step 3: Add the serial message ID**

In `shared/CANBase/include/SerialMessageId.h`, extend the `MessageType` enum after `SerialMessageBaro = 0x0A,`:

```cpp
    // Plugin -> DCU. Payload: float headingDegMag (4 bytes, host order), degrees
    // magnetic. Repacked by the DCU into CAN 0x107 as a big-endian uint16 of
    // degrees * 100.
    SerialMessageCompass = 0x0B,
```

- [ ] **Step 4: Verify every existing consumer still builds**

Run from the repo root:

```bash
cd AltimeterCAN && pio run -e nano && cd ..
cd DCU && pio run && cd ..
cd VerticalSpeedCAN && pio run -e nano && cd ..
```

Expected: three successful builds. Adding enum values cannot break a consumer, so a failure here means a syntax error in the headers — most likely a missing or doubled comma.

- [ ] **Step 5: Commit**

```bash
git add shared/CANBase/include/CanNodeId.h shared/CANBase/include/CanMessageId.h shared/CANBase/include/SerialMessageId.h
git commit -m "feat(canbase): add compass node, CAN 0x107 and serial 0x0B IDs"
```

---

### Task 2: Board scaffold and `CompassGeometry.h`

TDD applies here: the geometry header is the only natively testable piece of this board, so its tests are written first and must fail before the header exists. The scaffold files (`platformio.ini`, `Configuration.h`, `DebugLog.h`) are folded in because the test run needs them.

Deliverable: `pio test -e native` passes in `WhiskeyCompassCAN`.

**Files:**
- Create: `WhiskeyCompassCAN/platformio.ini`
- Create: `WhiskeyCompassCAN/include/CompassGeometry.h`
- Create: `WhiskeyCompassCAN/test/test_geometry/test_geometry.cpp`
- Create: `WhiskeyCompassCAN/src/Configuration.h`
- Create: `WhiskeyCompassCAN/src/DebugLog.h`

**Interfaces:**
- Consumes: `CanNodeId::compassNodeId` from Task 1.
- Produces:
  - `uint32_t normalizeStepPosition(int32_t position, uint32_t totalSteps)`
  - `uint32_t degreesToSteps(double degrees, uint32_t totalSteps)`
  - `int32_t shortestPathSteps(uint32_t current, uint32_t target, uint32_t totalSteps)`
  - `int16_t normalizeZeroAdjust(int32_t degrees)`
  - `int16_t accumulateZeroAdjust(int16_t stored, int32_t jogDegrees)`
  - `int32_t jogDegreesFromPosition(uint32_t position, uint32_t totalSteps)`
  - `Configuration.h` constants: `kNodeId`, `kCanIntPin`, `kCanCSPin`, `kStepperPins[4]`, `kLightCount`, `kLightPins[]`, `kHallPin`, `kTotalSteps`, `kCardInversed`, `kRpmLimits[]` with `maxRpm`/`minRpm`, `kDefaultZeroAdjustDegree`, `MASK_EXACT`, `BENCHDEBUG`
  - `DebugLog.h` macros: `DEBUGLOG_INIT`, `DEBUGLOG_PRINT`, `DEBUGLOG_PRINTLN`

  Tasks 3, 4 and 5 use all of the above.

- [ ] **Step 1: Create `platformio.ini`**

```ini
; PlatformIO Project Configuration File
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

[env:native]
platform = native
test_framework = unity
build_src_filter = -<*>
```

`build_src_filter = -<*>` keeps the Arduino-coupled `src/` out of the native build; only `include/CompassGeometry.h` is compiled there. `lib/` is not built either, because nothing in the native test includes it.

- [ ] **Step 2: Write the failing tests**

Create `WhiskeyCompassCAN/test/test_geometry/test_geometry.cpp`:

```cpp
#include <unity.h>
#include "CompassGeometry.h"

void setUp(void) {}
void tearDown(void) {}

// --- step position normalisation -------------------------------------------

void test_positions_inside_one_turn_pass_through(void) {
    TEST_ASSERT_EQUAL_UINT32(0, normalizeStepPosition(0, 4096));
    TEST_ASSERT_EQUAL_UINT32(1024, normalizeStepPosition(1024, 4096));
    TEST_ASSERT_EQUAL_UINT32(4095, normalizeStepPosition(4095, 4096));
}

void test_a_full_turn_folds_onto_zero(void) {
    TEST_ASSERT_EQUAL_UINT32(0, normalizeStepPosition(4096, 4096));
    TEST_ASSERT_EQUAL_UINT32(904, normalizeStepPosition(5000, 4096));
}

// getPosition() is int32_t and can go negative after a counter-clockwise move.
// Taking that at face value would index off the far end of the card.
void test_negative_positions_fold_forward(void) {
    TEST_ASSERT_EQUAL_UINT32(4095, normalizeStepPosition(-1, 4096));
    TEST_ASSERT_EQUAL_UINT32(3072, normalizeStepPosition(-1024, 4096));
}

void test_zero_total_steps_cannot_divide_by_zero(void) {
    TEST_ASSERT_EQUAL_UINT32(0, normalizeStepPosition(500, 0));
}

// --- degrees to steps -------------------------------------------------------

void test_cardinal_headings_map_to_quarter_turns(void) {
    TEST_ASSERT_EQUAL_UINT32(0, degreesToSteps(0.0, 4096));
    TEST_ASSERT_EQUAL_UINT32(1024, degreesToSteps(90.0, 4096));
    TEST_ASSERT_EQUAL_UINT32(2048, degreesToSteps(180.0, 4096));
    TEST_ASSERT_EQUAL_UINT32(3072, degreesToSteps(270.0, 4096));
}

void test_a_full_turn_of_heading_is_zero(void) {
    TEST_ASSERT_EQUAL_UINT32(0, degreesToSteps(360.0, 4096));
}

// compass_heading_deg_mag can come through slightly out of range; an unwrapped
// negative would otherwise cast into a position near the far end of the card.
void test_out_of_range_headings_wrap(void) {
    TEST_ASSERT_EQUAL_UINT32(3072, degreesToSteps(-90.0, 4096));
    TEST_ASSERT_EQUAL_UINT32(1024, degreesToSteps(450.0, 4096));
    TEST_ASSERT_EQUAL_UINT32(1024, degreesToSteps(-270.0, 4096));
}

// Rounding is to the nearest step, so a heading within half a step of due north
// lands on 0 rather than on totalSteps.
void test_headings_just_short_of_north_round_onto_zero(void) {
    TEST_ASSERT_EQUAL_UINT32(4095, degreesToSteps(359.9, 4096));
    TEST_ASSERT_EQUAL_UINT32(0, degreesToSteps(359.99, 4096));
}

void test_a_geared_card_uses_its_own_step_count(void) {
    TEST_ASSERT_EQUAL_UINT32(512, degreesToSteps(90.0, 2048));
    TEST_ASSERT_EQUAL_UINT32(0, degreesToSteps(90.0, 0));
}

// --- shortest path ----------------------------------------------------------

void test_a_quarter_turn_goes_the_direct_way(void) {
    TEST_ASSERT_EQUAL_INT32(1024, shortestPathSteps(0, 1024, 4096));
    TEST_ASSERT_EQUAL_INT32(-1024, shortestPathSteps(1024, 0, 4096));
}

// The whole reason this function exists: a card at 359 degrees moving to 1
// degree must travel two degrees, not 358.
void test_crossing_north_takes_the_short_way(void) {
    TEST_ASSERT_EQUAL_INT32(32, shortestPathSteps(4080, 16, 4096));
    TEST_ASSERT_EQUAL_INT32(-32, shortestPathSteps(16, 4080, 4096));
}

void test_three_quarters_clockwise_becomes_a_quarter_counterclockwise(void) {
    TEST_ASSERT_EQUAL_INT32(-1024, shortestPathSteps(0, 3072, 4096));
}

// Exactly half a turn is equidistant either way. The tie is broken clockwise;
// pinned here so a refactor cannot silently flip it.
void test_exactly_half_a_turn_resolves_clockwise(void) {
    TEST_ASSERT_EQUAL_INT32(2048, shortestPathSteps(0, 2048, 4096));
}

void test_no_move_needed_when_already_on_target(void) {
    TEST_ASSERT_EQUAL_INT32(0, shortestPathSteps(1234, 1234, 4096));
}

void test_unnormalized_inputs_are_folded_first(void) {
    TEST_ASSERT_EQUAL_INT32(10, shortestPathSteps(4106, 20, 4096));
}

void test_shortest_path_on_a_zero_step_card_is_no_move(void) {
    TEST_ASSERT_EQUAL_INT32(0, shortestPathSteps(0, 100, 0));
}

// --- zero adjust ------------------------------------------------------------

void test_small_offsets_pass_through_unchanged(void) {
    TEST_ASSERT_EQUAL_INT16(5, normalizeZeroAdjust(5));
    TEST_ASSERT_EQUAL_INT16(-5, normalizeZeroAdjust(-5));
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(0));
}

void test_full_turns_collapse_to_zero(void) {
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(360));
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(-360));
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(3600));
}

void test_offsets_fold_into_minus180_to_179(void) {
    TEST_ASSERT_EQUAL_INT16(-170, normalizeZeroAdjust(190));
    TEST_ASSERT_EQUAL_INT16(170, normalizeZeroAdjust(-190));
    TEST_ASSERT_EQUAL_INT16(-180, normalizeZeroAdjust(180));
    TEST_ASSERT_EQUAL_INT16(-180, normalizeZeroAdjust(-180));
}

// After homing the card sits at 0 with the stored offset already applied, so a
// calibration jog is added to it, never substituted.
void test_a_jog_is_added_to_the_stored_offset(void) {
    TEST_ASSERT_EQUAL_INT16(8, accumulateZeroAdjust(5, 3));
    TEST_ASSERT_EQUAL_INT16(2, accumulateZeroAdjust(5, -3));
}

void test_repeated_calibration_passes_converge_instead_of_drifting(void) {
    int16_t stored = 5;
    stored = accumulateZeroAdjust(stored, 3);   // 8
    stored = accumulateZeroAdjust(stored, -1);  // 7
    stored = accumulateZeroAdjust(stored, 0);   // 7
    TEST_ASSERT_EQUAL_INT16(7, stored);
}

void test_accumulated_offsets_never_exceed_half_a_turn(void) {
    int16_t stored = 170;
    stored = accumulateZeroAdjust(stored, 20);
    TEST_ASSERT_EQUAL_INT16(-170, stored);
}

// --- position to jog --------------------------------------------------------

void test_a_forward_jog_reads_as_positive_degrees(void) {
    TEST_ASSERT_EQUAL_INT32(90, jogDegreesFromPosition(1024, 4096));
    TEST_ASSERT_EQUAL_INT32(0, jogDegreesFromPosition(0, 4096));
}

// A small backwards nudge comes back from getPosition() as almost a whole turn.
// Storing that at face value would record a ~350 degree offset for a nudge of ten.
void test_a_backward_jog_reads_as_negative_degrees(void) {
    TEST_ASSERT_EQUAL_INT32(-10, jogDegreesFromPosition(4096 - 114, 4096));
}

void test_a_zero_step_card_cannot_divide_by_zero(void) {
    TEST_ASSERT_EQUAL_INT32(0, jogDegreesFromPosition(1024, 0));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_positions_inside_one_turn_pass_through);
    RUN_TEST(test_a_full_turn_folds_onto_zero);
    RUN_TEST(test_negative_positions_fold_forward);
    RUN_TEST(test_zero_total_steps_cannot_divide_by_zero);
    RUN_TEST(test_cardinal_headings_map_to_quarter_turns);
    RUN_TEST(test_a_full_turn_of_heading_is_zero);
    RUN_TEST(test_out_of_range_headings_wrap);
    RUN_TEST(test_headings_just_short_of_north_round_onto_zero);
    RUN_TEST(test_a_geared_card_uses_its_own_step_count);
    RUN_TEST(test_a_quarter_turn_goes_the_direct_way);
    RUN_TEST(test_crossing_north_takes_the_short_way);
    RUN_TEST(test_three_quarters_clockwise_becomes_a_quarter_counterclockwise);
    RUN_TEST(test_exactly_half_a_turn_resolves_clockwise);
    RUN_TEST(test_no_move_needed_when_already_on_target);
    RUN_TEST(test_unnormalized_inputs_are_folded_first);
    RUN_TEST(test_shortest_path_on_a_zero_step_card_is_no_move);
    RUN_TEST(test_small_offsets_pass_through_unchanged);
    RUN_TEST(test_full_turns_collapse_to_zero);
    RUN_TEST(test_offsets_fold_into_minus180_to_179);
    RUN_TEST(test_a_jog_is_added_to_the_stored_offset);
    RUN_TEST(test_repeated_calibration_passes_converge_instead_of_drifting);
    RUN_TEST(test_accumulated_offsets_never_exceed_half_a_turn);
    RUN_TEST(test_a_forward_jog_reads_as_positive_degrees);
    RUN_TEST(test_a_backward_jog_reads_as_negative_degrees);
    RUN_TEST(test_a_zero_step_card_cannot_divide_by_zero);
    return UNITY_END();
}
```

- [ ] **Step 3: Run the tests to verify they fail**

```bash
cd WhiskeyCompassCAN && pio test -e native
```

Expected: FAIL at compile time with `CompassGeometry.h: No such file or directory`.

- [ ] **Step 4: Write `include/CompassGeometry.h`**

```cpp
#ifndef COMPASSGEOMETRY_H
#define COMPASSGEOMETRY_H

#include <math.h>
#include <stdint.h>

// Geometry for the whiskey compass card: heading to step position, the shortest
// way round between two positions, and the Hall-zero-to-painted-N offset.
// Deliberately Arduino-free so all of it can be exercised by the native Unity
// tests in test/test_geometry (pio test -e native).

// --- step positions ---------------------------------------------------------

// Folds any step position, negative included, into 0..totalSteps-1. CheapStepper
// reports an int32_t that goes negative after a counter-clockwise move, so this
// runs on everything that comes out of getPosition().
inline uint32_t normalizeStepPosition(int32_t position, uint32_t totalSteps)
{
    if (totalSteps == 0)
    {
        return 0;
    }
    const int32_t total = (int32_t)totalSteps;
    return (uint32_t)((position % total + total) % total);
}

// Heading in degrees to a normalised step position, rounded to the nearest step.
// Headings outside 0..360 are wrapped first: compass_heading_deg_mag can arrive
// slightly negative or above 360, and an unwrapped value would send the card to
// a position most of a turn away.
inline uint32_t degreesToSteps(double degrees, uint32_t totalSteps)
{
    if (totalSteps == 0)
    {
        return 0;
    }
    double wrapped = fmod(degrees, 360.0);
    if (wrapped < 0.0)
    {
        wrapped += 360.0;
    }
    const int32_t steps = (int32_t)(wrapped * (double)totalSteps / 360.0 + 0.5);
    return normalizeStepPosition(steps, totalSteps);
}

// Signed step delta from `current` to `target`, taking the short way round.
// Positive is clockwise. Without this a card at 359 degrees moving to 1 degree
// would wind 358 degrees backwards in full view of the pilot.
// At exactly half a turn both ways are equal and the tie breaks clockwise.
inline int32_t shortestPathSteps(uint32_t current, uint32_t target, uint32_t totalSteps)
{
    if (totalSteps == 0)
    {
        return 0;
    }
    const uint32_t cur = normalizeStepPosition((int32_t)current, totalSteps);
    const uint32_t tgt = normalizeStepPosition((int32_t)target, totalSteps);
    const uint32_t diffCW = (tgt + totalSteps - cur) % totalSteps;
    const uint32_t diffCCW = (cur + totalSteps - tgt) % totalSteps;
    if (diffCW <= diffCCW)
    {
        return (int32_t)diffCW;
    }
    return -(int32_t)diffCCW;
}

// --- zero adjust ------------------------------------------------------------

// Folds a degree value into -180..179 so repeated calibration passes cannot
// accumulate whole turns.
inline int16_t normalizeZeroAdjust(int32_t degrees)
{
    int32_t wrapped = degrees % 360;
    if (wrapped < -180)
    {
        wrapped += 360;
    }
    else if (wrapped > 179)
    {
        wrapped -= 360;
    }
    return (int16_t)wrapped;
}

// After homing the card sits at position 0 with the stored offset already
// applied, so jogging it onto the painted N mark means the new offset is the sum
// of the two. Replacing instead of adding throws away every previous pass and
// makes the card wander a little further each time.
inline int16_t accumulateZeroAdjust(int16_t stored, int32_t jogDegrees)
{
    return normalizeZeroAdjust((int32_t)stored + jogDegrees);
}

// The stepper reports its position normalised into 0..totalSteps-1, so a small
// counter-clockwise jog comes back as nearly a full turn. Fold it back onto the
// short way round before converting to degrees.
inline int32_t jogDegreesFromPosition(uint32_t position, uint32_t totalSteps)
{
    if (totalSteps == 0)
    {
        return 0;
    }

    const int32_t total = (int32_t)totalSteps;
    int32_t steps = (int32_t)position;
    if (steps > total / 2)
    {
        steps -= total;
    }
    return steps * 360 / total;
}

#endif // COMPASSGEOMETRY_H
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
cd WhiskeyCompassCAN && pio test -e native
```

Expected: PASS, 25 tests, 0 failures.

- [ ] **Step 6: Write `src/Configuration.h`**

```cpp
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <CanNodeId.h>

#define BENCHDEBUG 0

const CanNodeId kNodeId = CanNodeId::compassNodeId;

// CAN (MCP2515 on hardware SPI: D11 MOSI, D12 MISO, D13 SCK).
// /CS sits on D10 deliberately. D10 is the AVR /SS pin and must be an output in
// master mode; making it the chip select means SPI.begin() drives it for us.
// Left unused it would be a floating input, and a low level there drops the SPI
// out of master mode.
//
// /INT must be D2 or D3: those are the ATmega328's only external-interrupt pins
// (INT0/INT1), and BaseCAN hangs its RX handler off attachInterrupt(). On any
// other pin digitalPinToInterrupt() returns NOT_AN_INTERRUPT and attachInterrupt
// silently does nothing. Nothing else needs D2 on this board, so it gets /INT.
const uint8_t kCanIntPin = 2;
const uint8_t kCanCSPin = 10;

// ULN2003 IN1..IN4. All four are plain digital outputs, so they deliberately sit
// on the non-PWM pins and leave every usable PWM pin for the panel lights.
const uint8_t kStepperPins[4] = {4, 7, 8, 9};

// Panel lights, 1..3 LEDs depending on how the instrument was populated. This is
// a soldering decision, so it is fixed at compile time. D3 is Timer2-backed PWM,
// D5/D6 are Timer0; the other PWM pins are unavailable (D10/D11 are SPI, D9 is a
// stepper pin). All LEDs share one brightness.
const uint8_t kLightCount = 3;
const uint8_t kLightPins[kLightCount] = {3, 5, 6};

// Compass card zero. INPUT_PULLUP, active LOW, polled — no interrupt needed, the
// homing state machine reads it between steps.
const uint8_t kHallPin = A0;

// 4096 mini-steps per card revolution, assuming the card sits directly on the
// 28BYJ-48's output shaft. If a gear reduction is fitted, this is the only value
// that changes — CompassGeometry.h is ratio-agnostic.
const uint32_t kTotalSteps = 4096L;

// Whether the card turns with or against the commanded heading depends on the
// mechanical build. Flip this once observed on the bench.
const bool kCardInversed = false;

enum RpmKeys {
    maxRpm = 0,
    minRpm,
    rpmKeyCount
};

// Below ~6 rpm the 28BYJ-48 overheats, above ~23 rpm it skips steps. The slow
// rate is only used for the fine Hall-window search during homing.
const double kRpmLimits[rpmKeyCount] = {
    14.0, // maxRpm
    10.0  // minRpm
};

// Offset between the Hall sensor's mechanical zero and the card's painted N mark.
// The live value lives in EEPROM (see WhiskeyCompass::Config); this is only the
// factory default a freshly flashed board starts from.
const int16_t kDefaultZeroAdjustDegree = 0;

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

#endif // CONFIGURATION_H
```

- [ ] **Step 7: Write `src/DebugLog.h`**

```cpp
#ifndef DEBUGLOG_H
#define DEBUGLOG_H

#define DEBUGLOG_ENABLE 0

#if DEBUGLOG_ENABLE
#define DEBUGLOG_INIT(x) Serial.begin(x)
#define DEBUGLOG_PRINT(x) Serial.print(x)
#define DEBUGLOG_PRINTLN(x) Serial.println(x)
#else
#define DEBUGLOG_INIT(x)
#define DEBUGLOG_PRINT(x)
#define DEBUGLOG_PRINTLN(x)
#endif

#endif // DEBUGLOG_H
```

- [ ] **Step 8: Re-run the native tests**

`Configuration.h` and `DebugLog.h` are in `src/`, which `build_src_filter = -<*>` excludes, so the test result must be unchanged.

```bash
cd WhiskeyCompassCAN && pio test -e native
```

Expected: PASS, 25 tests, 0 failures.

- [ ] **Step 9: Commit**

```bash
git add WhiskeyCompassCAN/platformio.ini WhiskeyCompassCAN/include/CompassGeometry.h WhiskeyCompassCAN/test/test_geometry/test_geometry.cpp WhiskeyCompassCAN/src/Configuration.h WhiskeyCompassCAN/src/DebugLog.h
git commit -m "feat(whiskey-compass): scaffold board project with tested card geometry"
```

---

### Task 3: Vendored `CheapStepper`, device logic and bench console

Deliverable: `pio run -e nano` builds, and with `BENCHDEBUG 1` the serial console can home the card, drive it to a heading, dim the LEDs and store a zero offset.

The vendored library, the device class and the bench console ship together because none of them is independently buildable: the `nano` env has no `main.cpp` until this task, and `main.cpp` needs something to drive.

**Files:**
- Create: `WhiskeyCompassCAN/lib/CheapStepper/library.json`
- Create: `WhiskeyCompassCAN/lib/CheapStepper/src/CheapStepper.h`
- Create: `WhiskeyCompassCAN/lib/CheapStepper/src/CheapStepper.cpp`
- Create: `WhiskeyCompassCAN/src/WhiskeyCompass.h`
- Create: `WhiskeyCompassCAN/src/WhiskeyCompass.cpp`
- Create: `WhiskeyCompassCAN/src/BenchDebug.h`
- Create: `WhiskeyCompassCAN/src/BenchDebug.cpp`
- Create: `WhiskeyCompassCAN/src/main.cpp`
- Modify: `WhiskeyCompassCAN/src/Configuration.h` (temporarily set `BENCHDEBUG 1`, back to `0` in Task 4)

**Interfaces:**
- Consumes: everything Task 2 produced.
- Produces:
  - `class CheapStepper` with `CheapStepper(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4, bool inversed = false)`, `setTotalSteps(uint32_t)`, `getTotalSteps()`, `setRpm(double)`, `newMove(bool clockwise, uint32_t numSteps, uint32_t uS = micros())`, `run(uint32_t uS = micros())`, `stop()`, `off()`, `getPosition()`, `getStepsLeft()`, `resetPosition(uint32_t = 0)`
  - `class WhiskeyCompass` with `bool isHomed`, `void loop()`, `void beginHoming()`, `bool isHoming() const`, `CompassResult moveToHeading(double degMag)`, `void setBrightness(uint8_t)`, `bool calibrateZero()`, `void wipeCalibration()`, `int16_t zeroAdjustDegree() const`, `void stop()`, `void off()`, `int32_t position() const`
  - `class BenchDebug` with `BenchDebug(WhiskeyCompass*)` and `void loop()`

  Task 4 uses `WhiskeyCompass` and replaces the `BENCHDEBUG` value.

- [ ] **Step 1: Copy `CheapStepper` into the vendored library**

```bash
mkdir -p WhiskeyCompassCAN/lib/CheapStepper/src
cp AltimeterCAN/src/CheapStepper.h WhiskeyCompassCAN/lib/CheapStepper/src/CheapStepper.h
cp AltimeterCAN/src/CheapStepper.cpp WhiskeyCompassCAN/lib/CheapStepper/src/CheapStepper.cpp
```

- [ ] **Step 2: Strip the MCP23017 coupling from the copy**

Three edits in `WhiskeyCompassCAN/lib/CheapStepper/src/CheapStepper.h`:

Remove the expander include — this board drives four pins directly and pulling in `MCP23017` would force `blemasle/MCP23017` into `lib_deps` for nothing:

```cpp
// delete this line
#include <MCP23017.h>
```

Remove the `patternOut` constructor from the public section:

```cpp
// delete this line
  CheapStepper (uint8_t* patternOut, bool inversed = false);
```

Remove the `patternOut` member from the private section:

```cpp
// delete this line
  uint8_t* patternOut = NULL; // pointer to pattern buffer
```

Also delete the unused `bool melde=false;` member while you are in there — it is dead weight in a 2 KB RAM budget.

In `WhiskeyCompassCAN/lib/CheapStepper/src/CheapStepper.cpp`, delete the `patternOut` constructor entirely:

```cpp
// delete this whole function
CheapStepper::CheapStepper(uint8_t *patternOut, bool inversed) : pins{0xff, 0xff, 0xff, 0xff}, inversed(inversed)
{
  this->patternOut = patternOut;
}
```

Then collapse the two `patternOut == NULL` branches. In `off()`:

```cpp
void CheapStepper::off()
{
  for (uint8_t p = 0; p < 4; p++)
    digitalWrite(pins[p], 0);
}
```

And at the end of `seq()`, replace the `if (patternOut == NULL) { ... } else { ... }` block with just the direct-pin half:

```cpp
  // write pattern to pins
  for (uint8_t p = 0; p < 4; p++)
  {
    digitalWrite(pins[p], pattern[p]);
  }
}
```

- [ ] **Step 3: Write `lib/CheapStepper/library.json`**

```json
{
  "name": "CheapStepper",
  "version": "0.2.0-local1",
  "description": "Local fork of Tyler Henry's CheapStepper for the 28BYJ-48 via a ULN2003 driver, with the MCP23017 port-expander output path removed - this board drives the four coil pins directly. Based on https://github.com/tyhenry/CheapStepper.",
  "keywords": "28byj-48, uln2003, stepper, compass",
  "frameworks": "arduino",
  "platforms": "atmelavr"
}
```

- [ ] **Step 4: Verify the stripped library still compiles**

There is no `main.cpp` yet, so build the library on its own by compiling the native test env (which ignores `lib/`) and then checking the header parses. The honest check is deferred to Step 10; for now confirm nothing in the copy still references the expander:

```bash
grep -rn "MCP23017\|patternOut\|melde" WhiskeyCompassCAN/lib/CheapStepper/src/
```

Expected: no output.

- [ ] **Step 5: Write `src/WhiskeyCompass.h`**

```cpp
#ifndef WHISKEYCOMPASS_H
#define WHISKEYCOMPASS_H
#include <Arduino.h>
#include <CheapStepper.h>
#include "Configuration.h"
#include "CompassGeometry.h"

class WhiskeyCompass
{
public:
    enum CompassResult {
        success = 0,
        notHomed,
        homingTimeout,
        cardStateReached
    };

    enum HomingPhase {
        unknown = 0,
        leaveZero,
        searchZero,
        searchZeroEnd,
        returnToZeroEnd,
        searchZeroStart,
        moveToTrueZero,
        moveToAdjustedZero,
        homed,
        timeout
    };

    WhiskeyCompass();

    // True once the card has found its Hall zero and moved onto the painted N
    // mark. Before that a heading setpoint means nothing.
    bool isHomed = false;

    void loop();

    // Starts a homing run that loop() drives forward. Unlike a blocking run this
    // returns immediately, so the CAN heartbeat keeps flowing in both directions
    // while the card searches — a run takes several seconds, well past the 1500ms
    // both ends use to declare each other dead.
    void beginHoming();
    bool isHoming() const { return homingActive; }

    CompassResult moveToHeading(double degMag);

    // One brightness for all populated LEDs.
    void setBrightness(uint8_t brightness);

    // Records the card's current position as the painted N mark and persists it.
    // Must be homed first; the card is re-zeroed on success.
    bool calibrateZero();

    // Factory reset: zero offset back to the compiled-in default.
    void wipeCalibration();

    int16_t zeroAdjustDegree() const { return config.zeroAdjustDegree; }

    void stop();
    void off();

    int32_t position() const { return card->getPosition(); }

private:
    struct Config
    {
        uint32_t magic;
        uint16_t version;
        int16_t zeroAdjustDegree;
    };

    void loadConfig();
    void saveConfig();
    void applyConfigDefaults();

    void fetchZeroedState();
    void moveSteps(int32_t steps);
    void moveDegree(int32_t degree);
    CompassResult lookForZeroChange(bool targetZeroedState);
    CompassResult nextHomingState();
    void runHomingStep();

    Config config;
    CheapStepper *card;

    HomingPhase homingState = unknown;
    uint32_t zeroEndPosition = 0;
    bool zeroedState = false;
    bool homingActive = false;
};

#endif // WHISKEYCOMPASS_H
```

- [ ] **Step 6: Write `src/WhiskeyCompass.cpp`**

```cpp
#include "WhiskeyCompass.h"
#include <EEPROM.h>
#include "DebugLog.h"

static const uint32_t kCompassConfigMagic = 0x574B4331; // 'W','K','C','1'
static const uint16_t kCompassConfigVersion = 1;
static const uint16_t kCompassEepromAddress = 0;

static const int32_t kDegreeFullRotation = 360L;

void WhiskeyCompass::applyConfigDefaults()
{
    config.magic = kCompassConfigMagic;
    config.version = kCompassConfigVersion;
    config.zeroAdjustDegree = kDefaultZeroAdjustDegree;
}

void WhiskeyCompass::loadConfig()
{
    EEPROM.get(kCompassEepromAddress, config);
    if (config.magic != kCompassConfigMagic || config.version != kCompassConfigVersion)
    {
        DEBUGLOG_PRINTLN(F("WKC: no valid EEPROM config, writing defaults"));
        applyConfigDefaults();
        EEPROM.put(kCompassEepromAddress, config);
    }
    else
    {
        DEBUGLOG_PRINTLN(F("WKC: EEPROM config loaded"));
    }
}

void WhiskeyCompass::saveConfig()
{
    EEPROM.put(kCompassEepromAddress, config);
    DEBUGLOG_PRINTLN(F("WKC: config saved"));
}

void WhiskeyCompass::wipeCalibration()
{
    applyConfigDefaults();
    saveConfig();
}

WhiskeyCompass::WhiskeyCompass()
{
    loadConfig();

    pinMode(kHallPin, INPUT_PULLUP);

    for (uint8_t i = 0; i < kLightCount; i++)
    {
        pinMode(kLightPins[i], OUTPUT);
        analogWrite(kLightPins[i], 0);
    }

    card = new CheapStepper(kStepperPins[0], kStepperPins[1],
                            kStepperPins[2], kStepperPins[3], kCardInversed);
    card->setTotalSteps(kTotalSteps);
    card->setRpm(kRpmLimits[maxRpm]);
    card->resetPosition();
}

void WhiskeyCompass::loop()
{
    if (homingActive)
    {
        runHomingStep();
    }
    card->run(micros());
}

void WhiskeyCompass::setBrightness(uint8_t brightness)
{
    for (uint8_t i = 0; i < kLightCount; i++)
    {
        analogWrite(kLightPins[i], brightness);
    }
}

void WhiskeyCompass::stop()
{
    card->stop();
}

void WhiskeyCompass::off()
{
    card->off();
}

void WhiskeyCompass::moveSteps(int32_t steps)
{
    card->newMove(steps > 0, (uint32_t)abs(steps));
}

void WhiskeyCompass::moveDegree(int32_t degree)
{
    moveSteps(degree * (int32_t)card->getTotalSteps() / kDegreeFullRotation);
}

WhiskeyCompass::CompassResult WhiskeyCompass::moveToHeading(double degMag)
{
    if (!isHomed)
    {
        return notHomed;
    }

    // The stored zero offset is NOT added here. Homing's moveToAdjustedZero phase
    // already drove the card onto the painted N mark and reset the position there,
    // so step position 0 is the painted zero. Adding the offset again would
    // double-apply it and leave the card off by that much.
    const uint32_t total = card->getTotalSteps();
    const uint32_t target = degreesToSteps(degMag, total);
    const uint32_t current = normalizeStepPosition(card->getPosition(), total);

    // A new setpoint replaces whatever move is in flight, recomputed from the
    // current position. At 50Hz a queue would only pile up stale headings.
    moveSteps(shortestPathSteps(current, target, total));
    return success;
}

bool WhiskeyCompass::calibrateZero()
{
    if (!isHomed)
    {
        DEBUGLOG_PRINTLN(F("WKC: not homed, cannot store zero"));
        return false;
    }

    // getPosition() is int32_t and may be negative; normalising first keeps the
    // fold in jogDegreesFromPosition well-defined.
    const uint32_t total = card->getTotalSteps();
    const int32_t jog = jogDegreesFromPosition(normalizeStepPosition(card->getPosition(), total),
                                               total);
    config.zeroAdjustDegree = accumulateZeroAdjust(config.zeroAdjustDegree, jog);
    saveConfig();

    // The card is now standing on what we just declared to be north.
    card->resetPosition();
    return true;
}

void WhiskeyCompass::fetchZeroedState()
{
    zeroedState = (digitalRead(kHallPin) == LOW);
}

WhiskeyCompass::CompassResult WhiskeyCompass::lookForZeroChange(bool targetZeroedState)
{
    fetchZeroedState();
    if (zeroedState != targetZeroedState && card->getStepsLeft() != 0)
    {
        return success; // still searching
    }
    if (zeroedState != targetZeroedState)
    {
        DEBUGLOG_PRINTLN(F("WKC: homing timeout"));
        return homingTimeout; // ran out of commanded travel
    }
    return cardStateReached;
}

WhiskeyCompass::CompassResult WhiskeyCompass::nextHomingState()
{
    CompassResult zeroState;
    switch (homingState)
    {
    case unknown:
        fetchZeroedState();
        card->stop();
        card->resetPosition();
        card->setRpm(kRpmLimits[maxRpm]);
        // 370 degrees is deliberately more than a full turn, so the sensor is
        // guaranteed to be passed no matter where the card started.
        moveDegree(370);
        homingState = leaveZero;
        return success;

    case leaveZero:
        zeroState = lookForZeroChange(false);
        if (zeroState == cardStateReached)
        {
            card->stop();
            card->resetPosition();
            moveDegree(370);
            homingState = searchZero;
            return success;
        }
        return zeroState;

    case searchZero:
        zeroState = lookForZeroChange(true);
        if (zeroState == cardStateReached)
        {
            card->stop();
            card->resetPosition();
            card->setRpm(kRpmLimits[minRpm]);
            moveDegree(60);
            homingState = searchZeroEnd;
            return success;
        }
        return zeroState;

    case searchZeroEnd:
        zeroState = lookForZeroChange(false);
        if (zeroState == cardStateReached)
        {
            zeroEndPosition = normalizeStepPosition(card->getPosition(), card->getTotalSteps());
            card->stop();
            card->resetPosition();
            moveDegree(-65);
            homingState = returnToZeroEnd;
            return success;
        }
        return zeroState;

    case returnToZeroEnd:
        zeroState = lookForZeroChange(true);
        if (zeroState == cardStateReached)
        {
            zeroEndPosition = normalizeStepPosition(card->getPosition(), card->getTotalSteps());
            card->stop();
            card->resetPosition();
            moveDegree(-65);
            homingState = searchZeroStart;
            return success;
        }
        return zeroState;

    case searchZeroStart:
        zeroState = lookForZeroChange(false);
        if (zeroState == cardStateReached)
        {
            const uint32_t zeroStartPosition =
                normalizeStepPosition(card->getPosition(), card->getTotalSteps());
            card->stop();
            card->resetPosition();
            card->setRpm(kRpmLimits[maxRpm]);
            // True zero is the middle of the Hall window. Taking the first edge
            // instead would make homing depend on approach direction.
            const int32_t zeroAdjust =
                ((int32_t)zeroEndPosition - (int32_t)zeroStartPosition) %
                (int32_t)card->getTotalSteps() / 2L;
            DEBUGLOG_PRINT(F("WKC: zero adjust steps "));
            DEBUGLOG_PRINTLN(zeroAdjust);
            moveSteps(zeroAdjust);
            homingState = moveToTrueZero;
            return success;
        }
        return zeroState;

    case moveToTrueZero:
        if (card->getStepsLeft() != 0)
        {
            return success;
        }
        card->resetPosition();
        moveDegree(config.zeroAdjustDegree);
        homingState = moveToAdjustedZero;
        return success;

    case moveToAdjustedZero:
        if (card->getStepsLeft() != 0)
        {
            return success;
        }
        card->resetPosition();
        homingState = homed;
        return success;

    case homed:
        return success;

    case timeout:
        return homingTimeout;
    }
    return homingTimeout;
}

void WhiskeyCompass::beginHoming()
{
    if (homingActive)
    {
        return;
    }

    card->stop();
    isHomed = false;
    homingState = unknown;
    nextHomingState();
    homingActive = true;
    DEBUGLOG_PRINTLN(F("WKC: homing started"));
}

void WhiskeyCompass::runHomingStep()
{
    const CompassResult result = nextHomingState();
    if (result != success)
    {
        DEBUGLOG_PRINTLN(F("WKC: homing failed"));
        homingState = timeout;
        homingActive = false;
        return;
    }

    if (homingState == homed)
    {
        homingActive = false;
        isHomed = true;
        DEBUGLOG_PRINTLN(F("WKC: homed"));
    }
}
```

- [ ] **Step 7: Write `src/BenchDebug.h`**

```cpp
#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include "Configuration.h"

#if BENCHDEBUG
#include <Arduino.h>
#include "WhiskeyCompass.h"

class BenchDebug {
    public:
        BenchDebug(WhiskeyCompass* compass);

        void loop();
    private:
        void handleUserInput();
        bool handleCommand(String command);
        void printStatus();

        String inputBuffer;

        WhiskeyCompass* compass;
};
#endif
#endif // BENCHDEBUG_H
```

- [ ] **Step 8: Write `src/BenchDebug.cpp`**

```cpp
#include <BenchDebug.h>
#if BENCHDEBUG

// No heartbeat LED here: the Nano's onboard LED is on D13, which is SPI SCK on
// this board. Driving it would fight the CAN link.

BenchDebug::BenchDebug(WhiskeyCompass* compass)
: compass(compass)
{
    Serial.begin(115200);
    Serial.println(F("WhiskeyCompass BenchDebug"));

    inputBuffer = "";

    Serial.println(F("System running! '?' for commands"));
}

void BenchDebug::printStatus()
{
    Serial.println(String(F("homed: ")) + (compass->isHomed ? F("yes") : F("no"))
                 + F(" homing: ") + (compass->isHoming() ? F("yes") : F("no"))
                 + F(" position: ") + String(compass->position())
                 + F(" zeroAdjust: ") + String(compass->zeroAdjustDegree()));
}

// One command per line, unlike the DCU console - this board has few enough of
// them that a splitter would be more code than it saves.
bool BenchDebug::handleCommand(String command)
{
    if (command.startsWith("hd")) {
        String rString = command.substring(2);
        rString.trim();
        float degrees = rString.toFloat();
        if (compass->moveToHeading(degrees) == WhiskeyCompass::notHomed) {
            Serial.println(F("Not homed - run 'ho' first"));
            return true;
        }
        Serial.println(String(F("Heading set to ")) + degrees + F(" deg"));
        return true;
    } else if (command.startsWith("ho")) {
        compass->beginHoming();
        Serial.println(F("Homing started"));
        return true;
    } else if (command.startsWith("li")) {
        String rString = command.substring(2);
        rString.trim();
        long value = rString.toInt();
        uint8_t pwm = (uint8_t)constrain(value, 0L, 255L);
        compass->setBrightness(pwm);
        Serial.println(String(F("Brightness set to ")) + pwm);
        return true;
    } else if (command.startsWith("cz")) {
        if (compass->calibrateZero()) {
            Serial.println(String(F("Zero stored, offset now "))
                         + compass->zeroAdjustDegree() + F(" deg"));
        } else {
            Serial.println(F("Not homed - cannot store zero"));
        }
        return true;
    } else if (command.startsWith("cw")) {
        compass->wipeCalibration();
        Serial.println(F("Calibration wiped to defaults"));
        return true;
    } else if (command.startsWith("st")) {
        compass->stop();
        compass->off();
        Serial.println(F("Card stopped and de-energised"));
        return true;
    } else if (command.startsWith("sd")) {
        printStatus();
        return true;
    } else if (command.startsWith("?")) {
        Serial.println(F("WhiskeyCompass Commands:"));
        Serial.println(F("ho: start homing"));
        Serial.println(F("hd<deg>: move card to magnetic heading"));
        Serial.println(F("li<0..255>: set LED brightness"));
        Serial.println(F("cz: store current position as north"));
        Serial.println(F("cw: wipe calibration to defaults"));
        Serial.println(F("st: stop and de-energise the card"));
        Serial.println(F("sd: show status"));
        return true;
    }
    return false;
}

void BenchDebug::handleUserInput()
{
    while (Serial.available() > 0)
    {
        char receivedChar = Serial.read();
        if (receivedChar == '\n')
        {
            Serial.println();
            inputBuffer.trim();
            if (inputBuffer.length() > 0 && !handleCommand(inputBuffer))
            {
                Serial.println(String(F("Unknown command: ")) + inputBuffer);
            }
            inputBuffer = "";
        }
        else if (receivedChar != '\r')
        {
            inputBuffer += receivedChar;
            Serial.print(receivedChar);
        }
    }
}

void BenchDebug::loop()
{
    handleUserInput();
}
#endif
```

- [ ] **Step 9: Write `src/main.cpp`**

```cpp
#include <Arduino.h>
#include "Configuration.h"
#include "WhiskeyCompass.h"
#include "DebugLog.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "CAN.h"
CAN* canBus;
#endif

WhiskeyCompass* compass;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("WhiskeyCompass initializing..."));

  compass = new WhiskeyCompass();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(compass);
  #else
  canBus = new CAN(compass);
  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("WhiskeyCompass started up"));
  }
  #endif
}

void loop() {
  #if BENCHDEBUG
  benchDebug->loop();
  #else
  canBus->loop();
  #endif
  compass->loop();
}
```

- [ ] **Step 10: Build in bench mode**

`CAN.h` does not exist yet, so this task can only build with `BENCHDEBUG 1`. In `WhiskeyCompassCAN/src/Configuration.h` change:

```cpp
#define BENCHDEBUG 1
```

Then:

```bash
cd WhiskeyCompassCAN && pio run -e nano
```

Expected: successful build. Note the reported RAM figure — PlatformIO's percentage counts statics only and hides the heap, and a stack/heap collision is what froze the CAN link on `RPMGaugeCAN`. Anything above roughly 60% static RAM on a 2 KB part is worth flagging in the task report.

- [ ] **Step 11: Re-run the native tests**

Nothing in this task touched `include/`, so the geometry tests must still pass.

```bash
cd WhiskeyCompassCAN && pio test -e native
```

Expected: PASS, 25 tests, 0 failures.

- [ ] **Step 12: Commit**

```bash
git add WhiskeyCompassCAN/lib WhiskeyCompassCAN/src
git commit -m "feat(whiskey-compass): add stepper driver, card logic and bench console"
```

---

### Task 4: CAN layer

Deliverable: `pio run -e nano` builds with `BENCHDEBUG 0`, and the board answers the gateway heartbeat, homes on first contact, follows `0x107` and dims on `0x203`.

**Files:**
- Create: `WhiskeyCompassCAN/src/CAN.h`
- Create: `WhiskeyCompassCAN/src/CAN.cpp`
- Modify: `WhiskeyCompassCAN/src/Configuration.h` (`BENCHDEBUG` back to `0`)

**Interfaces:**
- Consumes: `WhiskeyCompass` from Task 3; `CanMessageId::compass`, `CanNodeId::compassNodeId` from Task 1; `InstrumentCAN`, `CANFirmwareInfo`, `CAN_STD_ID`, `MASK_EXACT`.
- Produces: `class CAN : public InstrumentCAN` with `CAN(WhiskeyCompass* compass)`. Used by `main.cpp`, which Task 3 already wrote against this exact signature.

- [ ] **Step 1: Write `src/CAN.h`**

There is no board-specific transmit path — the whiskey compass has no inputs — so unlike `AltimeterCAN` this class does not override `loop()`.

```cpp
#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <InstrumentCAN.h>
#include <CanMessageId.h>
#include <CanNodeId.h>
#include "WhiskeyCompass.h"

class CAN : public InstrumentCAN {
    public:
        CAN(WhiskeyCompass* compass);

    protected:
        bool instrumentBegin() override;
        void onStartupFail() override;
        void handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t* data) override;
        void onGatewayHeartbeatTimeout() override;
        void onGatewayHeartbeatDiscovered() override;

    private:
        WhiskeyCompass* compass;
};

#endif
```

- [ ] **Step 2: Write `src/CAN.cpp`**

```cpp
#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

CAN::CAN(WhiskeyCompass *compass)
    : InstrumentCAN(kCanCSPin, kCanIntPin, CANFirmwareInfo{static_cast<uint16_t>(kNodeId), 1, 0}),
      compass(compass)
{
    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

void CAN::onStartupFail()
{
    DEBUGLOG_PRINTLN(F("CAN startup FAIL"));
    compass->setBrightness(0);
}

bool CAN::instrumentBegin()
{
    // Beide RX-Buffer vergleichen alle ID-Bits
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1

    // RXB0: Compass heading
    canBus->init_Filt(0, 0, CAN_STD_ID(CanMessageId::compass));
    canBus->init_Filt(1, 0, CAN_STD_ID(CanMessageId::compass));

    // RXB1: Lights und Gateway Heartbeat
    canBus->init_Filt(2, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(3, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(4, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(5, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));

    canBus->setMode(MCP_NORMAL);

    compass->setBrightness(0);

    return true;
}

void CAN::handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t *data)
{
    // No String here: heap churn + extra stack in the deepest call path was
    // part of the stack/heap collision that froze the CAN link on other nodes.
    DEBUGLOG_PRINT(F("CAN Message received: ID "));
    DEBUGLOG_PRINTLN(static_cast<uint16_t>(id));

    // We currently expect standard frames only (ext == 0).
    (void)ext;

    switch (id)
    {
    case CanMessageId::compass:
    {
        if (len >= 2)
        {
            // [0..1] magnetic heading, degrees * 100, big-endian.
            const uint16_t deg100 = (static_cast<uint16_t>(data[0]) << 8) |
                                    static_cast<uint16_t>(data[1]);

            // Before homing the card position means nothing, so a setpoint would
            // send it somewhere arbitrary.
            if (compass->isHomed)
            {
                compass->moveToHeading(static_cast<double>(deg100) / 100.);
            }
        }
        break;
    }

    case CanMessageId::lights:
    {
        if (len >= 8)
        {
            const uint16_t panelDim1000 = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
            float ratio = constrain(static_cast<float>(panelDim1000) / 1000., 0., 1.);
            uint8_t pwm = static_cast<uint8_t>(ratio * 255.);
            compass->setBrightness(pwm);
        }
        break;
    }

    default:
        break;
    }
}

void CAN::onGatewayHeartbeatTimeout()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat TIMEOUT"));
    compass->setBrightness(0);
}

void CAN::onGatewayHeartbeatDiscovered()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat OK"));
    compass->setBrightness(255);

    // First contact with the gateway is what starts the card: homing before that
    // would drive the instrument with no sim running.
    if (!compass->isHomed && !compass->isHoming())
    {
        compass->beginHoming();
    }
}
```

- [ ] **Step 3: Switch back to CAN mode**

In `WhiskeyCompassCAN/src/Configuration.h`:

```cpp
#define BENCHDEBUG 0
```

- [ ] **Step 4: Build both modes**

```bash
cd WhiskeyCompassCAN && pio run -e nano
```

Expected: successful build in CAN mode. Then temporarily flip `BENCHDEBUG` to `1`, run `pio run -e nano` again to confirm the bench path still compiles, and flip it back to `0`. Both must build — the whole point of the flag is that either one is one edit away.

- [ ] **Step 5: Re-run the native tests**

```bash
cd WhiskeyCompassCAN && pio test -e native
```

Expected: PASS, 25 tests, 0 failures.

- [ ] **Step 6: Commit**

```bash
git add WhiskeyCompassCAN/src/CAN.h WhiskeyCompassCAN/src/CAN.cpp WhiskeyCompassCAN/src/Configuration.h
git commit -m "feat(whiskey-compass): receive heading 0x107 and lights 0x203 over CAN"
```

---

### Task 5: DCU plumbing

Deliverable: `pio run` succeeds in `DCU`, and the DCU turns serial message `0x0B` into CAN frame `0x107` with a 5-second staleness resend, plus a `co<deg>` bench command.

**Files:**
- Modify: `DCU/src/DCUReceiver.h`
- Modify: `DCU/src/DCUReceiver.cpp`
- Modify: `DCU/src/BenchDebug.h`
- Modify: `DCU/src/BenchDebug.cpp`

**Interfaces:**
- Consumes: `MessageType::SerialMessageCompass`, `CanMessageId::compass` from Task 1; `packBE16`, `isStale`, `MessageMeta` which already exist in the DCU.
- Produces: `DCUReceiver::sendCompass()`, `BenchDebug::sendCompass()`. Nothing downstream consumes these.

- [ ] **Step 1: Declare the receiver state**

In `DCU/src/DCUReceiver.h`, add the send method after `sendAltimeterVsi();`:

```cpp
        void sendCompass();
```

Add the stored value after the altimeter/VSI block:

```cpp
        // Magnetic compass (whiskey compass)
        uint16_t compassDeg100 = 0;
```

Add the metadata after `altimeterVsiMeta;`:

```cpp
        MessageMeta compassMeta;
```

- [ ] **Step 2: Initialise the metadata**

In `DCU/src/DCUReceiver.cpp`, next to `airspeedMeta = {0, 5000};` in the constructor, add:

```cpp
  compassMeta = {0, 5000};
```

- [ ] **Step 3: Parse the serial message**

In `DCU/src/DCUReceiver.cpp::handleFrame`, add a case after the `SerialMessageAltimeterVsi` block:

```cpp
  case MessageType::SerialMessageCompass:
  {
    // Payload: float headingDegMag (4 bytes), degrees magnetic
    if (len != 4)
      return;

    float heading;
    memcpy(&heading, payload + 0, 4);

    // compass_heading_deg_mag can read slightly negative or above 360 depending
    // on X-Plane's internal state. Wrapping here is load-bearing: an unwrapped
    // negative cast to uint16 would send the card on a full spurious turn.
    heading = fmodf(heading, 360.f);
    if (heading < 0.f)
      heading += 360.f;

    uint16_t deg100 = static_cast<uint16_t>(heading * 100.f + 0.5f);
    if (deg100 > 35999)
      deg100 = 0;

    if (deg100 != compassDeg100)
    {
      compassDeg100 = deg100;
      DEBUGLOG_PRINTLN(String(F("Received MSG_COMPASS Datagram deg*100: ")) + String(deg100));
      sendCompass();
    }
    break;
  }
```

The `> 35999` guard catches the rounding edge: a heading of 359.999 rounds to 36000, which is a full turn, i.e. north.

- [ ] **Step 4: Add the CAN send path**

In `DCU/src/DCUReceiver.cpp`, after `sendAltimeterVsi()`:

```cpp
void DCUReceiver::sendCompass()
{
  byte data[8] = {0};

  // [0..1] magnetic heading, degrees * 100. [2..7] reserved.
  packBE16(data + 0, compassDeg100);

  canBus->sendMessage(CanMessageId::compass, 8, data);

  // Update last send timestamp for maxAge resync
  compassMeta.lastSendTimestamp = millis();
}
```

- [ ] **Step 5: Add the staleness resync**

At the end of `DCUReceiver::checkMaxAgeResync`, after the altimeter/VSI check:

```cpp
  // Check compass message
  if (isStale(compassMeta.lastSendTimestamp, now, compassMeta.maxAgeMs))
  {
    DEBUGLOG_PRINTLN(String(F("MaxAge resync for compass")));
    sendCompass();
  }
```

This is how the instrument recovers its heading after a plugin or DCU restart, with no onChange to react to.

- [ ] **Step 6: Add the bench command**

In `DCU/src/BenchDebug.h`, declare after `void sendAltimeterVsi();`:

```cpp
        void sendCompass();
```

and add the stored value next to the other bench setpoints:

```cpp
        float compassHeadingDeg = 0.;
```

In `DCU/src/BenchDebug.cpp`, add the sender after `sendAltimeterVsi()`:

```cpp
void BenchDebug::sendCompass() {
    byte data[8] = {0};
    // [0..1] magnetic heading, degrees * 100. [2..7] reserved.
    float wrapped = fmodf(compassHeadingDeg, 360.f);
    if (wrapped < 0.f)
        wrapped += 360.f;
    packBE16(data + 0, static_cast<uint16_t>(wrapped * 100.f + 0.5f));

    Serial.println(String(F("Send Compass: ")) + compassHeadingDeg + F(" deg"));

    canBus->sendMessage(CanMessageId::compass, 8, data);
}
```

Add the command in the `handleAltimeterInput` chain, next to the `vs` branch:

```cpp
    } else if (command.startsWith("co")) {
        String rString = command.substring(2);
        rString.trim();
        compassHeadingDeg = rString.toFloat();
        Serial.println(String(F("Compass heading set to "))+compassHeadingDeg+F(" deg"));
        sendCompass();
        return true;
```

And the help line, after the `vs` line in the `?` branch:

```cpp
        Serial.println(F("co<deg>: set magnetic compass heading"));
```

- [ ] **Step 7: Build the DCU in both modes**

```bash
cd DCU && pio run
```

Expected: successful build. Then flip `BENCHDEBUG` in `DCU/src/Configuration.h` to `1`, run `pio run` again to confirm the bench path compiles, and flip it back to its original value. If `fmodf` is unavailable, add `#include <math.h>` to the file that needs it.

- [ ] **Step 8: Commit**

```bash
git add DCU/src/DCUReceiver.h DCU/src/DCUReceiver.cpp DCU/src/BenchDebug.h DCU/src/BenchDebug.cpp
git commit -m "feat(dcu): forward compass heading from serial 0x0B to CAN 0x107"
```

---

### Task 6: Plugin plumbing

Deliverable: `./build-macos.sh` succeeds in `DCUProviderPlugin`, and the plugin sends the magnetic compass heading at 50 Hz.

**Files:**
- Modify: `DCUProviderPlugin/src/DataRefManager.h`
- Modify: `DCUProviderPlugin/src/DataRefManager.cpp`
- Modify: `DCUProviderPlugin/src/DCUProvider.h`
- Modify: `DCUProviderPlugin/src/DCUProvider.cpp`

**Interfaces:**
- Consumes: `MessageType::SerialMessageCompass` from Task 1; the existing `readFloat`, `msgQueue_->enqueueTx` and per-message accumulator pattern.
- Produces: `DataRefManager::getCompassHeadingDegMag()`. Nothing downstream consumes it.

- [ ] **Step 1: Declare the dataref accessor**

In `DCUProviderPlugin/src/DataRefManager.h`, after the altimeter/VSI accessors:

```cpp
    // Magnetic compass (whiskey compass)
    float getCompassHeadingDegMag() const;  // degrees magnetic, 0..360
```

and next to `dr_ias` in the member section:

```cpp
    XPLMDataRef dr_compass_heading = nullptr;
```

- [ ] **Step 2: Find and read the dataref**

In `DCUProviderPlugin/src/DataRefManager.cpp`, next to the `dr_ias` / `dr_tas` lookups:

```cpp
    dr_compass_heading = XPLMFindDataRef("sim/cockpit2/gauges/indicators/compass_heading_deg_mag");
```

and after `getVsiFpm()`:

```cpp
// Magnetic compass
float DataRefManager::getCompassHeadingDegMag() const
{
    return readFloat(dr_compass_heading, 0.0f);
}
```

- [ ] **Step 3: Add the send rate and accumulator**

In `DCUProviderPlugin/src/DCUProvider.h`, next to `ALTIMETER_VSI_RATE`:

```cpp
    static constexpr float COMPASS_RATE = 50.0f; // Hz
```

and next to `altimeterVsiAccumulator_`:

```cpp
    float compassAccumulator_ = 0.0f;
```

- [ ] **Step 4: Send the message**

In `DCUProviderPlugin/src/DCUProvider.cpp`, after the altimeter/VSI block:

```cpp
    // ============ Compass Data (50 Hz) ============
    compassAccumulator_ += dt;
    float compassRate = 1.0f / COMPASS_RATE;

    if (compassAccumulator_ >= compassRate)
    {
        struct CompassData
        {
            float headingDegMag;
        };

        CompassData compass;
        compass.headingDegMag = dataRefMgr_->getCompassHeadingDegMag();

        msgQueue_->enqueueTx(MessageType::SerialMessageCompass, &compass, sizeof(compass));

        compassAccumulator_ = 0.0f;
    }
```

- [ ] **Step 5: Build the plugin**

```bash
cd DCUProviderPlugin && ./build-macos.sh
```

Expected: successful release build. This is a compile-only check — whether the dataref actually reads requires X-Plane, which is part of the bench verification below, not this task.

- [ ] **Step 6: Commit**

```bash
git add DCUProviderPlugin/src/DataRefManager.h DCUProviderPlugin/src/DataRefManager.cpp DCUProviderPlugin/src/DCUProvider.h DCUProviderPlugin/src/DCUProvider.cpp
git commit -m "feat(plugin): send magnetic compass heading to the DCU at 50Hz"
```

---

### Task 7: Repo documentation

Deliverable: the root `CLAUDE.md` describes the new board, so the next session finds it without reading the tree.

**Files:**
- Modify: `CLAUDE.md`

**Interfaces:**
- Consumes: nothing.
- Produces: nothing.

- [ ] **Step 1: Add the board to the project list**

In the "Repo layout" section, add `WhiskeyCompassCAN` to the board-projects list. The line currently reading:

```
  `VerticalSpeedCAN`): independent PlatformIO/Arduino projects, each with its own
```

becomes:

```
  `VerticalSpeedCAN`, `WhiskeyCompassCAN`): independent PlatformIO/Arduino projects, each with its own
```

- [ ] **Step 2: Add it to the native-test list**

In the "Commands" section, the line:

```
                          # AirspeedCAN, AltimeterCAN, DCU, RudderCAN and VerticalSpeedCAN have real
```

becomes:

```
                          # AirspeedCAN, AltimeterCAN, DCU, RudderCAN, VerticalSpeedCAN and
                          # WhiskeyCompassCAN have real
```

- [ ] **Step 3: Add the tested header to the list**

The line listing the Arduino-free headers:

```
`AltimeterCalibration.h`); `src/` stays Arduino-coupled.
```

becomes:

```
`AltimeterCalibration.h`, `CompassGeometry.h`); `src/` stays Arduino-coupled.
```

- [ ] **Step 4: Note the compass on the CAN protocol side**

In the "CAN protocol" section, after the `SerialMessageId.h` bullet, add:

```markdown
- The whiskey compass is the simplest full-chain example to copy for a new read-only
  instrument: dataref in `DCUProviderPlugin/src/DataRefManager`, serial `0x0B` in
  `SerialMessageId.h`, `DCUReceiver::sendCompass()` in the DCU, CAN `0x107` consumed by
  `WhiskeyCompassCAN`. See `docs/superpowers/specs/2026-09-14-whiskey-compass-can-design.md`.
```

- [ ] **Step 5: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: record WhiskeyCompassCAN in the repo guide"
```

---

## Bench verification (after Task 7, requires hardware)

Not automatable — the repo has no hardware-in-the-loop harness. Run these on the rig
before merging, and report what was actually observed rather than what was expected.

- [ ] Flash with `BENCHDEBUG 1`. `ho`, then `sd` — confirm `homed: yes`.
- [ ] Home ten times in a row. The card must land on the same mark every time; a
      spread means the Hall-window midpoint logic is picking up only one edge.
- [ ] `hd0`, `hd90`, `hd180`, `hd270` — card follows, and each move takes the short way.
- [ ] `hd359` then `hd1` — the card must move ~2 degrees forward, not ~358 backwards.
      Then `hd1` then `hd359` for the reverse.
- [ ] If the card turns the wrong way, set `kCardInversed = true` in `Configuration.h`
      and reflash.
- [ ] `li0`, `li128`, `li255` — all populated LEDs dim together.
- [ ] Jog the card onto the painted N mark, `cz`, power-cycle, `ho` — the card must come
      back to N. This is the EEPROM round trip.
- [ ] `cw`, then `ho` — the card returns to the raw Hall zero.
- [ ] Flash with `BENCHDEBUG 0` and connect the real bus. Confirm the DCU lists the
      compass node as alive and does not report it going quiet during the homing run —
      that is the non-blocking homing requirement, and the bench console cannot test it.
- [ ] From the DCU bench console, `co90` / `co270` — card follows over CAN.
- [ ] With X-Plane running, turn the aircraft and confirm the card tracks the panel's
      magnetic heading, and that it recovers correctly after restarting the plugin.
