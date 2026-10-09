# AttitudeCAN Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Neues CAN-Instrument `AttitudeCAN` (künstlicher Horizont, zwei verschachtelte 28BYJ-48 auf einem Nano) plus die komplette Kette Plugin → DCU → Board.

**Architecture:** Board im Stil von `WhiskeyCompassCAN`: Arduino-freie Geometrie in `include/AttitudeGeometry.h` (nativ getestet), `AttitudeIndicator` mit nicht-blockierender Homing-State-Machine (Roll zuerst, Pitch gegengeführt), `CAN` als `InstrumentCAN`-Subklasse. DCU repackt Serial `0x0F` (zwei Floats) in CAN `0x101` (zwei int16 BE), Plugin liest `pitch/roll_vacuum_deg_pilot` mit 50 Hz.

**Tech Stack:** PlatformIO (atmelavr, nanoatmega328new), Unity (native), `coryjfowler/mcp_can`, vendored `CheapStepper`, X-Plane SDK (Plugin, CMake).

**Spec:** `docs/superpowers/specs/2026-10-09-attitude-can-design.md`

> **Historischer Stand.** Task 3/4 beschreiben die Pitch-Kopplung als gefaltetes Absolutziel
> (`pitchMotorTarget` + Shortest-Path auf dem Motor). Das war falsch: ab |Delta| > halbe Umdrehung
> fährt die *Anzeige* durch die Anschläge. Der Branch nutzt seit `95da02e` GyroDrives
> Delta-Formulierung (`pitchMotorDelta`), außerdem `axisDisplayDegree` statt `rollDisplayDegree`
> und ein `kPitchInversed`-Flag (`058ab32`). Die Spec ist nachgezogen; dieser Plan nicht.

## Global Constraints

- Branch `feature/attitude-can` existiert bereits (ab `feature/hsi-can`-HEAD). **Kein Worktree** — der Nutzer hat das explizit verneint. Die uncommittete Änderung an `HSICAN/src/Configuration.h` gehört nicht zu dieser Arbeit: nie `git add -A`, immer Dateien einzeln stagen.
- `pio` liegt in `~/.platformio/penv/bin/pio` (im PATH). Alle `pio`-Befehle aus dem jeweiligen Projektverzeichnis.
- Node-ID `attitudeNodeId = 0x0C`, CAN `attitude = 0x101`, Serial `SerialMessageAttitude = 0x0F`. Payload-Reihenfolge überall **Pitch, dann Roll**.
- CAN-Mehrbyte-Felder big-endian; Serial-Payloads host order (packed structs).
- Pins: `/CS` D10, `/INT` D2, Roll-Coils {4,7,8,9}, Pitch-Coils {A0,A1,A2,A3}, Hall Roll D3, Hall Pitch D6, LED D5. Halls `INPUT_PULLUP`, aktiv LOW, gepollt.
- `kPitchMaxDegree = 30.0`, `kPitchScale = 0.76923077`, `kRollInversed = true`, Zero-Defaults Roll 1 / Pitch −1, RPM max 14 / min 4, 4096 Steps beide Achsen.
- **Homing-Reihenfolge Roll → Pitch ist eine mechanische Vorgabe.** Jede Roll-Fahrt während des Homings bekommt die Pitch-Gegenbewegung (gleiche Stepzahl, gleicher Delay, umgekehrtes Vorzeichen). Pitch wird erst gehomt, wenn Roll auf 0 steht.
- `StepperBoard` wird nicht angefasst.
- Commit-Messages enden mit `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>`.
- Kein Log pro CAN-Frame im Board (`Serial.print` blockiert `run()`).

## Review Focus

1. **Pitch außerhalb ±30°** (X-Plane liefert beim Taumeln ±90): muss geklemmt werden, nie in den Anschlag. → Test `test_pitch_is_clamped_before_scaling` (Task 3).
2. **Negative `getPosition()`** der vendored CheapStepper (int32, geht nach CCW-Fahrt negativ): jede Zielrechnung normalisiert erst. → Test `test_pitch_motor_target_folds_negative_display_steps` (Task 3).
3. **NaN / Riesenwerte im Dataref** (Flugzeug noch nicht geladen): dürfen den int16 nicht umkippen oder UB in `lroundf` auslösen. → Tests `test_nan_becomes_zero`, `test_values_past_the_int16_are_clamped` (Task 8).
4. **Roll-Setpoint exakt ±180°**: Shortest-Path-Gleichstand muss deterministisch sein (CW). → Test `test_exactly_half_a_turn_resolves_clockwise` (Task 2, Kopie aus Compass).
5. **`0x101` kürzer als 4 Byte**: ignorieren, nicht aus `data[]` hinauslesen. → Längencheck in `CAN::handleFrame` (Task 6) + Bench-Punkt in Task 10 (CANDebugNode-Frame mit len 2).

---

### Task 1: Protokoll-Header in `shared/CANBase`

**Files:**
- Modify: `shared/CANBase/include/CanNodeId.h`
- Modify: `shared/CANBase/include/CanMessageId.h`
- Modify: `shared/CANBase/include/SerialMessageId.h`

**Interfaces:**
- Produces: `CanNodeId::attitudeNodeId`, `CanMessageId::attitude`, `MessageType::SerialMessageAttitude`, `struct AttitudeMessage { float pitchDeg; float rollDeg; }` (8 Byte).

- [ ] **Step 1: Node-ID**

In `CanNodeId.h` nach `hsiNodeId = 0x0B` einfügen (Komma an `hsiNodeId` ergänzen):

```cpp
  hsiNodeId = 0x0B,
  attitudeNodeId = 0x0C
```

- [ ] **Step 2: CAN-Message-ID**

In `CanMessageId.h` direkt nach dem `airspeed = 0x100,`-Block einfügen:

```cpp
  // 0x101: Attitude indicator (Gateway -> Instrument, 50Hz)
  // [0..1] pitch int16, degrees * 100 (sim/cockpit2/gauges/indicators/pitch_vacuum_deg_pilot)
  // [2..3] roll  int16, degrees * 100 (sim/cockpit2/gauges/indicators/roll_vacuum_deg_pilot)
  // [4..7] reserved
  // Both signed. The board clamps pitch to its mechanical +-30 degree window
  // itself; the gateway forwards whatever the sim says, including the tumble
  // of a failed vacuum gyro.
  attitude = 0x101,
```

- [ ] **Step 3: Serial-Message**

In `SerialMessageId.h` nach `SerialMessageHsiInput = 0x0E,` einfügen:

```cpp
    // Plugin -> DCU. Payload: AttitudeMessage (8 bytes, host order).
    // Repacked by the DCU into CAN 0x101 as two big-endian int16 of degrees * 100.
    SerialMessageAttitude = 0x0F,
```

Und nach den HSI-Structs (vor den `static_assert`s), in beiden `#if defined(_MSC_VER)`-Zweigen:

```cpp
#if defined(_MSC_VER)
    #pragma pack(push, 1)
    struct AttitudeMessage {
        float pitchDeg;   // pitch_vacuum_deg_pilot
        float rollDeg;    // roll_vacuum_deg_pilot
    };
    #pragma pack(pop)
#else
    struct __attribute__((packed)) AttitudeMessage {
        float pitchDeg;
        float rollDeg;
    };
#endif
```

Bei den `static_assert`s ergänzen:

```cpp
static_assert(sizeof(AttitudeMessage) == 8, "AttitudeMessage must be 8 bytes on the wire");
```

- [ ] **Step 4: Compile-Check über zwei Konsumenten**

```bash
cd DCU && pio run -e megaatmega2560 && pio test -e native
cd ../WhiskeyCompassCAN && pio run -e nano
```
Expected: beide Builds grün, DCU-Native-Tests PASS (Header werden von jedem Board eingezogen).

- [ ] **Step 5: Commit**

```bash
git add shared/CANBase/include/CanNodeId.h shared/CANBase/include/CanMessageId.h shared/CANBase/include/SerialMessageId.h
git commit -m "feat(canbase): add attitude node, CAN 0x101 and serial 0x0F IDs

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 2: Projektgerüst + Geometrie-Kopien mit Tests

**Files:**
- Create: `AttitudeCAN/platformio.ini`
- Create: `AttitudeCAN/lib/CheapStepper/library.json`, `AttitudeCAN/lib/CheapStepper/src/CheapStepper.cpp`, `AttitudeCAN/lib/CheapStepper/src/CheapStepper.h` (Kopie aus `WhiskeyCompassCAN/lib/CheapStepper/`)
- Create: `AttitudeCAN/include/AttitudeGeometry.h`
- Create: `AttitudeCAN/test/test_geometry/test_geometry.cpp`

**Interfaces:**
- Produces: `normalizeStepPosition(int32_t, uint32_t) -> uint32_t`, `shortestPathSteps(uint32_t cur, uint32_t tgt, uint32_t total) -> int32_t`, `homingWindowCentre(uint32_t zeroEnd, uint32_t zeroStart, uint32_t total, int32_t* centre) -> bool`, `normalizeZeroAdjust(int32_t) -> int16_t`, `accumulateZeroAdjust(int16_t, int32_t) -> int16_t`, `jogDegreesFromPosition(uint32_t, uint32_t) -> int32_t`.

- [ ] **Step 1: platformio.ini**

```ini
; PlatformIO Project Configuration File

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
build_flags = -DUNITY_INCLUDE_DOUBLE
```

- [ ] **Step 2: CheapStepper vendoren**

```bash
mkdir -p AttitudeCAN/lib/CheapStepper/src AttitudeCAN/include AttitudeCAN/test/test_geometry AttitudeCAN/src
cp WhiskeyCompassCAN/lib/CheapStepper/library.json AttitudeCAN/lib/CheapStepper/
cp WhiskeyCompassCAN/lib/CheapStepper/src/CheapStepper.cpp WhiskeyCompassCAN/lib/CheapStepper/src/CheapStepper.h AttitudeCAN/lib/CheapStepper/src/
```

In `AttitudeCAN/lib/CheapStepper/library.json` das Feld `"keywords"` auf `"28byj-48, uln2003, stepper, attitude"` ändern. Sonst nichts.

- [ ] **Step 3: Fehlschlagende Tests für die Kopien schreiben**

`AttitudeCAN/test/test_geometry/test_geometry.cpp`:

```cpp
#include <unity.h>
#include "AttitudeGeometry.h"

void setUp(void) {}
void tearDown(void) {}

// --- step position normalisation -------------------------------------------

void test_positions_inside_one_turn_pass_through(void) {
    TEST_ASSERT_EQUAL_UINT32(0, normalizeStepPosition(0, 4096));
    TEST_ASSERT_EQUAL_UINT32(1024, normalizeStepPosition(1024, 4096));
    TEST_ASSERT_EQUAL_UINT32(4095, normalizeStepPosition(4095, 4096));
}

void test_negative_positions_fold_forward(void) {
    TEST_ASSERT_EQUAL_UINT32(4095, normalizeStepPosition(-1, 4096));
    TEST_ASSERT_EQUAL_UINT32(3072, normalizeStepPosition(-1024, 4096));
}

void test_zero_total_steps_cannot_divide_by_zero(void) {
    TEST_ASSERT_EQUAL_UINT32(0, normalizeStepPosition(500, 0));
}

// --- shortest path ----------------------------------------------------------

void test_a_quarter_turn_goes_the_direct_way(void) {
    TEST_ASSERT_EQUAL_INT32(1024, shortestPathSteps(0, 1024, 4096));
    TEST_ASSERT_EQUAL_INT32(-1024, shortestPathSteps(1024, 0, 4096));
}

void test_crossing_zero_takes_the_short_way(void) {
    // 359 -> 1 degree is 23 steps forward, not 4073 backwards.
    TEST_ASSERT_EQUAL_INT32(23, shortestPathSteps(4085, 12, 4096));
    TEST_ASSERT_EQUAL_INT32(-23, shortestPathSteps(12, 4085, 4096));
}

void test_exactly_half_a_turn_resolves_clockwise(void) {
    TEST_ASSERT_EQUAL_INT32(2048, shortestPathSteps(0, 2048, 4096));
}

void test_no_move_needed_when_already_on_target(void) {
    TEST_ASSERT_EQUAL_INT32(0, shortestPathSteps(777, 777, 4096));
}

// --- homing window ----------------------------------------------------------

void test_a_normal_window_halves_to_its_centre(void) {
    int32_t centre = -1;
    TEST_ASSERT_TRUE(homingWindowCentre(60, 0, 4096, &centre));
    TEST_ASSERT_EQUAL_INT32(30, centre);
}

// The capture frame the state machine uses: end edge reset to 0, start edge
// captured as a negative (folded) position.
void test_a_window_ending_exactly_at_zero_is_handled(void) {
    int32_t centre = -1;
    TEST_ASSERT_TRUE(homingWindowCentre(0, 4066, 4096, &centre));
    TEST_ASSERT_EQUAL_INT32(15, centre);
}

void test_a_wrapped_difference_is_rejected_instead_of_becoming_180_degrees(void) {
    int32_t centre = 4242;
    TEST_ASSERT_FALSE(homingWindowCentre(0, 30, 4096, &centre));
    TEST_ASSERT_EQUAL_INT32(4242, centre);
}

void test_a_window_wider_than_an_eighth_of_a_turn_is_rejected(void) {
    int32_t centre = -1;
    TEST_ASSERT_FALSE(homingWindowCentre(1024, 0, 4096, &centre));
}

// --- zero adjust ------------------------------------------------------------

void test_offsets_fold_into_minus180_to_179(void) {
    TEST_ASSERT_EQUAL_INT16(-170, normalizeZeroAdjust(190));
    TEST_ASSERT_EQUAL_INT16(170, normalizeZeroAdjust(-190));
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(360));
}

void test_a_jog_is_added_to_the_stored_offset(void) {
    TEST_ASSERT_EQUAL_INT16(8, accumulateZeroAdjust(5, 3));
    TEST_ASSERT_EQUAL_INT16(2, accumulateZeroAdjust(5, -3));
}

void test_a_backward_jog_reads_as_negative_degrees(void) {
    TEST_ASSERT_EQUAL_INT32(90, jogDegreesFromPosition(1024, 4096));
    TEST_ASSERT_EQUAL_INT32(-90, jogDegreesFromPosition(4096 - 1024, 4096));
    TEST_ASSERT_EQUAL_INT32(0, jogDegreesFromPosition(100, 0));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_positions_inside_one_turn_pass_through);
    RUN_TEST(test_negative_positions_fold_forward);
    RUN_TEST(test_zero_total_steps_cannot_divide_by_zero);
    RUN_TEST(test_a_quarter_turn_goes_the_direct_way);
    RUN_TEST(test_crossing_zero_takes_the_short_way);
    RUN_TEST(test_exactly_half_a_turn_resolves_clockwise);
    RUN_TEST(test_no_move_needed_when_already_on_target);
    RUN_TEST(test_a_normal_window_halves_to_its_centre);
    RUN_TEST(test_a_window_ending_exactly_at_zero_is_handled);
    RUN_TEST(test_a_wrapped_difference_is_rejected_instead_of_becoming_180_degrees);
    RUN_TEST(test_a_window_wider_than_an_eighth_of_a_turn_is_rejected);
    RUN_TEST(test_offsets_fold_into_minus180_to_179);
    RUN_TEST(test_a_jog_is_added_to_the_stored_offset);
    RUN_TEST(test_a_backward_jog_reads_as_negative_degrees);
    return UNITY_END();
}
```

- [ ] **Step 4: Test laufen lassen, muss scheitern**

```bash
cd AttitudeCAN && pio test -e native
```
Expected: Compile-Fehler `AttitudeGeometry.h: No such file`.

- [ ] **Step 5: `AttitudeGeometry.h` mit den Kopien anlegen**

```cpp
#ifndef ATTITUDEGEOMETRY_H
#define ATTITUDEGEOMETRY_H

#include <math.h>
#include <stdint.h>

// Geometry for the attitude indicator: two nested 28BYJ-48 axes (the pitch
// motor rides on the roll frame), heading-style shortest paths on both, the
// mechanical pitch window, the Hall-window centre for homing and the EEPROM
// zero offsets. Deliberately Arduino-free so all of it runs under the native
// Unity tests in test/test_geometry (pio test -e native).

// --- step positions ---------------------------------------------------------

// Folds any step position, negative included, into 0..totalSteps-1. The
// vendored CheapStepper reports an int32_t that goes negative after a
// counter-clockwise move, so this runs on everything out of getPosition().
inline uint32_t normalizeStepPosition(int32_t position, uint32_t totalSteps)
{
    if (totalSteps == 0)
    {
        return 0;
    }
    const int32_t total = (int32_t)totalSteps;
    return (uint32_t)((position % total + total) % total);
}

// Signed step delta from `current` to `target`, the short way round. Positive
// is clockwise. At exactly half a turn both ways are equal; the tie breaks CW.
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

// --- homing window ----------------------------------------------------------

// Centre of the Hall window from its two edge captures, as an always-forward
// modular distance from zeroStart to zeroEnd. A Hall window spans a few
// degrees, so anything wider than an eighth of a turn is a misread (typically
// a phase that latched before the axis stepped, leaving one edge at 0 and the
// difference wrapped to most of a revolution — halving THAT would park the
// axis ~180 degrees off and still report success). Returns false and leaves
// *centreSteps untouched in that case.
inline bool homingWindowCentre(uint32_t zeroEnd, uint32_t zeroStart,
                               uint32_t totalSteps, int32_t *centreSteps)
{
    if (totalSteps == 0 || centreSteps == 0)
    {
        return false;
    }
    const uint32_t span = (zeroEnd + totalSteps - (zeroStart % totalSteps)) % totalSteps;
    if (span > totalSteps / 8)
    {
        return false;
    }
    *centreSteps = (int32_t)(span / 2);
    return true;
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

// After homing an axis sits at 0 with the stored offset already applied, so a
// jog onto the true zero means the new offset is the SUM. Replacing instead of
// adding throws away every previous pass.
inline int16_t accumulateZeroAdjust(int16_t stored, int32_t jogDegrees)
{
    return normalizeZeroAdjust((int32_t)stored + jogDegrees);
}

// A normalised position just below a full turn is a small backwards jog, not
// most of a turn forwards. Fold onto the short way round, then to degrees.
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

#endif // ATTITUDEGEOMETRY_H
```

- [ ] **Step 6: Tests laufen lassen**

```bash
cd AttitudeCAN && pio test -e native
```
Expected: 14 Tests PASS.

- [ ] **Step 7: Commit**

```bash
git add AttitudeCAN/platformio.ini AttitudeCAN/lib AttitudeCAN/include AttitudeCAN/test
git commit -m "feat(attitude-can): scaffold board project with tested geometry basics

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 3: Geometrie der verschachtelten Achsen (TDD)

**Files:**
- Modify: `AttitudeCAN/include/AttitudeGeometry.h`
- Modify: `AttitudeCAN/test/test_geometry/test_geometry.cpp`

**Interfaces:**
- Consumes: `normalizeStepPosition`, `shortestPathSteps` aus Task 2.
- Produces:
  - `int32_t signedDegreesToSteps(double degrees, uint32_t totalSteps)` — gerundet, NICHT normalisiert.
  - `double pitchDisplayDegree(double pitchDeg, double maxDegree, double scale)` — clamp, dann skalieren.
  - `double rollDisplayDegree(double rollDeg, bool inversed)`.
  - `uint32_t pitchMotorTarget(int32_t pitchDisplaySteps, uint32_t rollTargetSteps, uint32_t totalSteps)`.
  - `int32_t pitchDisplaySteps(int32_t pitchMotorPos, int32_t rollPos, uint32_t totalSteps)` — Umkehrung, gefaltet auf −total/2..total/2.
  - `void coordinatedDelays(int32_t stepsRoll, int32_t stepsPitch, uint32_t delayRollMax, uint32_t delayPitchMax, uint32_t *delayRoll, uint32_t *delayPitch)`.

- [ ] **Step 1: Tests schreiben (vor dem `int main`)**

```cpp
// --- signed degrees to steps -----------------------------------------------

void test_signed_degrees_round_to_the_nearest_step(void) {
    TEST_ASSERT_EQUAL_INT32(0, signedDegreesToSteps(0.0, 4096));
    TEST_ASSERT_EQUAL_INT32(1024, signedDegreesToSteps(90.0, 4096));
    TEST_ASSERT_EQUAL_INT32(-1024, signedDegreesToSteps(-90.0, 4096));
    // 0.05 degrees is 0.57 steps: rounds to 1, and symmetrically to -1.
    TEST_ASSERT_EQUAL_INT32(1, signedDegreesToSteps(0.05, 4096));
    TEST_ASSERT_EQUAL_INT32(-1, signedDegreesToSteps(-0.05, 4096));
    TEST_ASSERT_EQUAL_INT32(0, signedDegreesToSteps(90.0, 0));
}

// --- pitch window and roll sense -------------------------------------------

// X-Plane hands out +-90 while a failed vacuum gyro tumbles. The mechanism
// stops at +-30, so the clamp has to happen BEFORE the scale factor.
void test_pitch_is_clamped_before_scaling(void) {
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 30.0 * 0.76923077, pitchDisplayDegree(90.0, 30.0, 0.76923077));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -30.0 * 0.76923077, pitchDisplayDegree(-90.0, 30.0, 0.76923077));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 10.0 * 0.76923077, pitchDisplayDegree(10.0, 30.0, 0.76923077));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, pitchDisplayDegree(0.0, 30.0, 0.76923077));
}

void test_roll_is_inverted_only_when_asked(void) {
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -20.0, rollDisplayDegree(20.0, true));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 20.0, rollDisplayDegree(20.0, false));
}

// --- nested axes ------------------------------------------------------------

// Roll at zero: the pitch motor IS the pitch display.
void test_pitch_motor_equals_display_when_roll_is_zero(void) {
    TEST_ASSERT_EQUAL_UINT32(100, pitchMotorTarget(100, 0, 4096));
    TEST_ASSERT_EQUAL_UINT32(4096 - 100, pitchMotorTarget(-100, 0, 4096));
}

// The pitch motor rides on the roll frame, so holding the display still over
// a full roll turn means the pitch motor counter-rotates exactly one turn.
void test_holding_pitch_over_a_full_roll_counter_rotates_the_pitch_motor(void) {
    int32_t pitchMotorTravel = 0;
    uint32_t motorPos = pitchMotorTarget(0, 0, 4096);
    for (uint32_t rollPos = 64; rollPos <= 4096; rollPos += 64) {
        const uint32_t next = pitchMotorTarget(0, rollPos % 4096, 4096);
        pitchMotorTravel += shortestPathSteps(motorPos, next, 4096);
        motorPos = next;
    }
    TEST_ASSERT_EQUAL_INT32(-4096, pitchMotorTravel);
}

// Roll crossing zero (359 -> 1 degree) with pitch held must not make the pitch
// motor jump most of a turn.
void test_roll_crossing_zero_keeps_the_pitch_motor_delta_small(void) {
    const uint32_t before = pitchMotorTarget(200, 4085, 4096);
    const uint32_t after = pitchMotorTarget(200, 12, 4096);
    TEST_ASSERT_EQUAL_INT32(-23, shortestPathSteps(before, after, 4096));
}

// getPosition() can be negative; a negative display request too.
void test_pitch_motor_target_folds_negative_display_steps(void) {
    TEST_ASSERT_EQUAL_UINT32(4096 - 300, pitchMotorTarget(-100, 200, 4096));
    TEST_ASSERT_EQUAL_UINT32(0, pitchMotorTarget(-100, 200, 0));
}

void test_display_steps_are_recovered_from_motor_and_roll(void) {
    TEST_ASSERT_EQUAL_INT32(100, pitchDisplaySteps(100, 0, 4096));
    TEST_ASSERT_EQUAL_INT32(100, pitchDisplaySteps(4096 - 100, 200, 4096));
    TEST_ASSERT_EQUAL_INT32(-100, pitchDisplaySteps(-300, 200, 4096));
    TEST_ASSERT_EQUAL_INT32(0, pitchDisplaySteps(50, 0, 0));
}

// --- coordinated delays -----------------------------------------------------

void test_equal_travel_keeps_both_axes_at_their_max_rate(void) {
    uint32_t dr = 0, dp = 0;
    coordinatedDelays(100, 100, 1000, 1200, &dr, &dp);
    TEST_ASSERT_EQUAL_UINT32(1000, dr);
    TEST_ASSERT_EQUAL_UINT32(1200, dp);
}

// The axis with more steps sets the pace; the other is slowed so that both
// arrive together. Sign of the travel is irrelevant.
void test_the_shorter_axis_is_slowed_proportionally(void) {
    uint32_t dr = 0, dp = 0;
    coordinatedDelays(200, -100, 1000, 1200, &dr, &dp);
    TEST_ASSERT_EQUAL_UINT32(1000, dr);
    TEST_ASSERT_EQUAL_UINT32(2000, dp);

    coordinatedDelays(-50, 150, 1000, 1200, &dr, &dp);
    TEST_ASSERT_EQUAL_UINT32(3600, dr);
    TEST_ASSERT_EQUAL_UINT32(1200, dp);
}

void test_an_idle_axis_does_not_divide_by_zero(void) {
    uint32_t dr = 0, dp = 0;
    coordinatedDelays(0, 150, 1000, 1200, &dr, &dp);
    TEST_ASSERT_EQUAL_UINT32(1000, dr);
    TEST_ASSERT_EQUAL_UINT32(1200, dp);
    coordinatedDelays(150, 0, 1000, 1200, &dr, &dp);
    TEST_ASSERT_EQUAL_UINT32(1000, dr);
    TEST_ASSERT_EQUAL_UINT32(1200, dp);
}
```

In `main` ergänzen:

```cpp
    RUN_TEST(test_signed_degrees_round_to_the_nearest_step);
    RUN_TEST(test_pitch_is_clamped_before_scaling);
    RUN_TEST(test_roll_is_inverted_only_when_asked);
    RUN_TEST(test_pitch_motor_equals_display_when_roll_is_zero);
    RUN_TEST(test_holding_pitch_over_a_full_roll_counter_rotates_the_pitch_motor);
    RUN_TEST(test_roll_crossing_zero_keeps_the_pitch_motor_delta_small);
    RUN_TEST(test_pitch_motor_target_folds_negative_display_steps);
    RUN_TEST(test_display_steps_are_recovered_from_motor_and_roll);
    RUN_TEST(test_equal_travel_keeps_both_axes_at_their_max_rate);
    RUN_TEST(test_the_shorter_axis_is_slowed_proportionally);
    RUN_TEST(test_an_idle_axis_does_not_divide_by_zero);
```

- [ ] **Step 2: Tests laufen lassen, müssen scheitern**

```bash
cd AttitudeCAN && pio test -e native
```
Expected: Compile-Fehler, `signedDegreesToSteps` etc. nicht deklariert.

- [ ] **Step 3: Implementierung in `AttitudeGeometry.h`** (vor `#endif`, nach dem Zero-Adjust-Block)

```cpp
// --- degrees to steps (signed) ---------------------------------------------

// Rounded to the nearest step, symmetric around zero, NOT normalised: pitch
// is a signed quantity and the caller decides whether it wants a position
// (normalise) or a travel (keep the sign).
inline int32_t signedDegreesToSteps(double degrees, uint32_t totalSteps)
{
    if (totalSteps == 0)
    {
        return 0;
    }
    const double steps = degrees * (double)totalSteps / 360.0;
    return (int32_t)(steps >= 0.0 ? steps + 0.5 : steps - 0.5);
}

// --- pitch window and roll sense -------------------------------------------

// The pitch display has mechanical stops; X-Plane's vacuum datarefs happily
// report +-90 while a failed gyro tumbles. Clamp to the display window FIRST,
// then apply the gearing between display degrees and motor degrees
// (GyroDrive's kAdjustmentFactor).
inline double pitchDisplayDegree(double pitchDeg, double maxDegree, double scale)
{
    if (pitchDeg > maxDegree)
    {
        pitchDeg = maxDegree;
    }
    else if (pitchDeg < -maxDegree)
    {
        pitchDeg = -maxDegree;
    }
    return pitchDeg * scale;
}

// The AirManager LUA sent -roll; whether that is needed depends on which way
// the horizon disc is mounted, so it is a configuration flag.
inline double rollDisplayDegree(double rollDeg, bool inversed)
{
    return inversed ? -rollDeg : rollDeg;
}

// --- nested axes ------------------------------------------------------------

// The pitch motor rides on the roll frame: turning roll by R steps turns the
// pitch DISPLAY by R steps as well. In steps, display = motor + roll, so the
// motor position that shows a given display pitch at a given roll is their
// difference. This is GyroDrive's correctedPositionPitch, expressed as an
// absolute target instead of a delta correction. Both axes then take the
// shortest path to their target — safe, because the relative pitch never
// exceeds the display window, so neither motor's travel can cross a stop.
inline uint32_t pitchMotorTarget(int32_t pitchDisplaySteps, uint32_t rollTargetSteps,
                                 uint32_t totalSteps)
{
    return normalizeStepPosition(pitchDisplaySteps - (int32_t)rollTargetSteps, totalSteps);
}

// Inverse of the above for status and calibration: what the display currently
// shows, folded onto -total/2..total/2 so a small nose-down reads negative.
inline int32_t pitchDisplaySteps(int32_t pitchMotorPos, int32_t rollPos, uint32_t totalSteps)
{
    if (totalSteps == 0)
    {
        return 0;
    }
    const int32_t total = (int32_t)totalSteps;
    int32_t display = (int32_t)normalizeStepPosition(pitchMotorPos + rollPos, totalSteps);
    if (display > total / 2)
    {
        display -= total;
    }
    return display;
}

// --- coordinated delays -----------------------------------------------------

// The axis with more steps runs at its own max rate; the other gets its step
// delay stretched by the ratio so both arrive at the same time. An idle axis
// keeps its max delay — there is nothing to coordinate and nothing to divide by.
inline void coordinatedDelays(int32_t stepsRoll, int32_t stepsPitch,
                              uint32_t delayRollMax, uint32_t delayPitchMax,
                              uint32_t *delayRoll, uint32_t *delayPitch)
{
    const uint32_t r = (uint32_t)(stepsRoll < 0 ? -stepsRoll : stepsRoll);
    const uint32_t p = (uint32_t)(stepsPitch < 0 ? -stepsPitch : stepsPitch);
    *delayRoll = delayRollMax;
    *delayPitch = delayPitchMax;
    if (r == 0 || p == 0)
    {
        return;
    }
    if (r > p)
    {
        *delayPitch = delayRollMax * r / p;
    }
    else if (p > r)
    {
        *delayRoll = delayPitchMax * p / r;
    }
}
```

- [ ] **Step 4: Tests laufen lassen**

```bash
cd AttitudeCAN && pio test -e native
```
Expected: 25 Tests PASS.

- [ ] **Step 5: Commit**

```bash
git add AttitudeCAN/include/AttitudeGeometry.h AttitudeCAN/test/test_geometry/test_geometry.cpp
git commit -m "feat(attitude-can): nested-axis geometry, pitch window and paced delays

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 4: Configuration, DebugLog, main, `AttitudeIndicator` ohne Homing

**Files:**
- Create: `AttitudeCAN/src/Configuration.h`
- Create: `AttitudeCAN/src/DebugLog.h`
- Create: `AttitudeCAN/src/main.cpp`
- Create: `AttitudeCAN/src/AttitudeIndicator.h`, `AttitudeCAN/src/AttitudeIndicator.cpp`
- Create: `AttitudeCAN/src/CAN.h` (nur Stub, damit `main.cpp` mit `BENCHDEBUG 0` kompiliert; Task 6 füllt ihn)

**Interfaces:**
- Consumes: Geometrie aus Task 2/3, `CheapStepper` (Konstruktor `(in1,in2,in3,in4,inversed)`, `setTotalSteps`, `setRpm(double) -> uint32_t delay`, `setDelay`, `newMove(bool cw, uint32_t steps, uint32_t micros)`, `run(micros)`, `stop()`, `off()`, `getPosition() -> int32_t`, `getStepsLeft() -> int32_t`, `resetPosition(uint32_t = 0)`, `getTotalSteps()`).
- Produces: Klasse `AttitudeIndicator` mit der in Step 3 gezeigten Public-API; Task 5 ergänzt das Homing in derselben Klasse.

- [ ] **Step 1: `Configuration.h`**

```cpp
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <CanNodeId.h>

#define BENCHDEBUG 0

const CanNodeId kNodeId = CanNodeId::attitudeNodeId;

// CAN (MCP2515 on hardware SPI: D11 MOSI, D12 MISO, D13 SCK).
// /CS sits on D10 deliberately. D10 is the AVR /SS pin and must be an output in
// master mode; making it the chip select means SPI.begin() drives it for us.
// Left unused it would be a floating input, and a low level there drops the SPI
// out of master mode.
//
// /INT must be D2 or D3: those are the ATmega328's only external-interrupt pins
// (INT0/INT1), and BaseCAN hangs its RX handler off attachInterrupt(). The old
// StepperBoard firmware had both Hall sensors on interrupt pins; here they are
// polled instead (the homing state machine reads them between steps), which
// frees D2 for CAN.
const uint8_t kCanIntPin = 2;
const uint8_t kCanCSPin = 10;

// Panel light. D5 is Timer0-backed PWM (OC0B); analogWrite there does not touch
// the Timer0 prescaler, so millis()/micros() stay correct.
const uint8_t kLightPin = 5;

enum AttitudeAxis {
    roll = 0,
    pitch,
    attitudeAxisCount
};

enum RpmKeys {
    maxRpm = 0,
    minRpm,
    rpmKeyCount
};

// ULN2003 IN1..IN4 per axis. Plain digital outputs; A0..A3 are full GPIO on the
// Nano. A4/A5 stay free for I2C should it ever be needed.
const uint8_t kStepperPins[attitudeAxisCount][4] = {
    {4, 7, 8, 9},       // roll
    {A0, A1, A2, A3}    // pitch
};

// Hall sensors, INPUT_PULLUP, active LOW, polled.
const uint8_t kHallPins[attitudeAxisCount] = {
    3,  // roll
    6   // pitch
};

// 4096 mini-steps per turn on both axes. The nested-axis maths in
// AttitudeGeometry.h assumes both motors share this count (display = motor +
// roll in the same step units), which is why it is one constant, not two.
const uint32_t kTotalSteps = 4096L;

// Whether a motor turns with or against its commanded direction depends on the
// wiring of the coil pins. Flip on the bench if an axis runs backwards.
const bool kStepperInversed[attitudeAxisCount] = { false, false };

// The AirManager LUA sent -roll. Kept as the default so the instrument behaves
// like before; flip if the horizon banks the wrong way on the rig.
const bool kRollInversed = true;

// Display pitch window (LUA capped at +-30) and the gearing between display
// degrees and pitch-motor degrees (GyroDrive's kAdjustmentFactor).
const double kPitchMaxDegree = 30.0;
const double kPitchScale = 0.76923077;

// 4 rpm sits below CheapStepper's "may overheat" mark; GyroDrive used it only
// for the brief fine-search homing phases and so do we.
const double kRpmLimits[attitudeAxisCount][rpmKeyCount] = {
    {14.0, 4.0},  // roll
    {14.0, 4.0}   // pitch
};

// Factory defaults for the zero offsets (GyroDrive's kRollZeroAdjustDegree /
// kPitchZeroAdjustDegree). Live values are in EEPROM, see AttitudeIndicator.
const int16_t kDefaultZeroAdjustDegree[attitudeAxisCount] = { 1, -1 };

// Homing travel budgets in degrees, copied from GyroDrive::homeRollAxis /
// homePitchAxis. A search phase that uses up its budget without the expected
// Hall change has hit a stop or a dead sensor and fails homing.
const int16_t kRollHomingLeaveDegree = 370;       // more than a turn: sensor passed from anywhere
const int16_t kRollHomingSearchDegree = 370;
const int16_t kRollHomingSlowEndDegree = 30;
const int16_t kRollHomingSlowReturnDegree = -35;
const int16_t kRollHomingSlowStartDegree = -35;
const int16_t kPitchHomingLeaveDegree = -30;
const int16_t kPitchHomingSearchDegree = 62;
const int16_t kPitchHomingBackOffDegree = -10;
const int16_t kPitchHomingSlowSearchDegree = 15;
const int16_t kPitchHomingParkDegree = -30;       // sensor edge back to level

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

#endif // CONFIGURATION_H
```

- [ ] **Step 2: `DebugLog.h`** (identisch zum Compass)

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

- [ ] **Step 3: `AttitudeIndicator.h`**

```cpp
#ifndef ATTITUDEINDICATOR_H
#define ATTITUDEINDICATOR_H
#include <Arduino.h>
#include <CheapStepper.h>
#include "Configuration.h"
#include "AttitudeGeometry.h"

class AttitudeIndicator
{
public:
    enum AttitudeResult {
        success = 0,
        notHomed,
        homingTimeout,
        axisStateReached
    };

    // Roll first, then pitch — a mechanical requirement, see beginHoming().
    enum HomingPhase {
        unknown = 0,
        rollLeaveZero,
        rollSearchZero,
        rollSearchZeroEnd,
        rollReturnToZeroEnd,
        rollSearchZeroStart,
        rollMoveToTrueZero,
        rollMoveToAdjustedZero,
        pitchLeaveZero,
        pitchSearchZero,
        pitchBackOff,
        pitchSlowSearchZero,
        pitchMoveToAdjustedZero,
        homed,
        timeout
    };

    AttitudeIndicator();

    // True once both axes found their Hall zero. Before that a setpoint means
    // nothing and would drive the display into a stop.
    bool isHomed = false;

    void loop();

    // Non-blocking: loop() drives the run forward so the CAN heartbeat keeps
    // flowing while the axes search (a run takes several seconds, well past
    // the 1500ms both ends use to declare each other dead).
    void beginHoming();
    bool isHoming() const { return homingActive; }

    // True from a failed homing run until the next beginHoming(). While set,
    // the panel light blinks instead of following the commanded brightness.
    bool hasHomingFailed() const { return homingFailed; }

    // Display-frame setpoint, degrees. Pitch is clamped to the mechanical
    // window, roll is a full circle.
    AttitudeResult moveToAttitude(double pitchDeg, double rollDeg);

    // Bench: move one axis in the display frame, the other keeps its last
    // commanded value (a roll move still counter-moves the pitch motor).
    AttitudeResult moveAxisToDegree(AttitudeAxis axis, double deg);

    void setBrightness(uint8_t brightness);

    // Records the axis' current display position as its true zero and persists
    // it (accumulating). Must be homed first.
    bool calibrateZero(AttitudeAxis axis);

    // Factory reset of both offsets. Forces a re-home.
    void wipeCalibration();

    int16_t zeroAdjustDegree(AttitudeAxis axis) const { return config.zeroAdjustDegree[axis]; }

    void stop();
    void off();

    int32_t position(AttitudeAxis axis) const { return axes[axis]->getPosition(); }
    bool hallActive(AttitudeAxis axis) const { return digitalRead(kHallPins[axis]) == LOW; }

    // What the display currently shows, in display degrees (pitch unscaled).
    double displayPitchDegree() const;
    double displayRollDegree() const;

private:
    struct Config
    {
        uint32_t magic;
        uint16_t version;
        int16_t zeroAdjustDegree[attitudeAxisCount];
    };

    void loadConfig();
    void saveConfig();
    void applyConfigDefaults();

    void applyLight(uint8_t brightness);
    void updateFailBlink();

    // Both motors, coordinated so they arrive together.
    void moveMotors(int32_t stepsRoll, int32_t stepsPitch);
    void moveToTargets(uint32_t rollTarget, uint32_t pitchMotorTargetPos);

    // Homing moves. Roll always drags the pitch motor the other way by the same
    // amount so the pitch display stays where it is.
    void homingMoveRollSteps(int32_t steps, RpmKeys rate);
    void homingMoveRoll(int16_t degree, RpmKeys rate);
    void homingMovePitch(int16_t degree, RpmKeys rate);
    void stopAll();
    void resetAll();

    void fetchHallStates();
    AttitudeResult lookForZeroChange(AttitudeAxis axis, bool targetState);
    AttitudeResult nextHomingState();
    void runHomingStep();
    void logHomingPhase();

    Config config;
    CheapStepper *axes[attitudeAxisCount];
    uint32_t stepDelay[attitudeAxisCount][rpmKeyCount];

    double commandedPitchDeg = 0.0;
    double commandedRollDeg = 0.0;

    HomingPhase homingState = unknown;
    bool homingActive = false;
    bool homingFailed = false;
    bool hallState[attitudeAxisCount] = { false, false };

    uint8_t commandedBrightness = 0;
    bool failBlinkOn = false;
    uint32_t failBlinkLastToggleMs = 0;
};

#endif // ATTITUDEINDICATOR_H
```

- [ ] **Step 4: `AttitudeIndicator.cpp` — alles außer Homing** (die Homing-Methoden bekommen vorerst Stubs, Task 5 ersetzt sie)

```cpp
#include "AttitudeIndicator.h"
#include <EEPROM.h>
#include "DebugLog.h"

static const uint32_t kAttitudeConfigMagic = 0x41544331; // 'A','T','C','1'
static const uint16_t kAttitudeConfigVersion = 1;
static const uint16_t kAttitudeEepromAddress = 0;

// Failed homing is signalled on the panel light: 1s on, 1s off, forever.
static const uint32_t kFailBlinkIntervalMs = 1000;
// The gateway may legitimately dim the panel to zero, which would hide the
// fault blink. Blink at least this bright; a brighter panel setting still wins.
static const uint8_t kFailBlinkMinBrightness = 64;

// --- config -----------------------------------------------------------------

void AttitudeIndicator::applyConfigDefaults()
{
    config.magic = kAttitudeConfigMagic;
    config.version = kAttitudeConfigVersion;
    for (uint8_t i = 0; i < attitudeAxisCount; i++)
    {
        config.zeroAdjustDegree[i] = kDefaultZeroAdjustDegree[i];
    }
}

void AttitudeIndicator::loadConfig()
{
    EEPROM.get(kAttitudeEepromAddress, config);
    if (config.magic != kAttitudeConfigMagic || config.version != kAttitudeConfigVersion)
    {
        DEBUGLOG_PRINTLN(F("ATT: no valid EEPROM config, writing defaults"));
        applyConfigDefaults();
        EEPROM.put(kAttitudeEepromAddress, config);
    }
    else
    {
        DEBUGLOG_PRINTLN(F("ATT: EEPROM config loaded"));
    }
}

void AttitudeIndicator::saveConfig()
{
    EEPROM.put(kAttitudeEepromAddress, config);
    DEBUGLOG_PRINTLN(F("ATT: config saved"));
}

void AttitudeIndicator::wipeCalibration()
{
    applyConfigDefaults();
    saveConfig();
    // Homing folded the OLD offsets into both origins. Wiping them does not
    // move anything, so every setpoint would now be off by exactly the wiped
    // amount with isHomed still true. Force a re-home.
    isHomed = false;
}

// --- construction / loop ----------------------------------------------------

AttitudeIndicator::AttitudeIndicator()
{
    loadConfig();

    pinMode(kLightPin, OUTPUT);
    applyLight(0);

    for (uint8_t i = 0; i < attitudeAxisCount; i++)
    {
        pinMode(kHallPins[i], INPUT_PULLUP);
        axes[i] = new CheapStepper(kStepperPins[i][0], kStepperPins[i][1],
                                   kStepperPins[i][2], kStepperPins[i][3],
                                   kStepperInversed[i]);
        axes[i]->setTotalSteps(kTotalSteps);
        // setRpm returns the step delay it computed; remember both rates so a
        // move can pick them without recomputing.
        stepDelay[i][minRpm] = axes[i]->setRpm(kRpmLimits[i][minRpm]);
        stepDelay[i][maxRpm] = axes[i]->setRpm(kRpmLimits[i][maxRpm]);
        axes[i]->resetPosition();
    }
    fetchHallStates();
}

void AttitudeIndicator::loop()
{
    if (homingActive)
    {
        runHomingStep();
    }
    updateFailBlink();
    const uint32_t now = micros();
    axes[roll]->run(now);
    axes[pitch]->run(now);
}

// --- light ------------------------------------------------------------------

void AttitudeIndicator::applyLight(uint8_t brightness)
{
    analogWrite(kLightPin, brightness);
}

void AttitudeIndicator::setBrightness(uint8_t brightness)
{
    // Always remember the commanded level, even while the fault blink owns the
    // light: it is what the blink's on-phase uses, and what the panel returns
    // to once homing succeeds.
    commandedBrightness = brightness;
    if (!homingFailed)
    {
        applyLight(brightness);
    }
}

void AttitudeIndicator::updateFailBlink()
{
    if (!homingFailed)
    {
        return;
    }
    const uint32_t now = millis();
    if (now - failBlinkLastToggleMs < kFailBlinkIntervalMs)
    {
        return;
    }
    failBlinkLastToggleMs = now;
    failBlinkOn = !failBlinkOn;
    applyLight(failBlinkOn ? max(commandedBrightness, kFailBlinkMinBrightness) : 0);
}

// --- motion -----------------------------------------------------------------

void AttitudeIndicator::stop()
{
    stopAll();
}

void AttitudeIndicator::off()
{
    axes[roll]->off();
    axes[pitch]->off();
}

void AttitudeIndicator::stopAll()
{
    axes[roll]->stop();
    axes[pitch]->stop();
}

void AttitudeIndicator::resetAll()
{
    axes[roll]->resetPosition();
    axes[pitch]->resetPosition();
}

void AttitudeIndicator::moveMotors(int32_t stepsRoll, int32_t stepsPitch)
{
    uint32_t delayRoll = 0;
    uint32_t delayPitch = 0;
    coordinatedDelays(stepsRoll, stepsPitch,
                      stepDelay[roll][maxRpm], stepDelay[pitch][maxRpm],
                      &delayRoll, &delayPitch);
    axes[roll]->setDelay(delayRoll);
    axes[pitch]->setDelay(delayPitch);

    // Same timestamp for both so the paced delays actually line up.
    const uint32_t now = micros();
    axes[roll]->newMove(stepsRoll > 0, (uint32_t)labs(stepsRoll), now);
    axes[pitch]->newMove(stepsPitch > 0, (uint32_t)labs(stepsPitch), now);
}

void AttitudeIndicator::moveToTargets(uint32_t rollTarget, uint32_t pitchMotorTargetPos)
{
    const uint32_t rollCurrent = normalizeStepPosition(axes[roll]->getPosition(), kTotalSteps);
    const uint32_t pitchCurrent = normalizeStepPosition(axes[pitch]->getPosition(), kTotalSteps);
    // A new setpoint replaces whatever move is in flight, recomputed from the
    // current positions. At 50Hz a queue would only pile up stale attitudes.
    moveMotors(shortestPathSteps(rollCurrent, rollTarget, kTotalSteps),
               shortestPathSteps(pitchCurrent, pitchMotorTargetPos, kTotalSteps));
}

AttitudeIndicator::AttitudeResult AttitudeIndicator::moveToAttitude(double pitchDeg, double rollDeg)
{
    if (!isHomed)
    {
        return notHomed;
    }
    commandedPitchDeg = pitchDeg;
    commandedRollDeg = rollDeg;

    // The stored zero offsets are NOT added here: homing already drove each axis
    // onto its adjusted zero and reset the position there, so position 0 is
    // level / wings level. Adding them again would double-apply them.
    const uint32_t rollTarget = normalizeStepPosition(
        signedDegreesToSteps(rollDisplayDegree(rollDeg, kRollInversed), kTotalSteps), kTotalSteps);
    const int32_t pitchSteps =
        signedDegreesToSteps(pitchDisplayDegree(pitchDeg, kPitchMaxDegree, kPitchScale), kTotalSteps);
    moveToTargets(rollTarget, pitchMotorTarget(pitchSteps, rollTarget, kTotalSteps));
    return success;
}

AttitudeIndicator::AttitudeResult AttitudeIndicator::moveAxisToDegree(AttitudeAxis axis, double deg)
{
    if (axis == roll)
    {
        return moveToAttitude(commandedPitchDeg, deg);
    }
    return moveToAttitude(deg, commandedRollDeg);
}

double AttitudeIndicator::displayRollDegree() const
{
    const int32_t steps = jogDegreesFromPosition(
        normalizeStepPosition(axes[roll]->getPosition(), kTotalSteps), kTotalSteps);
    return rollDisplayDegree((double)steps, kRollInversed);
}

double AttitudeIndicator::displayPitchDegree() const
{
    const int32_t steps = pitchDisplaySteps(axes[pitch]->getPosition(),
                                            axes[roll]->getPosition(), kTotalSteps);
    return ((double)steps * 360.0 / (double)kTotalSteps) / kPitchScale;
}

// --- calibration ------------------------------------------------------------

bool AttitudeIndicator::calibrateZero(AttitudeAxis axis)
{
    if (!isHomed)
    {
        DEBUGLOG_PRINTLN(F("ATT: not homed, cannot store zero"));
        return false;
    }
    // Freeze before reading: a move still in flight would carry the axis past
    // the point we are about to declare zero.
    stopAll();

    const int32_t rollPos = axes[roll]->getPosition();
    const int32_t pitchPos = axes[pitch]->getPosition();
    int32_t jog;
    if (axis == roll)
    {
        jog = jogDegreesFromPosition(normalizeStepPosition(rollPos, kTotalSteps), kTotalSteps);
    }
    else
    {
        // The pitch offset is a DISPLAY quantity: motor + roll, not the motor alone.
        jog = jogDegreesFromPosition(normalizeStepPosition(pitchPos + rollPos, kTotalSteps), kTotalSteps);
    }
    config.zeroAdjustDegree[axis] = accumulateZeroAdjust(config.zeroAdjustDegree[axis], jog);
    saveConfig();

    // Re-base the positions on what we just declared zero, keeping display =
    // motor + roll intact: declaring roll zero shifts the pitch motor's frame by
    // the old roll, declaring pitch zero moves the pitch motor to -roll.
    if (axis == roll)
    {
        axes[roll]->resetPosition();
        axes[pitch]->resetPosition(normalizeStepPosition(pitchPos + rollPos, kTotalSteps));
    }
    else
    {
        axes[pitch]->resetPosition(normalizeStepPosition(-rollPos, kTotalSteps));
    }
    return true;
}

// --- homing (filled in by Task 5) -------------------------------------------

void AttitudeIndicator::homingMoveRollSteps(int32_t, RpmKeys) {}
void AttitudeIndicator::homingMoveRoll(int16_t, RpmKeys) {}
void AttitudeIndicator::homingMovePitch(int16_t, RpmKeys) {}
void AttitudeIndicator::fetchHallStates() {}
AttitudeIndicator::AttitudeResult AttitudeIndicator::lookForZeroChange(AttitudeAxis, bool) { return homingTimeout; }
AttitudeIndicator::AttitudeResult AttitudeIndicator::nextHomingState() { return homingTimeout; }
void AttitudeIndicator::runHomingStep() {}
void AttitudeIndicator::logHomingPhase() {}
void AttitudeIndicator::beginHoming() {}
```

- [ ] **Step 5: `main.cpp`**

```cpp
#include <Arduino.h>
#include "Configuration.h"
#include "AttitudeIndicator.h"
#include "DebugLog.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "CAN.h"
CAN* canBus;
#endif

AttitudeIndicator* attitude;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("AttitudeIndicator initializing..."));

  attitude = new AttitudeIndicator();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(attitude);
  #else
  canBus = new CAN(attitude);
  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("AttitudeIndicator started up"));
  } else {
    // InstrumentCAN::loop() early-returns forever once begin() failed, so the
    // board goes quiet rather than obviously dead. Say so on the console.
    DEBUGLOG_PRINTLN(F("AttitudeIndicator CAN startup FAILED - board is inert"));
  }
  #endif
}

void loop() {
  #if BENCHDEBUG
  benchDebug->loop();
  #else
  canBus->loop();
  #endif
  attitude->loop();
}
```

- [ ] **Step 6: `CAN.h`-Stub** (Task 6 ersetzt ihn vollständig; hier nur, damit der Build durchläuft)

```cpp
#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <InstrumentCAN.h>
#include <CanMessageId.h>
#include <CanNodeId.h>
#include "AttitudeIndicator.h"

class CAN : public InstrumentCAN {
    public:
        CAN(AttitudeIndicator* attitude)
        : InstrumentCAN(kCanCSPin, kCanIntPin, CANFirmwareInfo{static_cast<uint16_t>(kNodeId), 1, 0}),
          attitude(attitude) {}

    protected:
        bool instrumentBegin() override { return true; }
        void handleFrame(CanMessageId, uint8_t, uint8_t, const uint8_t*) override {}

    private:
        AttitudeIndicator* attitude;
};

#endif
```

- [ ] **Step 7: Build**

```bash
cd AttitudeCAN && pio run -e nano && pio test -e native
```
Expected: Build grün (RAM/Flash-Zeilen beachten; Nano hat 2 KB RAM — bei >70 % RAM hier anhalten und melden), Tests PASS.

- [ ] **Step 8: Commit**

```bash
git add AttitudeCAN/src/Configuration.h AttitudeCAN/src/DebugLog.h AttitudeCAN/src/main.cpp AttitudeCAN/src/AttitudeIndicator.h AttitudeCAN/src/AttitudeIndicator.cpp AttitudeCAN/src/CAN.h
git commit -m "feat(attitude-can): indicator driver with nested-axis setpoints and EEPROM zero

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 5: Homing-State-Machine (Roll → Pitch)

**Files:**
- Modify: `AttitudeCAN/src/AttitudeIndicator.cpp` (Stubs am Ende ersetzen)

**Interfaces:**
- Consumes: `homingWindowCentre`, `normalizeStepPosition`, `signedDegreesToSteps`; Konstanten `k*Homing*Degree`, `kRpmLimits`.
- Produces: funktionierendes `beginHoming()` / `isHoming()` / `hasHomingFailed()` / `isHomed`.

- [ ] **Step 1: Stub-Block ersetzen**

Den gesamten Block ab `// --- homing (filled in by Task 5)` durch Folgendes ersetzen:

```cpp
// --- homing -----------------------------------------------------------------
//
// ORDER IS A MECHANICAL REQUIREMENT. Roll is homed first, and every roll move
// drags the pitch motor the other way by the same amount, so the pitch display
// does not move while roll searches its sensor. The pitch display has stops;
// searching roll with the pitch motor standing still would turn the display
// along with the frame and drive it into one. Pitch is homed only once roll
// stands at 0, at which point pitch motor == pitch display and the pitch
// phases need no correction. This is GyroDrive::homeAllAxis() made non-blocking.

// A window of a step or two means two consecutive phases were satisfied with
// essentially no travel between them (a chattering edge); its midpoint carries
// no information. The upper bound lives in homingWindowCentre().
static const int32_t kMinPlausibleWindowSteps = 4; // ~0.35 degrees

void AttitudeIndicator::homingMoveRollSteps(int32_t steps, RpmKeys rate)
{
    // Same delay on both motors: they must stay in lockstep so a stop() in the
    // middle of a phase leaves the pitch display exactly where it was.
    axes[roll]->setDelay(stepDelay[roll][rate]);
    axes[pitch]->setDelay(stepDelay[roll][rate]);
    const uint32_t now = micros();
    axes[roll]->newMove(steps > 0, (uint32_t)labs(steps), now);
    axes[pitch]->newMove(steps < 0, (uint32_t)labs(steps), now);
}

void AttitudeIndicator::homingMoveRoll(int16_t degree, RpmKeys rate)
{
    homingMoveRollSteps(signedDegreesToSteps((double)degree, kTotalSteps), rate);
}

void AttitudeIndicator::homingMovePitch(int16_t degree, RpmKeys rate)
{
    const int32_t steps = signedDegreesToSteps((double)degree, kTotalSteps);
    axes[pitch]->setDelay(stepDelay[pitch][rate]);
    axes[pitch]->newMove(steps > 0, (uint32_t)labs(steps), micros());
}

void AttitudeIndicator::fetchHallStates()
{
    for (uint8_t i = 0; i < attitudeAxisCount; i++)
    {
        const bool state = (digitalRead(kHallPins[i]) == LOW);
        // Log edges only: a print per poll would flood the line and stall the
        // steppers; a print per edge is a handful of lines per run.
        if (state != hallState[i])
        {
            DEBUGLOG_PRINT(i == roll ? F("ATT: hall roll ") : F("ATT: hall pitch "));
            DEBUGLOG_PRINT(state ? F("ON") : F("OFF"));
            DEBUGLOG_PRINT(F(" @ "));
            DEBUGLOG_PRINTLN(axes[i]->getPosition());
        }
        hallState[i] = state;
    }
}

AttitudeIndicator::AttitudeResult AttitudeIndicator::lookForZeroChange(AttitudeAxis axis, bool targetState)
{
    fetchHallStates();
    if (hallState[axis] != targetState && axes[axis]->getStepsLeft() != 0)
    {
        return success; // still searching
    }
    if (hallState[axis] != targetState)
    {
        // Ran out of commanded travel: a stop, or a sensor that never changes.
        DEBUGLOG_PRINTLN(F("ATT: homing timeout"));
        return homingTimeout;
    }
    return axisStateReached;
}

AttitudeIndicator::AttitudeResult AttitudeIndicator::nextHomingState()
{
    AttitudeResult state;
    switch (homingState)
    {
    // ---------------- roll ----------------
    case unknown:
        fetchHallStates();
        stopAll();
        resetAll();
        homingMoveRoll(kRollHomingLeaveDegree, maxRpm);
        homingState = rollLeaveZero;
        return success;

    case rollLeaveZero:
        state = lookForZeroChange(roll, false);
        if (state == axisStateReached)
        {
            stopAll();
            resetAll();
            homingMoveRoll(kRollHomingSearchDegree, maxRpm);
            homingState = rollSearchZero;
            return success;
        }
        return state;

    case rollSearchZero:
        state = lookForZeroChange(roll, true);
        if (state == axisStateReached)
        {
            stopAll();
            resetAll();
            homingMoveRoll(kRollHomingSlowEndDegree, minRpm);
            homingState = rollSearchZeroEnd;
            return success;
        }
        return state;

    case rollSearchZeroEnd:
        state = lookForZeroChange(roll, false);
        if (state == axisStateReached)
        {
            stopAll();
            resetAll();
            homingMoveRoll(kRollHomingSlowReturnDegree, minRpm);
            homingState = rollReturnToZeroEnd;
            return success;
        }
        return state;

    case rollReturnToZeroEnd:
        state = lookForZeroChange(roll, true);
        if (state == axisStateReached)
        {
            // Position is reset ON the window's end edge here, so the start
            // edge is captured in the next phase as a negative offset from it.
            stopAll();
            resetAll();
            homingMoveRoll(kRollHomingSlowStartDegree, minRpm);
            homingState = rollSearchZeroStart;
            return success;
        }
        return state;

    case rollSearchZeroStart:
        state = lookForZeroChange(roll, false);
        if (state == axisStateReached)
        {
            const uint32_t zeroStart = normalizeStepPosition(axes[roll]->getPosition(), kTotalSteps);
            stopAll();
            resetAll();
            // End edge sits at 0 in this frame (see above), so the window is the
            // modular distance from zeroStart forward to 0, and its centre is
            // half of that. Bounded on both sides: see homingWindowCentre() and
            // kMinPlausibleWindowSteps.
            int32_t centre = 0;
            const bool plausible = homingWindowCentre(0, zeroStart, kTotalSteps, &centre)
                                   && centre * 2 >= kMinPlausibleWindowSteps;
            DEBUGLOG_PRINT(F("ATT: roll window start "));
            DEBUGLOG_PRINT(zeroStart);
            DEBUGLOG_PRINT(F(" centre "));
            DEBUGLOG_PRINTLN(centre);
            if (!plausible)
            {
                DEBUGLOG_PRINTLN(F("ATT: implausible roll zero window, homing failed"));
                return homingTimeout;
            }
            homingMoveRollSteps(centre, maxRpm);
            homingState = rollMoveToTrueZero;
            return success;
        }
        return state;

    case rollMoveToTrueZero:
        if (axes[roll]->getStepsLeft() != 0)
        {
            return success;
        }
        resetAll();
        homingMoveRoll(config.zeroAdjustDegree[roll], maxRpm);
        homingState = rollMoveToAdjustedZero;
        return success;

    case rollMoveToAdjustedZero:
        if (axes[roll]->getStepsLeft() != 0)
        {
            return success;
        }
        // Roll now stands at 0. From here on pitch motor == pitch display.
        resetAll();
        homingMovePitch(kPitchHomingLeaveDegree, maxRpm);
        homingState = pitchLeaveZero;
        return success;

    // ---------------- pitch ----------------
    case pitchLeaveZero:
        state = lookForZeroChange(pitch, false);
        if (state == axisStateReached)
        {
            axes[pitch]->stop();
            axes[pitch]->resetPosition();
            homingMovePitch(kPitchHomingSearchDegree, maxRpm);
            homingState = pitchSearchZero;
            return success;
        }
        return state;

    case pitchSearchZero:
        state = lookForZeroChange(pitch, true);
        if (state == axisStateReached)
        {
            axes[pitch]->stop();
            axes[pitch]->resetPosition();
            homingMovePitch(kPitchHomingBackOffDegree, maxRpm);
            homingState = pitchBackOff;
            return success;
        }
        return state;

    case pitchBackOff:
        if (axes[pitch]->getStepsLeft() != 0)
        {
            return success;
        }
        axes[pitch]->resetPosition();
        homingMovePitch(kPitchHomingSlowSearchDegree, minRpm);
        homingState = pitchSlowSearchZero;
        return success;

    case pitchSlowSearchZero:
        state = lookForZeroChange(pitch, true);
        if (state == axisStateReached)
        {
            axes[pitch]->stop();
            axes[pitch]->resetPosition();
            homingMovePitch(kPitchHomingParkDegree + config.zeroAdjustDegree[pitch], maxRpm);
            homingState = pitchMoveToAdjustedZero;
            return success;
        }
        return state;

    case pitchMoveToAdjustedZero:
        if (axes[pitch]->getStepsLeft() != 0)
        {
            return success;
        }
        resetAll();
        homingState = homed;
        return success;

    case homed:
        return success;

    case timeout:
        return homingTimeout;
    }
    return homingTimeout;
}

void AttitudeIndicator::beginHoming()
{
    if (homingActive)
    {
        return;
    }
    stopAll();
    isHomed = false;
    // A retry clears the fault blink and hands the light back to the panel
    // brightness; a second failure switches it on again.
    homingFailed = false;
    failBlinkOn = false;
    applyLight(commandedBrightness);
    commandedPitchDeg = 0.0;
    commandedRollDeg = 0.0;
    homingState = unknown;
    DEBUGLOG_PRINTLN(F("ATT: homing started"));
    nextHomingState();
    logHomingPhase();
    homingActive = true;
}

void AttitudeIndicator::runHomingStep()
{
    // Phase transitions are the interesting events; log on change, not per loop.
    const HomingPhase phaseBefore = homingState;
    const AttitudeResult result = nextHomingState();
    if (homingState != phaseBefore)
    {
        logHomingPhase();
    }
    if (result != success)
    {
        // A timeout leaves homingState on the phase that ran out of travel; log
        // it before it is overwritten - that name is the whole diagnosis.
        logHomingPhase();
        DEBUGLOG_PRINTLN(F("ATT: homing failed"));
        stopAll();
        homingState = timeout;
        homingActive = false;
        homingFailed = true;
        failBlinkOn = true;
        failBlinkLastToggleMs = millis();
        applyLight(max(commandedBrightness, kFailBlinkMinBrightness));
        return;
    }
    if (homingState == homed)
    {
        homingActive = false;
        isHomed = true;
        DEBUGLOG_PRINTLN(F("ATT: homed"));
    }
}

void AttitudeIndicator::logHomingPhase()
{
#if DEBUGLOG_ENABLE
    static const char *const kPhaseNames[] = {
        "unknown", "rollLeaveZero", "rollSearchZero", "rollSearchZeroEnd",
        "rollReturnToZeroEnd", "rollSearchZeroStart", "rollMoveToTrueZero",
        "rollMoveToAdjustedZero", "pitchLeaveZero", "pitchSearchZero", "pitchBackOff",
        "pitchSlowSearchZero", "pitchMoveToAdjustedZero", "homed", "timeout"};
    DEBUGLOG_PRINT(F("ATT: phase "));
    DEBUGLOG_PRINT(kPhaseNames[homingState]);
    DEBUGLOG_PRINT(F(" left r/p "));
    DEBUGLOG_PRINT(axes[roll]->getStepsLeft());
    DEBUGLOG_PRINT(F("/"));
    DEBUGLOG_PRINT(axes[pitch]->getStepsLeft());
    DEBUGLOG_PRINT(F(" hall r/p "));
    DEBUGLOG_PRINT(hallState[roll] ? F("ON") : F("OFF"));
    DEBUGLOG_PRINT(F("/"));
    DEBUGLOG_PRINTLN(hallState[pitch] ? F("ON") : F("OFF"));
#endif
}
```

- [ ] **Step 2: Build mit und ohne DebugLog**

```bash
cd AttitudeCAN && pio run -e nano
sed -i '' 's/#define DEBUGLOG_ENABLE 0/#define DEBUGLOG_ENABLE 1/' src/DebugLog.h && pio run -e nano
sed -i '' 's/#define DEBUGLOG_ENABLE 1/#define DEBUGLOG_ENABLE 0/' src/DebugLog.h
```
Expected: beide Builds grün (der zweite prüft den `#if DEBUGLOG_ENABLE`-Zweig von `logHomingPhase`). `DebugLog.h` steht danach wieder auf 0 — mit `git diff src/DebugLog.h` prüfen (leer).

- [ ] **Step 3: Review gegen GyroDrive**

`StepperBoard/src/GyroDrive.cpp` `homeRollAxis()` und `homePitchAxis()` neben die State-Machine legen und abhaken: gleiche Reihenfolge, gleiche Budgets, gleiche Raten, Pitch-Gegenbewegung bei jeder Roll-Fahrt (`homingMoveRollSteps`), Pitch-Phasen ohne Roll-Anteil. Abweichung von GyroDrive ist nur die Fenstermitte (Compass-Frame + Plausibilität statt `(end − start) % total / 2`).

- [ ] **Step 4: Commit**

```bash
git add AttitudeCAN/src/AttitudeIndicator.cpp
git commit -m "feat(attitude-can): non-blocking homing, roll first with pitch counter-move

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 6: `CAN`-Klasse

**Files:**
- Modify: `AttitudeCAN/src/CAN.h` (Stub ersetzen)
- Create: `AttitudeCAN/src/CAN.cpp`

**Interfaces:**
- Consumes: `AttitudeIndicator::moveToAttitude`, `setBrightness`, `beginHoming`, `isHomed`, `isHoming()`; `CanMessageId::attitude`, `lights`, `gatewayHeartbeat`.

- [ ] **Step 1: `CAN.h`**

```cpp
#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <InstrumentCAN.h>
#include <CanMessageId.h>
#include <CanNodeId.h>
#include "AttitudeIndicator.h"

class CAN : public InstrumentCAN {
    public:
        CAN(AttitudeIndicator* attitude);

    protected:
        bool instrumentBegin() override;
        void onStartupFail() override;
        void handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t* data) override;
        void onGatewayHeartbeatTimeout() override;
        void onGatewayHeartbeatDiscovered() override;

    private:
        AttitudeIndicator* attitude;

        // Last panel brightness from a lights frame. 0 is a legitimate dimmer
        // setting, so "known" is tracked separately from the value.
        uint8_t panelPwm = 0;
        bool panelPwmKnown = false;
};

#endif
```

- [ ] **Step 2: `CAN.cpp`**

```cpp
#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

CAN::CAN(AttitudeIndicator *attitude)
    : InstrumentCAN(kCanCSPin, kCanIntPin, CANFirmwareInfo{static_cast<uint16_t>(kNodeId), 1, 0}),
      attitude(attitude)
{
    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

void CAN::onStartupFail()
{
    DEBUGLOG_PRINTLN(F("CAN startup FAIL"));
    attitude->setBrightness(0);
}

bool CAN::instrumentBegin()
{
    // Beide RX-Buffer vergleichen alle ID-Bits
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1

    // RXB0: Attitude setpoint
    canBus->init_Filt(0, 0, CAN_STD_ID(CanMessageId::attitude));
    canBus->init_Filt(1, 0, CAN_STD_ID(CanMessageId::attitude));

    // RXB1: Lights und Gateway Heartbeat
    canBus->init_Filt(2, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(3, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(4, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(5, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));

    canBus->setMode(MCP_NORMAL);

    attitude->setBrightness(0);

    return true;
}

void CAN::handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t *data)
{
    // Deliberately no per-frame log here. The DCU re-sends the attitude on every
    // change, so a print per frame is a continuous stream; at 115200 baud
    // Serial.print blocks once the 64-byte TX buffer fills, inside the same
    // loop() that steps the motors.

    // We currently expect standard frames only (ext == 0).
    (void)ext;

    switch (id)
    {
    case CanMessageId::attitude:
    {
        if (len >= 4)
        {
            // [0..1] pitch, [2..3] roll, both int16 degrees * 100, big-endian.
            const int16_t pitch100 = static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) |
                                                          static_cast<uint16_t>(data[1]));
            const int16_t roll100 = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) |
                                                         static_cast<uint16_t>(data[3]));
            // Before homing the positions mean nothing, so a setpoint would send
            // the display somewhere arbitrary - possibly into a stop.
            if (attitude->isHomed)
            {
                attitude->moveToAttitude(static_cast<double>(pitch100) / 100.,
                                         static_cast<double>(roll100) / 100.);
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
            panelPwm = static_cast<uint8_t>(ratio * 255.);
            panelPwmKnown = true;
            attitude->setBrightness(panelPwm);
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
    attitude->setBrightness(0);
}

void CAN::onGatewayHeartbeatDiscovered()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat OK"));

    // First contact with no lights frame yet: come up lit so the instrument
    // visibly joins the bus. After that, honour whatever we were last told,
    // including zero.
    attitude->setBrightness(panelPwmKnown ? panelPwm : 255);

    // First contact with the gateway is what starts the axes: homing before
    // that would drive the instrument with no sim running.
    if (!attitude->isHomed && !attitude->isHoming())
    {
        attitude->beginHoming();
    }
}
```

- [ ] **Step 3: Build**

```bash
cd AttitudeCAN && pio run -e nano
```
Expected: grün.

- [ ] **Step 4: Commit**

```bash
git add AttitudeCAN/src/CAN.h AttitudeCAN/src/CAN.cpp
git commit -m "feat(attitude-can): receive attitude 0x101 and lights 0x203 over CAN

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 7: BenchDebug

**Files:**
- Create: `AttitudeCAN/src/BenchDebug.h`, `AttitudeCAN/src/BenchDebug.cpp`

**Interfaces:**
- Consumes: komplette `AttitudeIndicator`-Public-API.

- [ ] **Step 1: `BenchDebug.h`**

```cpp
#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include "Configuration.h"

#if BENCHDEBUG
#include <Arduino.h>
#include "AttitudeIndicator.h"

class BenchDebug {
    public:
        BenchDebug(AttitudeIndicator* attitude);

        void loop();
    private:
        void handleUserInput();
        bool handleCommand(String command);
        void printStatus();
        void hallMonitor();

        String inputBuffer;

        AttitudeIndicator* attitude;
};
#endif
#endif // BENCHDEBUG_H
```

- [ ] **Step 2: `BenchDebug.cpp`**

```cpp
#include <BenchDebug.h>
#if BENCHDEBUG

// No heartbeat LED here: the Nano's onboard LED is on D13, which is SPI SCK on
// this board. Driving it would fight the CAN link.

BenchDebug::BenchDebug(AttitudeIndicator* attitude)
: attitude(attitude)
{
    Serial.begin(115200);
    Serial.println(F("AttitudeIndicator BenchDebug"));

    inputBuffer = "";

    Serial.println(F("System running! '?' for commands"));
}

void BenchDebug::printStatus()
{
    Serial.println(String(F("homed: ")) + (attitude->isHomed ? F("yes") : F("no"))
                 + F(" homing: ") + (attitude->isHoming() ? F("yes") : F("no"))
                 + F(" homingFailed: ") + (attitude->hasHomingFailed() ? F("yes") : F("no")));
    Serial.println(String(F("motor pos roll/pitch: ")) + String(attitude->position(roll))
                 + F("/") + String(attitude->position(pitch)));
    Serial.println(String(F("display roll/pitch deg: ")) + String(attitude->displayRollDegree(), 1)
                 + F("/") + String(attitude->displayPitchDegree(), 1));
    Serial.println(String(F("hall roll/pitch: ")) + (attitude->hallActive(roll) ? F("ON") : F("OFF"))
                 + F("/") + (attitude->hallActive(pitch) ? F("ON") : F("OFF")));
    Serial.println(String(F("zeroAdjust roll/pitch: ")) + String(attitude->zeroAdjustDegree(roll))
                 + F("/") + String(attitude->zeroAdjustDegree(pitch)));
}

// Prints both Hall states on every change until a line arrives on Serial.
// Turn the axes by hand to find the sensor windows.
void BenchDebug::hallMonitor()
{
    Serial.println(F("Hall monitor - press Enter to stop"));
    bool lastRoll = attitude->hallActive(roll);
    bool lastPitch = attitude->hallActive(pitch);
    Serial.println(String(F("roll ")) + (lastRoll ? F("ON") : F("OFF"))
                 + F(" pitch ") + (lastPitch ? F("ON") : F("OFF")));
    while (true)
    {
        while (Serial.available() > 0)
        {
            const char c = Serial.read();
            if (c == '\n')
            {
                Serial.println(F("Hall monitor stopped"));
                return;
            }
        }
        const bool r = attitude->hallActive(roll);
        const bool p = attitude->hallActive(pitch);
        if (r != lastRoll || p != lastPitch)
        {
            lastRoll = r;
            lastPitch = p;
            Serial.println(String(F("roll ")) + (r ? F("ON") : F("OFF"))
                         + F(" pitch ") + (p ? F("ON") : F("OFF")));
        }
        delay(5);
    }
}

static bool reportMove(AttitudeIndicator::AttitudeResult result)
{
    if (result == AttitudeIndicator::notHomed)
    {
        Serial.println(F("Not homed - run 'ho' first"));
        return false;
    }
    return true;
}

// One command per line, unlike the DCU console - this board has few enough of
// them that a splitter would be more code than it saves.
bool BenchDebug::handleCommand(String command)
{
    if (command.startsWith("ho")) {
        attitude->beginHoming();
        Serial.println(F("Homing started"));
        return true;
    } else if (command.startsWith("ro")) {
        String rString = command.substring(2);
        rString.trim();
        if (rString.length() == 0) {
            Serial.println(F("Usage: ro<deg>, e.g. ro-15"));
            return true;
        }
        const float degrees = rString.toFloat();
        if (reportMove(attitude->moveAxisToDegree(roll, degrees))) {
            Serial.println(String(F("Roll set to ")) + degrees + F(" deg"));
        }
        return true;
    } else if (command.startsWith("pi")) {
        String rString = command.substring(2);
        rString.trim();
        if (rString.length() == 0) {
            Serial.println(F("Usage: pi<deg>, e.g. pi10 (clamped to +-30)"));
            return true;
        }
        const float degrees = rString.toFloat();
        if (reportMove(attitude->moveAxisToDegree(pitch, degrees))) {
            Serial.println(String(F("Pitch set to ")) + degrees + F(" deg"));
        }
        return true;
    } else if (command.startsWith("at")) {
        String rString = command.substring(2);
        rString.trim();
        const int comma = rString.indexOf(',');
        if (comma < 0) {
            Serial.println(F("Usage: at<pitch>,<roll>, e.g. at10,-20"));
            return true;
        }
        const float pitchDeg = rString.substring(0, comma).toFloat();
        const float rollDeg = rString.substring(comma + 1).toFloat();
        if (reportMove(attitude->moveToAttitude(pitchDeg, rollDeg))) {
            Serial.println(String(F("Attitude set to pitch ")) + pitchDeg + F(" roll ") + rollDeg);
        }
        return true;
    } else if (command.startsWith("zr")) {
        if (attitude->calibrateZero(roll)) {
            Serial.println(String(F("Roll zero stored, offset now ")) + attitude->zeroAdjustDegree(roll) + F(" deg"));
        } else {
            Serial.println(F("Not homed - cannot store zero"));
        }
        return true;
    } else if (command.startsWith("zp")) {
        if (attitude->calibrateZero(pitch)) {
            Serial.println(String(F("Pitch zero stored, offset now ")) + attitude->zeroAdjustDegree(pitch) + F(" deg"));
        } else {
            Serial.println(F("Not homed - cannot store zero"));
        }
        return true;
    } else if (command.startsWith("cw")) {
        attitude->wipeCalibration();
        Serial.println(F("Calibration wiped to defaults - re-home with 'ho'"));
        return true;
    } else if (command.startsWith("li")) {
        String rString = command.substring(2);
        rString.trim();
        long value = rString.toInt();
        uint8_t pwm = (uint8_t)constrain(value, 0L, 255L);
        attitude->setBrightness(pwm);
        Serial.println(String(F("Brightness set to ")) + pwm);
        return true;
    } else if (command.startsWith("mo")) {
        attitude->stop();
        attitude->off();
        Serial.println(F("Motors stopped and de-energised"));
        return true;
    } else if (command.startsWith("st")) {
        printStatus();
        return true;
    } else if (command.startsWith("hm")) {
        hallMonitor();
        return true;
    } else if (command.startsWith("?")) {
        Serial.println(F("AttitudeIndicator Commands:"));
        Serial.println(F("ho: start homing (roll first, then pitch)"));
        Serial.println(F("ro<deg>: roll to degrees (display frame)"));
        Serial.println(F("pi<deg>: pitch to degrees (display frame, clamped +-30)"));
        Serial.println(F("at<pitch>,<roll>: setpoint as sent by the bus"));
        Serial.println(F("zr / zp: store current roll / pitch position as zero"));
        Serial.println(F("cw: wipe calibration to defaults"));
        Serial.println(F("li<0..255>: set LED brightness"));
        Serial.println(F("mo: stop and de-energise both motors"));
        Serial.println(F("st: show status"));
        Serial.println(F("hm: hall monitor (Enter stops)"));
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

- [ ] **Step 3: Build in beiden Modi**

```bash
cd AttitudeCAN && sed -i '' 's/#define BENCHDEBUG 0/#define BENCHDEBUG 1/' src/Configuration.h && pio run -e nano
sed -i '' 's/#define BENCHDEBUG 1/#define BENCHDEBUG 0/' src/Configuration.h && pio run -e nano
git diff --stat AttitudeCAN/src/Configuration.h
```
Expected: beide grün, `git diff` leer (BENCHDEBUG wieder 0). RAM-Prozent des Bench-Builds notieren; `String`-Verkettungen im Status sind Heap — bei >75 % RAM die `printStatus`-Zeilen in einzelne `Serial.print` aufbrechen.

- [ ] **Step 4: Commit**

```bash
git add AttitudeCAN/src/BenchDebug.h AttitudeCAN/src/BenchDebug.cpp
git commit -m "feat(attitude-can): bench console with homing, axis moves and hall monitor

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 8: DCU — Serial `0x0F` → CAN `0x101`

**Files:**
- Create: `shared/CANBase/include/AttitudeWire.h`
- Create: `DCU/test/test_attitude_wire/test_attitude_wire.cpp`
- Modify: `DCU/src/DCUReceiver.h`
- Modify: `DCU/src/DCUReceiver.cpp`

**Interfaces:**
- Consumes: `MessageType::SerialMessageAttitude`, `AttitudeMessage`, `CanMessageId::attitude`, `packBE16` (`DCU/include/WireEncoding.h`, bereits von `sendCompass` genutzt), `MessageMeta`, `isStale`.
- Produces: `int16_t attitudeDeg100(float degrees)` (shared, auch für das Plugin nutzbar), `DCUReceiver::sendAttitude()`.

- [ ] **Step 1: Test schreiben**

`DCU/test/test_attitude_wire/test_attitude_wire.cpp`:

```cpp
#include <unity.h>
#include <math.h>
#include "AttitudeWire.h"

void setUp(void) {}
void tearDown(void) {}

void test_degrees_scale_to_hundredths_with_rounding(void) {
    TEST_ASSERT_EQUAL_INT16(0, attitudeDeg100(0.0f));
    TEST_ASSERT_EQUAL_INT16(1234, attitudeDeg100(12.34f));
    TEST_ASSERT_EQUAL_INT16(-1234, attitudeDeg100(-12.34f));
    // 0.005 rounds away from zero on both sides, not towards +inf.
    TEST_ASSERT_EQUAL_INT16(1, attitudeDeg100(0.005f));
    TEST_ASSERT_EQUAL_INT16(-1, attitudeDeg100(-0.005f));
}

// A tumbling vacuum gyro reports +-90 pitch and the full +-180 roll; both fit.
void test_the_full_tumble_range_fits(void) {
    TEST_ASSERT_EQUAL_INT16(9000, attitudeDeg100(90.0f));
    TEST_ASSERT_EQUAL_INT16(-18000, attitudeDeg100(-180.0f));
    TEST_ASSERT_EQUAL_INT16(18000, attitudeDeg100(180.0f));
}

// Garbage from an unloaded aircraft must not wrap the int16 into the other
// sign: 400 degrees would otherwise read as -255.36.
void test_values_past_the_int16_are_clamped(void) {
    TEST_ASSERT_EQUAL_INT16(32767, attitudeDeg100(400.0f));
    TEST_ASSERT_EQUAL_INT16(-32768, attitudeDeg100(-400.0f));
    TEST_ASSERT_EQUAL_INT16(32767, attitudeDeg100(1e9f));
}

// lroundf(NaN) is undefined; a NaN dataref read has to come out as level.
void test_nan_becomes_zero(void) {
    TEST_ASSERT_EQUAL_INT16(0, attitudeDeg100(NAN));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_degrees_scale_to_hundredths_with_rounding);
    RUN_TEST(test_the_full_tumble_range_fits);
    RUN_TEST(test_values_past_the_int16_are_clamped);
    RUN_TEST(test_nan_becomes_zero);
    return UNITY_END();
}
```

- [ ] **Step 2: Test laufen lassen, muss scheitern**

```bash
cd DCU && pio test -e native -f test_attitude_wire
```
Expected: Compile-Fehler `AttitudeWire.h: No such file`.

- [ ] **Step 3: `AttitudeWire.h`**

```cpp
#ifndef ATTITUDEWIRE_H
#define ATTITUDEWIRE_H

#include <stdint.h>
#include <math.h>

// Wire helper for CAN 0x101, shared by the DCU and DCUProviderPlugin - both
// resolve this directory already. Deliberately Arduino-free: the native Unity
// tests in DCU/test/test_attitude_wire exercise it (pio test -e native).

// Degrees to the int16 hundredths the frame carries. Pitch and roll are both
// legitimately negative, so this rounds half away from zero instead of the
// "+0.5" trick the unsigned fields use. Values beyond the int16 are clamped
// rather than wrapped (an unloaded aircraft can hand out anything), and a NaN
// read comes out level - lroundf(NaN) is undefined behaviour.
inline int16_t attitudeDeg100(float degrees)
{
    if (degrees != degrees) // NaN
    {
        return 0;
    }
    const float scaled = degrees * 100.0f;
    if (scaled >= 32767.0f)
    {
        return 32767;
    }
    if (scaled <= -32768.0f)
    {
        return -32768;
    }
    return (int16_t)lroundf(scaled);
}

#endif // ATTITUDEWIRE_H
```

- [ ] **Step 4: Tests laufen lassen**

```bash
cd DCU && pio test -e native
```
Expected: alle Suiten PASS, inklusive 4 neuer Tests.

- [ ] **Step 5: `DCUReceiver.h`**

Nach `void sendHsiNav();`:

```cpp
        void sendAttitude();
```

Nach dem HSI-State-Block (`int16_t  hsiGsDots1000 = 0;`):

```cpp
        // Attitude indicator (CAN 0x101)
        int16_t attitudePitch100 = 0;
        int16_t attitudeRoll100 = 0;
```

Nach `MessageMeta hsiNavMeta;`:

```cpp
        MessageMeta attitudeMeta;
```

- [ ] **Step 6: `DCUReceiver.cpp`**

Include ergänzen (neben den anderen Shared-Includes am Dateianfang):

```cpp
#include <AttitudeWire.h>
```

Im Konstruktor nach `hsiNavMeta = {0, 5000};`:

```cpp
  attitudeMeta = {0, 5000};
```

In `handleFrame` nach dem `SerialMessageHsiNav`-Case (vor `default`):

```cpp
  case MessageType::SerialMessageAttitude:
  {
    if (len != sizeof(AttitudeMessage))
      return;

    AttitudeMessage msg;
    memcpy(&msg, payload, sizeof(msg));

    const int16_t pitch100 = attitudeDeg100(msg.pitchDeg);
    const int16_t roll100 = attitudeDeg100(msg.rollDeg);

    if (pitch100 != attitudePitch100 || roll100 != attitudeRoll100)
    {
      attitudePitch100 = pitch100;
      attitudeRoll100 = roll100;
      sendAttitude();
    }
    break;
  }
```

Nach `sendHsiNav()`:

```cpp
void DCUReceiver::sendAttitude()
{
  byte data[8] = {0};

  // [0..1] pitch, [2..3] roll, both int16 degrees * 100, [4..7] reserved.
  packBE16(data + 0, static_cast<uint16_t>(attitudePitch100));
  packBE16(data + 2, static_cast<uint16_t>(attitudeRoll100));

  canBus->sendMessage(CanMessageId::attitude, 8, data);

  attitudeMeta.lastSendTimestamp = millis();
}
```

In `checkMaxAgeResync` nach dem `hsiNavMeta`-Block:

```cpp
  if (isStale(attitudeMeta.lastSendTimestamp, now, attitudeMeta.maxAgeMs))
  {
    DEBUGLOG_PRINTLN(String(F("MaxAge resync for attitude")));
    sendAttitude();
  }
```

- [ ] **Step 7: Build + Tests**

```bash
cd DCU && pio run -e megaatmega2560 && pio test -e native
```
Expected: grün. RAM-Zeile des Mega-Builds notieren (zwei int16 + ein Meta, vernachlässigbar).

- [ ] **Step 8: Commit**

```bash
git add shared/CANBase/include/AttitudeWire.h DCU/test/test_attitude_wire DCU/src/DCUReceiver.h DCU/src/DCUReceiver.cpp
git commit -m "feat(dcu): forward attitude from serial 0x0F to CAN 0x101

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 9: Plugin — Datarefs + 50-Hz-Sender

**Files:**
- Modify: `DCUProviderPlugin/src/DataRefManager.h`
- Modify: `DCUProviderPlugin/src/DataRefManager.cpp`
- Modify: `DCUProviderPlugin/src/DCUProvider.h`
- Modify: `DCUProviderPlugin/src/DCUProvider.cpp`

**Interfaces:**
- Consumes: `AttitudeMessage`, `MessageType::SerialMessageAttitude`, `readFloat(XPLMDataRef, float)` (bestehend), `msgQueue_->enqueueTx(type, ptr, size)`.
- Produces: `float DataRefManager::getPitchVacuumDeg() const`, `float DataRefManager::getRollVacuumDeg() const`.

- [ ] **Step 1: `DataRefManager.h`**

Nach `float getCompassHeadingDegMag() const;`:

```cpp
    // Attitude indicator (vacuum-driven gyro: tumbles on vacuum failure)
    float getPitchVacuumDeg() const;  // pitch_vacuum_deg_pilot, degrees, nose up positive
    float getRollVacuumDeg() const;   // roll_vacuum_deg_pilot, degrees, right wing down positive
```

Nach `XPLMDataRef dr_vsi = nullptr;`:

```cpp
    XPLMDataRef dr_pitch_vacuum = nullptr;
    XPLMDataRef dr_roll_vacuum = nullptr;
```

- [ ] **Step 2: `DataRefManager.cpp`**

In `initialize()` (oder wo `dr_vsi` gesucht wird) nach der `dr_vsi`-Zeile:

```cpp
    // Attitude indicator. The _vacuum variants already model the gyro spinning
    // down on vacuum failure, so no separate failure flag goes over the wire.
    dr_pitch_vacuum = XPLMFindDataRef("sim/cockpit2/gauges/indicators/pitch_vacuum_deg_pilot");
    dr_roll_vacuum = XPLMFindDataRef("sim/cockpit2/gauges/indicators/roll_vacuum_deg_pilot");
```

Nach `getCompassHeadingDegMag()`:

```cpp
// Attitude indicator
float DataRefManager::getPitchVacuumDeg() const
{
    return readFloat(dr_pitch_vacuum, 0.0f);
}

float DataRefManager::getRollVacuumDeg() const
{
    return readFloat(dr_roll_vacuum, 0.0f);
}
```

- [ ] **Step 3: `DCUProvider.h`**

Nach `float hsiNavAccumulator_ = 0.0f;`:

```cpp
    float attitudeAccumulator_ = 0.0f;
```

Nach `static constexpr float HSI_NAV_RATE = 10.0f;     // Hz`:

```cpp
    static constexpr float ATTITUDE_RATE = 50.0f;    // Hz
```

- [ ] **Step 4: `DCUProvider.cpp`**

Nach dem `// ============ HSI deviation needles (10 Hz) ============`-Block, vor `// ============ Odometer Data (10 Hz) ============`:

```cpp
    // ============ Attitude indicator (50 Hz) ============
    attitudeAccumulator_ += dt;
    float attitudeRate = 1.0f / ATTITUDE_RATE;

    if (attitudeAccumulator_ >= attitudeRate)
    {
        AttitudeMessage attitude;
        attitude.pitchDeg = dataRefMgr_->getPitchVacuumDeg();
        attitude.rollDeg = dataRefMgr_->getRollVacuumDeg();

        msgQueue_->enqueueTx(MessageType::SerialMessageAttitude, &attitude, sizeof(attitude));

        attitudeAccumulator_ = 0.0f;
    }
```

- [ ] **Step 5: Build**

```bash
cd DCUProviderPlugin && ./build-macos.sh
```
Expected: Build grün. Falls der Sandbox-Build an cmake/clang scheitert (siehe Memory `sandbox-toolchain-setup`): den exakten Fehler melden, nicht umgehen.

- [ ] **Step 6: Commit**

```bash
git add DCUProviderPlugin/src/DataRefManager.h DCUProviderPlugin/src/DataRefManager.cpp DCUProviderPlugin/src/DCUProvider.h DCUProviderPlugin/src/DCUProvider.cpp
git commit -m "feat(plugin): send vacuum pitch and roll to the DCU at 50Hz

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 10: Dokumentation + Bench-Checkliste

**Files:**
- Modify: `CLAUDE.md`
- Modify: `DCU/CLAUDE.md`
- Create: `AttitudeCAN/README.md`

- [ ] **Step 1: Root `CLAUDE.md`**

Board-Liste (Zeile mit `AirspeedCAN`, `AltimeterCAN`, …): `AttitudeCAN` alphabetisch nach `AltimeterDriver` einfügen.

Nach dem `HSIDriver`-Fallback-Absatz ergänzen:

```markdown
  `StepperBoard` still carries the pre-CAN attitude-indicator driver (`GyroDrive`) next to its
  X25 motors; `AttitudeCAN` supersedes that part. `StepperBoard` stays as the nominal fallback
  until the CAN board is confirmed on the rig and keeps driving the X25 gauges regardless.
```

In der `pio test`-Kommentarzeile `AirspeedCAN, AltimeterCAN, DCU, HSICAN, RudderCAN,` → `AirspeedCAN, AltimeterCAN, AttitudeCAN, DCU, HSICAN, RudderCAN,`.

In der Header-Liste `(`AdaptiveFilter.h`, …, `HSICalibration.h`)` → `AttitudeGeometry.h` anhängen.

Im Architektur-Abschnitt nach dem Whiskey-Compass-Absatz:

```markdown
  `AttitudeCAN` is the nested-axis variant: CAN `0x101` carries signed pitch/roll, and the
  board's `AttitudeGeometry.h` turns a display attitude into two coupled motor targets (the
  pitch motor rides on the roll frame). Its homing order — roll first, every roll move
  counter-moved on pitch, pitch last — is a mechanical constraint, not a style choice.
  See `docs/superpowers/specs/2026-10-09-attitude-can-design.md`.
```

- [ ] **Step 2: `DCU/CLAUDE.md`**

In der Receiver-Aufzählung `SerialMessageCompass),` → `SerialMessageCompass`, `SerialMessageHsiHeading`, `SerialMessageHsiNav`, `SerialMessageAttitude`),` (falls die HSI-Einträge dort noch fehlen, mitziehen). Danach ergänzen:

```markdown
  `SerialMessageAttitude` (`0x0F`) carries `AttitudeMessage` (two floats, pitch then roll);
  `DCUReceiver` repacks it into CAN `0x101` as two big-endian `int16` of degrees × 100 via
  `attitudeDeg100()` from `shared/CANBase/include/AttitudeWire.h` (clamped, NaN-safe, tested in
  `test_attitude_wire`).
```

- [ ] **Step 3: `AttitudeCAN/README.md`**

```markdown
# AttitudeCAN

CAN instrument board for the attitude indicator: two nested 28BYJ-48 steppers (pitch motor on
the roll frame), one Hall sensor per axis, one panel LED. Design:
`docs/superpowers/specs/2026-10-09-attitude-can-design.md`.

## Build / test

    pio run -e nano            # firmware (BENCHDEBUG 0 in src/Configuration.h)
    pio test -e native         # AttitudeGeometry.h unit tests
    pio device monitor -b 115200

## Bench checklist (BENCHDEBUG 1)

1. `hm` — both Hall sensors toggle when the axes are turned by hand. Enter stops.
2. `ho` — roll finds its window, then pitch finds its sensor; no stop is touched. `st` shows
   `homed: yes`. A failure blinks the LED 1s/1s; the DebugLog (DEBUGLOG_ENABLE 1) names the
   phase that ran out of travel.
3. `ro90`, `ro-90`, `ro180` — the pitch display stays put while roll turns.
4. `pi30`, `pi-30`, `pi40` (clamped to 30) — no stop is reached.
5. `at10,-20` — both axes arrive together.
6. `li0`, `li255`.
7. `zr` / `zp` after jogging onto the optical zero; `st` shows the accumulated offsets.
   Note: an EEPROM wipe on reflash resets them — re-run after flashing a new firmware.

## Rig checklist (BENCHDEBUG 0)

1. Gateway heartbeat starts homing; LED comes up at the last panel brightness.
2. Roll right in X-Plane: horizon banks the right way, else flip `kRollInversed`.
3. Nose up: horizon moves the right way, else flip `kStepperInversed[pitch]` (and re-check
   the homing search direction on the bench).
4. From `CANDebugNode`, send `0x101` with len 2: the board must ignore it (no move, no reset).
5. Pull the gateway: LED off; reconnect: LED back to the last brightness, no re-home.
```

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md DCU/CLAUDE.md AttitudeCAN/README.md
git commit -m "docs: record AttitudeCAN in the repo guides with bench and rig checklists

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

## Abschluss

Nach Task 10: `superpowers:finishing-a-development-branch`. Build-Matrix als Abnahme:

```bash
cd AttitudeCAN && pio run -e nano && pio test -e native
cd ../DCU && pio run -e megaatmega2560 && pio test -e native
cd ../WhiskeyCompassCAN && pio run -e nano
cd ../HSICAN && pio run -e nano
cd ../DCUProviderPlugin && ./build-macos.sh
```

Offen bleibt die Hardware-Abnahme (Bench- und Rig-Checkliste im README) — das kann die Sandbox nicht. Nach Bestätigung am Rig: Fallback-Hinweis in `CLAUDE.md` zu `StepperBoard` zurücknehmen.
