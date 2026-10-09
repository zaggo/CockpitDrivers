# TurnCoordinatorCAN Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Neues CAN-Instrument `TurnCoordinatorCAN` (Wendezeiger: Turn-Rate-Nadel + Slip-Kugel, zwei X27.168 an einem AX1201728SG auf einem Nano) plus die komplette Kette Plugin → DCU → Board.

**Architecture:** Board im Stil von `VerticalSpeedCAN`: vendored, nicht-blockierender `vid6608`-Fork für beide Kanäle, Arduino-freie Kalibriertabelle in `include/TurnCoordinatorCalibration.h` (nativ getestet, eine Tabelle pro Nadel im EEPROM), `TurnCoordinator` als Nadel-/EEPROM-/LED-Klasse, `CAN` als `InstrumentCAN`-Subklasse. DCU repackt Serial `0x10` (zwei Floats) in CAN `0x103` (zwei int16 BE, Grad·100) über das vorhandene `attitudeDeg100()`; Plugin liest `turn_rate_roll_deg_pilot` und `slip_deg` mit 50 Hz.

**Tech Stack:** PlatformIO (atmelavr, nanoatmega328new), Unity (native), `coryjfowler/mcp_can`, vendored `vid6608`-Fork, X-Plane SDK (Plugin, CMake).

**Spec:** `docs/superpowers/specs/2026-10-09-turn-coordinator-can-design.md`

## Global Constraints

- Arbeit direkt auf `main`, **kein Worktree, kein Feature-Branch** — der Nutzer hat das explizit so gewollt. Andere Sessions teilen den Checkout: nie `git add -A`, immer Dateien einzeln stagen. `git` liegt unter `/opt/homebrew/bin/git`, falls es nicht im PATH ist.
- `pio` liegt in `~/.platformio/penv/bin/pio` (im PATH). Alle `pio`-Befehle aus dem jeweiligen Projektverzeichnis.
- Node-ID `turnCoordinatorNodeId = 0x0D`, CAN `turnCoordinator = 0x103`, Serial `SerialMessageTurnCoordinator = 0x10`. Payload-Reihenfolge überall **Turn Rate, dann Slip**.
- CAN-Mehrbyte-Felder big-endian; Serial-Payloads host order (packed structs, MSVC-Zweig mit `#pragma pack`).
- Pins: `/CS` D10, `/INT` D2, Turn Rate STEP D7 / DIR D8, Slip STEP D4 / DIR D6, RST D9 (gemeinsam), LED D3.
- `kSteps = 320 * 12` für beide Motoren; VSI-Accel-Tabelle (Cruise nach 32 Steps) übernehmen.
- Beide Nadeln: 0 ist ein **angelernter Punkt**, nicht der Home-Anschlag. Leere Tabelle = Nadel parkt auf Step 0.
- Keine Richtungsumkehr im Code — die Tabelle lernt die Drehrichtung mit.
- `StepperBoard` wird nicht angefasst.
- Kein Log pro CAN-Frame im Board (Serial blockiert den Stepper-Takt).
- Commit-Messages enden mit `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>`.

## Review Focus

1. **Slip-Skala asymmetrisch** (links und rechts verschieden steil): jede Hälfte muss auf ihrer eigenen Steigung interpolieren, kein Spiegeln um die Mitte. → Test `test_slip_table_interpolates_each_side_on_its_own_slope` (Task 2).
2. **Turn Rate jenseits der Skala** (X-Plane liefert beim Trudeln >110°): Nadel parkt am äußersten Punkt, kein Anschlag. → Test `test_turn_rate_past_the_outer_mark_parks_on_it` (Task 2).
3. **`0x103` kürzer als 4 Byte**: ignorieren, nicht aus `data[]` hinauslesen. → Längencheck in `CAN::handleFrame` (Task 3) + Rig-Punkt in Task 7.
4. **NaN / Riesenwerte im Dataref** (Flugzeug noch nicht geladen): `attitudeDeg100()` klemmt und macht NaN zu 0 — bereits durch `DCU/test/test_attitude_wire` abgedeckt; Task 5 verwendet genau diesen Helper, keinen eigenen.
5. **EEPROM mit gültigem Magic, aber kaputtem `count` in nur einer der beiden Tabellen**: beide Tabellen wipen, nie über das Array hinaus indizieren. → `loadConfig()` prüft beide Tabellen (Task 3); Bench-Punkt „`cw`, Neustart, `?` zeigt 0/0 Punkte“ in Task 7.

---

### Task 1: Protokoll-Header in `shared/CANBase`

**Files:**
- Modify: `shared/CANBase/include/CanNodeId.h`
- Modify: `shared/CANBase/include/CanMessageId.h`
- Modify: `shared/CANBase/include/SerialMessageId.h`

**Interfaces:**
- Produces: `CanNodeId::turnCoordinatorNodeId`, `CanMessageId::turnCoordinator`, `MessageType::SerialMessageTurnCoordinator`, `struct TurnCoordinatorMessage { float turnRateDeg; float slipDeg; }` (8 Byte).

- [ ] **Step 1: Node-ID**

In `CanNodeId.h` nach `attitudeNodeId = 0x0C` einfügen (Komma an `attitudeNodeId` ergänzen):

```cpp
  attitudeNodeId = 0x0C,
  turnCoordinatorNodeId = 0x0D
```

- [ ] **Step 2: CAN-Message-ID**

In `CanMessageId.h` direkt nach dem `altimeterVsi = 0x102,`-Block und vor dem `0x107: Magnetic compass`-Kommentar einfügen:

```cpp
  // 0x103: Turn coordinator (Gateway -> Instrument, 50Hz)
  // [0..1] turn rate int16, degrees * 100 (sim/cockpit2/gauges/indicators/turn_rate_roll_deg_pilot)
  // [2..3] side slip int16, degrees * 100 (sim/cockpit2/gauges/indicators/slip_deg)
  // [4..7] reserved
  // Both signed; both needles rest mid-dial at 0. The gateway forwards whatever
  // the sim says, the board clamps via its calibration table.
  turnCoordinator = 0x103,
```

- [ ] **Step 3: Serial-Message-Typ + Payload**

In `SerialMessageId.h` im `MessageType`-Enum nach `SerialMessageAttitude = 0x0F,` einfügen:

```cpp
    // Plugin -> DCU. Payload: TurnCoordinatorMessage (8 bytes, host order).
    // Repacked by the DCU into CAN 0x103 as two big-endian int16 of degrees * 100.
    SerialMessageTurnCoordinator = 0x10,
```

Im MSVC-Zweig (`#if defined(_MSC_VER)`, innerhalb `#pragma pack(push, 1)` … `pop`) nach `struct AttitudeMessage { ... };` einfügen:

```cpp
    struct TurnCoordinatorMessage {
        float turnRateDeg;  // turn_rate_roll_deg_pilot
        float slipDeg;      // slip_deg
    };
```

Im `#else`-Zweig nach `struct __attribute__((packed)) AttitudeMessage { ... };` einfügen:

```cpp
    struct __attribute__((packed)) TurnCoordinatorMessage {
        float turnRateDeg;
        float slipDeg;
    };
```

Nach dem `AttitudeMessage`-`static_assert` ergänzen:

```cpp
static_assert(sizeof(TurnCoordinatorMessage) == 8, "TurnCoordinatorMessage must be 8 bytes on the wire");
```

- [ ] **Step 4: Compile-Check über einen bestehenden Konsumenten**

Run: `cd DCU && pio run -e megaatmega2560`
Expected: `SUCCESS` — die Header werden von jedem Board gezogen; DCU kompiliert alle drei.

- [ ] **Step 5: Commit**

```bash
git add shared/CANBase/include/CanNodeId.h shared/CANBase/include/CanMessageId.h shared/CANBase/include/SerialMessageId.h
git commit -m "feat(canbase): turn coordinator node 0x0D, CAN 0x103, serial 0x10

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 2: Projektgerüst + Kalibriertabelle mit Tests (TDD)

**Files:**
- Create: `TurnCoordinatorCAN/platformio.ini`
- Create: `TurnCoordinatorCAN/lib/vid6608/` (Kopie aus `VerticalSpeedCAN/lib/vid6608`)
- Create: `TurnCoordinatorCAN/include/TurnCoordinatorCalibration.h`
- Create: `TurnCoordinatorCAN/test/test_calibration/test_calibration.cpp`

**Interfaces:**
- Produces: `CalibrationPoint { int16_t value; uint16_t step; }`, `CalibrationTable { uint8_t count; CalibrationPoint points[kMaxCalibrationPoints]; }`, `kMaxCalibrationPoints = 12`, `calibrationWipe(CalibrationTable&)`, `bool calibrationSet(CalibrationTable&, int16_t value, uint16_t step)`, `int16_t calibrationMinValue(const CalibrationTable&)`, `int16_t calibrationMaxValue(const CalibrationTable&)`, `uint16_t calibrationStepFor(const CalibrationTable&, float value)`.

- [ ] **Step 1: Projektverzeichnis + `platformio.ini`**

```bash
mkdir -p TurnCoordinatorCAN/include TurnCoordinatorCAN/src TurnCoordinatorCAN/test/test_calibration TurnCoordinatorCAN/lib
cp -R VerticalSpeedCAN/lib/vid6608 TurnCoordinatorCAN/lib/
```

`TurnCoordinatorCAN/platformio.ini`:

```ini
[avr]
framework = arduino
platform = atmelavr
monitor_speed = 115200
lib_deps =
    ; vid6608 is a LOCAL FORK vendored in lib/vid6608 (non-blocking stepping) -
    ; do NOT re-add petrows/vid6608 here or it will shadow the fork.
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

- [ ] **Step 2: Failing Tests schreiben**

`TurnCoordinatorCAN/test/test_calibration/test_calibration.cpp`:

```cpp
#include <unity.h>
#include "TurnCoordinatorCalibration.h"

void setUp(void) {}
void tearDown(void) {}

// Turn rate needle after a bench walkthrough: the left mark (pt-90), the
// level mark (pt0) and the right mark (pt90). The level mark sits mid-dial,
// nowhere near the mechanical home stop at step 0.
static CalibrationTable turnRateTable(void) {
    CalibrationTable table;
    calibrationWipe(table);
    calibrationSet(table, -90, 300);
    calibrationSet(table, 0, 1920);
    calibrationSet(table, 90, 3540);
    return table;
}

// Slip ball: the old AirManager LUA shows the dial is not symmetric about
// the centre (0.084 of travel on one side, 0.072 on the other), so the two
// halves get different step spans.
static CalibrationTable slipTable(void) {
    CalibrationTable table;
    calibrationWipe(table);
    calibrationSet(table, -8, 0);
    calibrationSet(table, 0, 950);
    calibrationSet(table, 8, 1764);
    return table;
}

void test_wipe_leaves_no_points_at_all(void) {
    CalibrationTable table;
    calibrationWipe(table);
    calibrationSet(table, 90, 3540); // garbage from a previous session
    calibrationWipe(table);
    TEST_ASSERT_EQUAL_UINT8(0, table.count);
}

void test_points_are_kept_sorted_regardless_of_entry_order(void) {
    CalibrationTable table;
    calibrationWipe(table);
    TEST_ASSERT_TRUE(calibrationSet(table, 90, 3540));
    TEST_ASSERT_TRUE(calibrationSet(table, -90, 300));
    TEST_ASSERT_TRUE(calibrationSet(table, 0, 1920));
    TEST_ASSERT_EQUAL_UINT8(3, table.count);
    TEST_ASSERT_EQUAL_INT16(-90, table.points[0].value);
    TEST_ASSERT_EQUAL_UINT16(300, table.points[0].step);
    TEST_ASSERT_EQUAL_INT16(0, table.points[1].value);
    TEST_ASSERT_EQUAL_UINT16(1920, table.points[1].step);
    TEST_ASSERT_EQUAL_INT16(90, table.points[2].value);
}

void test_setting_a_known_value_replaces_its_step(void) {
    CalibrationTable table = turnRateTable();
    TEST_ASSERT_TRUE(calibrationSet(table, -90, 320));
    TEST_ASSERT_EQUAL_UINT8(3, table.count);
    TEST_ASSERT_EQUAL_UINT16(320, table.points[0].step);
}

void test_a_full_table_rejects_further_points(void) {
    CalibrationTable table;
    calibrationWipe(table);
    for (int16_t i = 0; i < (int16_t)kMaxCalibrationPoints; i++) {
        TEST_ASSERT_TRUE(calibrationSet(table, (int16_t)(i * 20 - 120), (uint16_t)(i * 300)));
    }
    TEST_ASSERT_EQUAL_UINT8(kMaxCalibrationPoints, table.count);
    TEST_ASSERT_FALSE(calibrationSet(table, 300, 3800));
    TEST_ASSERT_EQUAL_UINT8(kMaxCalibrationPoints, table.count);
}

void test_a_full_table_still_accepts_a_replacement(void) {
    CalibrationTable table;
    calibrationWipe(table);
    for (int16_t i = 0; i < (int16_t)kMaxCalibrationPoints; i++) {
        calibrationSet(table, (int16_t)(i * 20 - 120), (uint16_t)(i * 300));
    }
    TEST_ASSERT_TRUE(calibrationSet(table, -120, 11));
    TEST_ASSERT_EQUAL_UINT16(11, table.points[0].step);
}

void test_min_and_max_report_the_outer_points(void) {
    CalibrationTable table = turnRateTable();
    TEST_ASSERT_EQUAL_INT16(-90, calibrationMinValue(table));
    TEST_ASSERT_EQUAL_INT16(90, calibrationMaxValue(table));
}

void test_min_and_max_of_an_empty_table_are_zero(void) {
    CalibrationTable table;
    calibrationWipe(table);
    TEST_ASSERT_EQUAL_INT16(0, calibrationMinValue(table));
    TEST_ASSERT_EQUAL_INT16(0, calibrationMaxValue(table));
}

void test_calibrated_points_map_to_their_own_step(void) {
    CalibrationTable table = turnRateTable();
    TEST_ASSERT_EQUAL_UINT16(300, calibrationStepFor(table, -90.));
    TEST_ASSERT_EQUAL_UINT16(1920, calibrationStepFor(table, 0.));
    TEST_ASSERT_EQUAL_UINT16(3540, calibrationStepFor(table, 90.));
}

void test_values_between_two_points_interpolate_linearly(void) {
    CalibrationTable table = turnRateTable();
    // -45 sits halfway from -90 to 0: 300 + 0.5 * 1620.
    TEST_ASSERT_EQUAL_UINT16(1110, calibrationStepFor(table, -45.));
    // 45 sits halfway from 0 to 90: 1920 + 0.5 * 1620.
    TEST_ASSERT_EQUAL_UINT16(2730, calibrationStepFor(table, 45.));
}

void test_interpolation_works_across_the_sign_change(void) {
    CalibrationTable table;
    calibrationWipe(table);
    calibrationSet(table, -90, 300);
    calibrationSet(table, 90, 3540);
    // No point at 0 - the segment spans the sign change: 300 + 0.5 * 3240.
    TEST_ASSERT_EQUAL_UINT16(1920, calibrationStepFor(table, 0.));
    TEST_ASSERT_EQUAL_UINT16(1110, calibrationStepFor(table, -45.));
}

void test_turn_rate_past_the_outer_mark_parks_on_it(void) {
    // X-Plane hands out well over 110 degrees in a spin; the dial has no marks
    // out there, so the needle stays on the last calibrated point.
    CalibrationTable table = turnRateTable();
    TEST_ASSERT_EQUAL_UINT16(3540, calibrationStepFor(table, 91.));
    TEST_ASSERT_EQUAL_UINT16(3540, calibrationStepFor(table, 200.));
    TEST_ASSERT_EQUAL_UINT16(300, calibrationStepFor(table, -91.));
    TEST_ASSERT_EQUAL_UINT16(300, calibrationStepFor(table, -200.));
}

void test_slip_table_interpolates_each_side_on_its_own_slope(void) {
    CalibrationTable table = slipTable();
    // Left half spans 950 steps over 8 degrees, right half 814. -4 and +4 must
    // land on their own half's slope, not on a mirrored one.
    TEST_ASSERT_EQUAL_UINT16(475, calibrationStepFor(table, -4.));   // 0 + 0.5 * 950
    TEST_ASSERT_EQUAL_UINT16(1357, calibrationStepFor(table, 4.));   // 950 + 0.5 * 814
    // Sub-degree input works too: 950 + (0.5 / 8) * 814 = 1000.875 -> 1001.
    TEST_ASSERT_EQUAL_UINT16(1001, calibrationStepFor(table, 0.5));
}

void test_zero_is_not_the_home_step(void) {
    CalibrationTable table = turnRateTable();
    TEST_ASSERT_EQUAL_UINT16(1920, calibrationStepFor(table, 0.));
}

void test_an_uncalibrated_table_holds_the_needle_on_the_home_stop(void) {
    CalibrationTable table;
    calibrationWipe(table);
    TEST_ASSERT_EQUAL_UINT16(0, calibrationStepFor(table, 0.));
    TEST_ASSERT_EQUAL_UINT16(0, calibrationStepFor(table, 90.));
    TEST_ASSERT_EQUAL_UINT16(0, calibrationStepFor(table, -90.));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_wipe_leaves_no_points_at_all);
    RUN_TEST(test_points_are_kept_sorted_regardless_of_entry_order);
    RUN_TEST(test_setting_a_known_value_replaces_its_step);
    RUN_TEST(test_a_full_table_rejects_further_points);
    RUN_TEST(test_a_full_table_still_accepts_a_replacement);
    RUN_TEST(test_min_and_max_report_the_outer_points);
    RUN_TEST(test_min_and_max_of_an_empty_table_are_zero);
    RUN_TEST(test_calibrated_points_map_to_their_own_step);
    RUN_TEST(test_values_between_two_points_interpolate_linearly);
    RUN_TEST(test_interpolation_works_across_the_sign_change);
    RUN_TEST(test_turn_rate_past_the_outer_mark_parks_on_it);
    RUN_TEST(test_slip_table_interpolates_each_side_on_its_own_slope);
    RUN_TEST(test_zero_is_not_the_home_step);
    RUN_TEST(test_an_uncalibrated_table_holds_the_needle_on_the_home_stop);
    return UNITY_END();
}
```

- [ ] **Step 3: Tests laufen lassen — müssen fehlschlagen**

Run: `cd TurnCoordinatorCAN && pio test -e native`
Expected: Build-Fehler `TurnCoordinatorCalibration.h: No such file or directory`.

- [ ] **Step 4: Header schreiben**

`TurnCoordinatorCAN/include/TurnCoordinatorCalibration.h`:

```cpp
#ifndef TURNCOORDINATORCALIBRATION_H
#define TURNCOORDINATORCALIBRATION_H

#include <stdint.h>

// Degrees -> needle step mapping for both needles of the turn coordinator,
// described by a table of measured calibration points and linear interpolation
// in between. Deliberately Arduino-free so the mapping can be exercised by the
// native Unity tests in test/test_calibration (pio test -e native).
//
// Same shape as VerticalSpeedCAN's table, because the needles share its two
// quirks:
//
//  * Values are signed. Points are kept sorted ascending, so the leftmost
//    deflection sits at index 0 and the rightmost at count-1.
//  * 0 is NOT the mechanical home position. Each needle homes against a stop
//    off the scale; the 0 mark sits mid-dial at whatever step count the bench
//    calibration finds. So a wiped table has no points at all and holds the
//    needle on the home stop until the first point is taught.
//
// The turn rate needle uses degrees of the aircraft symbol (points around
// +-90), the slip ball degrees of slip (points at -8/0/8). One table each,
// both of this type. Nothing here assumes symmetry about 0 - the slip dial
// is not symmetric, and the table simply records what the bench measured.

static const uint8_t kMaxCalibrationPoints = 12;

struct CalibrationPoint
{
    int16_t value;
    uint16_t step;
};

struct CalibrationTable
{
    uint8_t count;
    CalibrationPoint points[kMaxCalibrationPoints];
};

// Factory reset: no points at all. The needle parks on the mechanical home
// stop (step 0) until the bench teaches at least one point.
inline void calibrationWipe(CalibrationTable &table)
{
    table.count = 0;
}

// Adds or updates the point for `value`, keeping the table sorted by value.
// Returns false only when a *new* point does not fit any more - replacing a
// known value always succeeds.
inline bool calibrationSet(CalibrationTable &table, int16_t value, uint16_t step)
{
    uint8_t index = 0;
    while (index < table.count && table.points[index].value < value)
    {
        index++;
    }

    if (index < table.count && table.points[index].value == value)
    {
        table.points[index].step = step;
        return true;
    }

    if (table.count >= kMaxCalibrationPoints)
    {
        return false;
    }

    for (uint8_t i = table.count; i > index; i--)
    {
        table.points[i] = table.points[i - 1];
    }
    table.points[index].value = value;
    table.points[index].step = step;
    table.count++;
    return true;
}

// The leftmost calibrated deflection, i.e. the bottom of the displayable range.
inline int16_t calibrationMinValue(const CalibrationTable &table)
{
    if (table.count == 0)
    {
        return 0;
    }
    return table.points[0].value;
}

// The rightmost calibrated deflection, i.e. the top of the displayable range.
inline int16_t calibrationMaxValue(const CalibrationTable &table)
{
    if (table.count == 0)
    {
        return 0;
    }
    return table.points[table.count - 1].value;
}

// Needle position for a value, clamped to the lowest and highest calibrated
// point - the dial simply has no marks beyond either end.
inline uint16_t calibrationStepFor(const CalibrationTable &table, float value)
{
    if (table.count == 0)
    {
        return 0;
    }
    if (value <= (float)table.points[0].value)
    {
        return table.points[0].step;
    }

    const uint8_t last = table.count - 1;
    if (value >= (float)table.points[last].value)
    {
        return table.points[last].step;
    }

    uint8_t upper = 1;
    while (upper < last && (float)table.points[upper].value < value)
    {
        upper++;
    }

    const CalibrationPoint &low = table.points[upper - 1];
    const CalibrationPoint &high = table.points[upper];
    const float span = (float)high.value - (float)low.value;
    const float ratio = (value - (float)low.value) / span;
    const float step = (float)low.step + ratio * ((float)high.step - (float)low.step);
    return (uint16_t)(step + 0.5f);
}

#endif // TURNCOORDINATORCALIBRATION_H
```

- [ ] **Step 5: Tests laufen lassen — müssen bestehen**

Run: `cd TurnCoordinatorCAN && pio test -e native`
Expected: `14 Tests 0 Failures 0 Ignored` — `PASSED`.

- [ ] **Step 6: Commit**

```bash
git add TurnCoordinatorCAN/platformio.ini TurnCoordinatorCAN/lib/vid6608 TurnCoordinatorCAN/include/TurnCoordinatorCalibration.h TurnCoordinatorCAN/test/test_calibration/test_calibration.cpp
git commit -m "feat(turn-coordinator-can): project scaffold and per-needle calibration table

Vendors the non-blocking vid6608 fork from VerticalSpeedCAN and copies its
signed, mid-dial-zero calibration table; one table per needle. Slip test pins
the asymmetric dial.

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 3: Configuration, DebugLog, `TurnCoordinator`, `CAN`, `main` — Firmware baut

**Files:**
- Create: `TurnCoordinatorCAN/src/Configuration.h`
- Create: `TurnCoordinatorCAN/src/DebugLog.h`
- Create: `TurnCoordinatorCAN/src/TurnCoordinator.h`
- Create: `TurnCoordinatorCAN/src/TurnCoordinator.cpp`
- Create: `TurnCoordinatorCAN/src/CAN.h`
- Create: `TurnCoordinatorCAN/src/CAN.cpp`
- Create: `TurnCoordinatorCAN/src/main.cpp`

**Interfaces:**
- Consumes: Task 2 (`CalibrationTable`, `calibrationStepFor`, `calibrationSet`, `calibrationWipe`), `vid6608` (`moveTo`, `zero`, `getPosition`, `loop`, `setAccelTable`), `InstrumentCAN` aus `shared/CANBase`.
- Produces: `enum Needle { turnRate = 0, sideSlip, needleCount }` (in `Configuration.h`), Klasse `TurnCoordinator` mit `moveNeedle(Needle, float deg, bool calibration = false)`, `setTurnCoordinator(float turnRateDeg, float slipDeg)`, `setBrightness(uint8_t)`, `home()`, `bool calibratePoint(Needle, int16_t deg)`, `wipeCalibration()`, `const CalibrationTable& calibration(Needle) const`, `loop()`. Klasse `CAN(TurnCoordinator*)`.

- [ ] **Step 1: `Configuration.h`**

```cpp
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <CanNodeId.h>

#define BENCHDEBUG 0

const CanNodeId kNodeId = CanNodeId::turnCoordinatorNodeId;

// CAN (MCP2515 on hardware SPI: D11 MOSI, D12 MISO, D13 SCK).
// /CS sits on D10 deliberately: D10 is the AVR /SS pin and must be an output in
// master mode, so making it the chip select lets SPI.begin() drive it.
// /INT must be D2 or D3 (INT0/INT1), BaseCAN hangs its RX handler off
// attachInterrupt().
const uint8_t kCanIntPin = 2;
const uint8_t kCanCSPin = 10;

// The two needles. Index into every per-needle array below and into the
// calibration tables in EEPROM; also the order of the two int16 in CAN 0x103.
enum Needle {
    turnRate = 0,   // aircraft symbol, degrees of indicated roll
    sideSlip,       // ball, degrees of slip
    needleCount
};

// One AX1201728SG (VID6606/6608 clone) drives both X27.168: STEP/DIR per
// channel, one shared /RST. 12 steps per degree.
const uint8_t kStepPins[needleCount] = {7, 4};
const uint8_t kDirPins[needleCount] = {8, 6};
const uint8_t kRstPin = 9;

const uint16_t kMaximumDegree = 320;
const uint16_t kSteps = kMaximumDegree * 12; // 320 degrees at 1/12 degree steps

// Panel light. D3 is Timer2 PWM; analogWrite there leaves Timer0 and thus
// millis()/micros() alone.
const uint8_t kLightPin = 3;

// The degree-to-needle mapping lives entirely in EEPROM: one table of measured
// calibration points per needle (see include/TurnCoordinatorCalibration.h),
// taught on the bench with ho/ct/pt (turn rate) and ho/cs/ps (slip) and
// interpolated linearly in between. There is no compiled-in range or
// direction - both come out of the taught points. The 0 mark of either needle
// is just another taught point, not the mechanical home stop.

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

#endif // CONFIGURATION_H
```

- [ ] **Step 2: `DebugLog.h`**

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

- [ ] **Step 3: `TurnCoordinator.h`**

```cpp
#ifndef TURNCOORDINATOR_H
#define TURNCOORDINATOR_H
#include <Arduino.h>
#include <vid6608.h>
#include "TurnCoordinatorCalibration.h"
#include "Configuration.h"

class TurnCoordinator
{
public:
    TurnCoordinator();
    ~TurnCoordinator();

    // Moves one needle. Normal mode maps `deg` through that needle's
    // calibration table; calibration mode drives raw dial degrees from the
    // home stop (12 steps per degree) so a mark can be found and taught.
    void moveNeedle(Needle needle, float deg, bool calibration = false);

    // Both needles from one 0x103 frame.
    void setTurnCoordinator(float turnRateDeg, float slipDeg);

    void setBrightness(uint8_t brightness);

    // Re-runs the blocking homing sweep on both motors, one after the other,
    // and parks each needle on its mechanical stop. Calibration always starts
    // here, because every stored step count is relative to that stop. The stop
    // is *not* the 0 mark - that one is taught like any other point.
    void home();

    // Records the needle's current position as the calibration point for
    // `deg` (negative for left deflection). Returns false if the table is full.
    bool calibratePoint(Needle needle, int16_t deg);

    // Factory reset: drops every calibration point of both needles, leaving
    // them parked on the home stop.
    void wipeCalibration();

    const CalibrationTable &calibration(Needle needle) const { return config.tables[needle]; }

    void loop();

private:
    struct Config
    {
        uint32_t magic;
        uint16_t version;
        CalibrationTable tables[needleCount];
    };

    void loadConfig();
    void saveConfig();

    vid6608 *motors[needleCount];
    Config config;
};

#endif
```

- [ ] **Step 4: `TurnCoordinator.cpp`**

```cpp
#include "TurnCoordinator.h"
#include "DebugLog.h"
#include <EEPROM.h>

static const uint32_t kTurnCoordinatorConfigMagic = 0x54434E31; // 'T','C','N','1'
static const uint16_t kTurnCoordinatorConfigVersion = 1;
static const uint16_t kTurnCoordinatorEepromAddress = 0;

// CAN updates arrive at ~50Hz, so most moves only span a few steps and never
// leave the slow ramp-up zone of vid6608's default table (reaches top speed
// only after 800 steps). Same start delay (stall-safe) and top speed (proven)
// as the default table, but ramps to cruise within 32 steps instead of 800.
// Copied from VerticalSpeedCAN.
static vid6608::AccelTable kTurnCoordinatorAccelTable[] = {
    {2, 3000},
    {4, 2400},
    {6, 1900},
    {8, 1500},
    {10, 1150},
    {12, 900},
    {15, 700},
    {18, 550},
    {21, 430},
    {24, 360},
    {27, 320},
    {32, 300},
};

TurnCoordinator::TurnCoordinator()
{
    pinMode(kRstPin, OUTPUT);
    digitalWrite(kRstPin, LOW);
    for (uint8_t i = 0; i < needleCount; i++)
    {
        motors[i] = new vid6608(kStepPins[i], kDirPins[i], kSteps);
        motors[i]->setAccelTable(kTurnCoordinatorAccelTable);
    }

    pinMode(kLightPin, OUTPUT);
    analogWrite(kLightPin, 0);

    for (int i = 0; i < 3; i++)
    {
        delay(250);
        analogWrite(kLightPin, 255);
        delay(500);
        analogWrite(kLightPin, 0);
    }

    // One /RST for both channels: release it once, then home the motors one
    // after the other. zero() is blocking and runs once at startup.
    digitalWrite(kRstPin, HIGH);
    for (uint8_t i = 0; i < needleCount; i++)
    {
        motors[i]->zero();
    }

    loadConfig();
}

TurnCoordinator::~TurnCoordinator()
{
    for (uint8_t i = 0; i < needleCount; i++)
    {
        delete motors[i];
    }
}

void TurnCoordinator::loadConfig()
{
    EEPROM.get(kTurnCoordinatorEepromAddress, config);
    if (config.magic != kTurnCoordinatorConfigMagic || config.version != kTurnCoordinatorConfigVersion)
    {
        DEBUGLOG_PRINTLN(F("TC: No valid EEPROM config, writing defaults"));
        config.magic = kTurnCoordinatorConfigMagic;
        config.version = kTurnCoordinatorConfigVersion;
        for (uint8_t i = 0; i < needleCount; i++)
        {
            calibrationWipe(config.tables[i]);
        }
        EEPROM.put(kTurnCoordinatorEepromAddress, config);
        return;
    }

    // Same magic and version but a nonsensical count in either table: refuse
    // to index past the array and fall back to factory settings for both. A
    // count of 0 is legal - a wiped table holds no anchor point.
    bool corrupt = false;
    for (uint8_t i = 0; i < needleCount; i++)
    {
        if (config.tables[i].count > kMaxCalibrationPoints)
        {
            corrupt = true;
        }
    }
    if (corrupt)
    {
        DEBUGLOG_PRINTLN(F("TC: EEPROM calibration corrupt, wiping"));
        for (uint8_t i = 0; i < needleCount; i++)
        {
            calibrationWipe(config.tables[i]);
        }
        EEPROM.put(kTurnCoordinatorEepromAddress, config);
        return;
    }

    DEBUGLOG_PRINT(F("TC: EEPROM config loaded, points turn="));
    DEBUGLOG_PRINT(config.tables[turnRate].count);
    DEBUGLOG_PRINT(F(" slip="));
    DEBUGLOG_PRINTLN(config.tables[sideSlip].count);
}

void TurnCoordinator::saveConfig()
{
    EEPROM.put(kTurnCoordinatorEepromAddress, config);
    DEBUGLOG_PRINTLN(F("TC: config saved"));
}

void TurnCoordinator::home()
{
    // Feeding the known position in keeps zero() from taking the long way
    // round and bouncing off the end stop.
    for (uint8_t i = 0; i < needleCount; i++)
    {
        motors[i]->zero(motors[i]->getPosition());
    }
}

bool TurnCoordinator::calibratePoint(Needle needle, int16_t deg)
{
    const uint16_t step = motors[needle]->getPosition();
    if (!calibrationSet(config.tables[needle], deg, step))
    {
        DEBUGLOG_PRINTLN(F("TC: calibration table full"));
        return false;
    }
    DEBUGLOG_PRINT(F("TC: needle "));
    DEBUGLOG_PRINT(needle);
    DEBUGLOG_PRINT(F(" calibrated "));
    DEBUGLOG_PRINT(deg);
    DEBUGLOG_PRINT(F("deg at step "));
    DEBUGLOG_PRINTLN(step);
    saveConfig();
    return true;
}

void TurnCoordinator::wipeCalibration()
{
    for (uint8_t i = 0; i < needleCount; i++)
    {
        calibrationWipe(config.tables[i]);
    }
    saveConfig();
}

void TurnCoordinator::setBrightness(uint8_t brightness)
{
    analogWrite(kLightPin, brightness);
}

void TurnCoordinator::moveNeedle(Needle needle, float deg, bool calibration)
{
    if (calibration)
    {
        motors[needle]->moveTo(static_cast<uint16_t>(deg * 12.)); // 12 steps per degree
        return;
    }

    // No isMoving() bail-out: the motor retargets on the fly (see the local
    // vid6608 fork), so feeding it fresh setpoints at 50Hz keeps the needle
    // tracking smoothly instead of stopping to accept each waypoint.
    //
    // Piecewise linear across the calibration points; values beyond the
    // lowest or highest calibrated point park on it, because the dial has no
    // marks out there.
    motors[needle]->moveTo(calibrationStepFor(config.tables[needle], deg));
}

void TurnCoordinator::setTurnCoordinator(float turnRateDeg, float slipDeg)
{
    moveNeedle(turnRate, turnRateDeg);
    moveNeedle(sideSlip, slipDeg);
}

void TurnCoordinator::loop()
{
    for (uint8_t i = 0; i < needleCount; i++)
    {
        motors[i]->loop();
    }
}
```

- [ ] **Step 5: `CAN.h`**

```cpp
#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <InstrumentCAN.h>
#include "TurnCoordinator.h"
#include <CanMessageId.h>
#include <CanNodeId.h>

class CAN : public InstrumentCAN {
    public:
        CAN(TurnCoordinator* turnCoordinator);

    protected:
        bool instrumentBegin() override;
        void onStartupFail() override;
        void handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t* data) override;
        void onGatewayHeartbeatTimeout() override;
        void onGatewayHeartbeatDiscovered() override;

    private:
        TurnCoordinator* turnCoordinator;
};

#endif
```

- [ ] **Step 6: `CAN.cpp`**

```cpp
#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

CAN::CAN(TurnCoordinator *turnCoordinator)
    : InstrumentCAN(kCanCSPin, kCanIntPin, CANFirmwareInfo{static_cast<uint16_t>(kNodeId), 1, 0}),
      turnCoordinator(turnCoordinator)
{
    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

void CAN::onStartupFail()
{
    DEBUGLOG_PRINTLN(F("CAN startup FAIL"));
    turnCoordinator->setBrightness(0);
}

bool CAN::instrumentBegin()
{
    // Beide RX-Buffer vergleichen alle ID-Bits
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1

    // RXB0: Turn coordinator
    canBus->init_Filt(0, 0, CAN_STD_ID(CanMessageId::turnCoordinator));
    canBus->init_Filt(1, 0, CAN_STD_ID(CanMessageId::turnCoordinator));

    // RXB1: Lights und Gateway Heartbeat
    canBus->init_Filt(2, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(3, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(4, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(5, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));

    canBus->setMode(MCP_NORMAL);

    turnCoordinator->setTurnCoordinator(0., 0.);
    turnCoordinator->setBrightness(0);

    return true;
}

void CAN::handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t *data)
{
    // No per-frame logging: Serial.print blocks long enough to stretch the
    // step period of both motors.

    // We currently expect standard frames only (ext == 0).
    (void)ext;

    switch (id)
    {
    case CanMessageId::turnCoordinator:
    {
        if (len >= 4)
        {
            // [0..1] turn rate, [2..3] slip, both int16 degrees * 100, signed.
            const int16_t turn100 = static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]));
            const int16_t slip100 = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3]));
            turnCoordinator->setTurnCoordinator(static_cast<float>(turn100) / 100., static_cast<float>(slip100) / 100.);
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
            turnCoordinator->setBrightness(pwm);
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
    turnCoordinator->setBrightness(0);
}

void CAN::onGatewayHeartbeatDiscovered()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat OK"));
    turnCoordinator->setBrightness(255);
}
```

- [ ] **Step 7: `main.cpp`**

```cpp
#include <Arduino.h>
#include "Configuration.h"
#include "TurnCoordinator.h"
#include "DebugLog.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "CAN.h"
CAN* canBus;
#endif

TurnCoordinator* turnCoordinator;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("Turn Coordinator initializing..."));

  turnCoordinator = new TurnCoordinator();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(turnCoordinator);
  #else
  canBus = new CAN(turnCoordinator);
  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("Turn Coordinator started up"));
  }
  #endif
}

void loop() {
  #if BENCHDEBUG
  benchDebug->loop();
  #else
  canBus->loop();
  #endif
  turnCoordinator->loop();
}
```

- [ ] **Step 8: Build**

Run: `cd TurnCoordinatorCAN && pio run -e nano`
Expected: `SUCCESS`. RAM-Anteil notieren (zwei Tabellen à 12×4 Byte + Header ≈ 110 Byte Config im RAM; muss deutlich unter dem VSI-Wert + 60 Byte liegen — bei > 70 % siehe `avr-ram-headroom`-Memory).

- [ ] **Step 9: Native Tests weiterhin grün**

Run: `cd TurnCoordinatorCAN && pio test -e native`
Expected: `14 Tests 0 Failures`.

- [ ] **Step 10: Commit**

```bash
git add TurnCoordinatorCAN/src/Configuration.h TurnCoordinatorCAN/src/DebugLog.h TurnCoordinatorCAN/src/TurnCoordinator.h TurnCoordinatorCAN/src/TurnCoordinator.cpp TurnCoordinatorCAN/src/CAN.h TurnCoordinatorCAN/src/CAN.cpp TurnCoordinatorCAN/src/main.cpp
git commit -m "feat(turn-coordinator-can): two-needle firmware on CAN 0x103

TurnCoordinator drives both X27 channels of one AX1201728SG through the
vid6608 fork, one EEPROM calibration table per needle; CAN maps 0x103 to
both needles and 0x203 to the panel light.

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 4: BenchDebug

**Files:**
- Create: `TurnCoordinatorCAN/src/BenchDebug.h`
- Create: `TurnCoordinatorCAN/src/BenchDebug.cpp`
- Modify (temporär): `TurnCoordinatorCAN/src/Configuration.h` (`BENCHDEBUG 1` für den Build-Check, danach zurück auf 0)

**Interfaces:**
- Consumes: Task 3 (`TurnCoordinator`, `Needle`), Task 2 (`calibrationMinValue`, `calibrationMaxValue`, `kMaxCalibrationPoints`).
- Produces: Klasse `BenchDebug(TurnCoordinator*)` mit `loop()`.

- [ ] **Step 1: `BenchDebug.h`**

```cpp
#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include "Configuration.h"

#if BENCHDEBUG
#include <Arduino.h>
#include "TurnCoordinator.h"

class BenchDebug {
    public:
        BenchDebug(TurnCoordinator* turnCoordinator);
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        bool handleTurnCoordinatorInput(char* command);
        void printCalibration();
        void printTable(const __FlashStringHelper* name, const CalibrationTable& table);

        static const uint8_t kInputBufferSize = 32;
        char inputBuffer[kInputBufferSize];
        uint8_t inputLength = 0;

        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        static const uint16_t kDeliveryIntervalMs = 20; // 50Hz, matches real CAN cadence

        bool continuousTestActive = false;

        // Random targets per needle are not applied in one jump - they are
        // interpolated and delivered via turnCoordinator->moveNeedle() at
        // kDeliveryIntervalMs, just like real 50Hz CAN telemetry would.
        struct NeedleMove {
            float from = 0;
            float to = 0;
            uint32_t startMillis = 0;
            uint32_t durationMillis = 0;
        };
        NeedleMove moves[needleCount];
        uint32_t lastDeliveryMillis = 0;

        TurnCoordinator* turnCoordinator;
};
#endif
#endif // BENCHDEBUG_H
```

- [ ] **Step 2: `BenchDebug.cpp`**

```cpp
#include <BenchDebug.h>
#if BENCHDEBUG
#include <string.h>
#include <stdlib.h>
#include "DebugLog.h"

const int kLedPin = 13;

BenchDebug::BenchDebug(TurnCoordinator* turnCoordinator)
: turnCoordinator(turnCoordinator)
{
    #if !DEBUGLOG_ENABLE
    Serial.begin(115200);
    #endif
    Serial.println(F("Turn Coordinator BenchDebug"));

    inputBuffer[0] = '\0';
    inputLength = 0;

    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);

    turnCoordinator->setTurnCoordinator(0., 0.);

    Serial.println(F("System running!"));
}

BenchDebug::~BenchDebug()
{
}

void BenchDebug::printTable(const __FlashStringHelper* name, const CalibrationTable& table) {
    Serial.print(name);
    Serial.print(F(" calibration points ("));
    Serial.print(table.count);
    Serial.print(F("/"));
    Serial.print(kMaxCalibrationPoints);
    Serial.println(F("):"));
    if (table.count == 0) {
        Serial.println(F("  none - needle parks on the home stop"));
    }
    for (uint8_t i = 0; i < table.count; i++) {
        Serial.print(F("  "));
        Serial.print(table.points[i].value);
        Serial.print(F("deg -> step "));
        Serial.println(table.points[i].step);
    }
}

void BenchDebug::printCalibration() {
    printTable(F("Turn rate"), turnCoordinator->calibration(turnRate));
    printTable(F("Slip"), turnCoordinator->calibration(sideSlip));
}

bool BenchDebug::handleTurnCoordinatorInput(char* command) {
    if (continuousTestActive &&
        (strncmp(command, "tr", 2) == 0 ||
         strncmp(command, "sl", 2) == 0 ||
         strncmp(command, "ct", 2) == 0 ||
         strncmp(command, "cs", 2) == 0 ||
         strncmp(command, "pt", 2) == 0 ||
         strncmp(command, "ps", 2) == 0 ||
         strncmp(command, "ho", 2) == 0 ||
         strncmp(command, "cw", 2) == 0)) {
        Serial.println(F("Continuous test running. Type 'cx' to stop."));
        return true;
    }

    if (strncmp(command, "tr", 2) == 0) {
        turnCoordinator->moveNeedle(turnRate, atof(command + 2));
        return true;
    } else if (strncmp(command, "sl", 2) == 0) {
        turnCoordinator->moveNeedle(sideSlip, atof(command + 2));
        return true;
    } else if (strncmp(command, "ct", 2) == 0) {
        turnCoordinator->moveNeedle(turnRate, atof(command + 2), true);
        return true;
    } else if (strncmp(command, "cs", 2) == 0) {
        turnCoordinator->moveNeedle(sideSlip, atof(command + 2), true);
        return true;
    } else if (strncmp(command, "ho", 2) == 0) {
        Serial.println(F("Homing both needles..."));
        turnCoordinator->home();
        Serial.println(F("Needles homed."));
        return true;
    } else if (strncmp(command, "pt", 2) == 0 || strncmp(command, "ps", 2) == 0) {
        // The needle must already sit on the dial mark, put there with ct/cs.
        // Negative values are expected here - pt-90 teaches the left mark and
        // pt0 the level mark, which is nowhere near the mechanical home stop.
        const Needle needle = (command[1] == 't') ? turnRate : sideSlip;
        const int16_t deg = static_cast<int16_t>(atoi(command + 2));
        if (!turnCoordinator->calibratePoint(needle, deg)) {
            Serial.println(F("Calibration table full. Wipe with 'cw' or reuse a point."));
            return true;
        }
        printCalibration();
        return true;
    } else if (strncmp(command, "cw", 2) == 0) {
        turnCoordinator->wipeCalibration();
        Serial.println(F("Calibration wiped (both needles)."));
        printCalibration();
        return true;
    } else if (strncmp(command, "br", 2) == 0) {
        turnCoordinator->setBrightness(static_cast<uint8_t>(atoi(command + 2)));
        return true;
    } else if (strncmp(command, "co", 2) == 0) {
        continuousTestActive = true;
        for (uint8_t i = 0; i < needleCount; i++) {
            moves[i].from = 0;
            moves[i].to = 0;
            moves[i].startMillis = millis();
            moves[i].durationMillis = 0; // forces an immediate target pick on the first tick
        }
        lastDeliveryMillis = millis();
        return true;
    } else if (strncmp(command, "cx", 2) == 0) {
        continuousTestActive = false;
        return true;
    } else if (command[0] == '?') {
        Serial.println(F("Turn Coordinator Commands:"));
        Serial.println(F("tr<deg>: turn rate needle via calibration (negative = left)"));
        Serial.println(F("sl<deg>: slip ball via calibration (negative = left)"));
        Serial.println(F("br<0..255>: set light brightness"));
        Serial.println(F("ho: home both needles (start of every calibration)"));
        Serial.println(F("ct<degree> / cs<degree>: move turn rate / slip needle to a raw dial angle"));
        Serial.println(F("pt<deg> / ps<deg>: store current turn rate / slip position as the mark for <deg>"));
        Serial.println(F("cw: wipe all calibration points of both needles (factory reset)"));
        Serial.println(F("co: start continuous test on both needles"));
        Serial.println(F("cx: stop continuous test"));
        Serial.println(F("Calibration: ho, then ct<degree> until the needle sits on a mark,"));
        Serial.println(F("then pt<deg> for that mark. Repeat for every mark you want -"));
        Serial.println(F("including pt0, since 0 is not the home stop. Same with cs/ps for the ball."));
        printCalibration();
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
            inputBuffer[inputLength] = '\0';

            bool commandExecuted = false;
            char* token = strtok(inputBuffer, " ");
            while (token != nullptr) {
                if (strlen(token) > 0 && handleTurnCoordinatorInput(token)) {
                    commandExecuted = true;
                }
                token = strtok(nullptr, " ");
            }

            if (!commandExecuted) {
                Serial.println(F("Unknown command. Type '?' for help."));
            }

            inputLength = 0;
        }
        else if (receivedChar != '\r')
        {
            if (inputLength < kInputBufferSize - 1) {
                inputBuffer[inputLength++] = receivedChar;
                Serial.print(receivedChar);
            }
        }
    }
}

void BenchDebug::loop()
{
    if (millis() - heartbeat > 1000L)
    {
        heartbeat = millis();
        digitalWrite(kLedPin, heartbeatLedOn ? HIGH : LOW);
        heartbeatLedOn = !heartbeatLedOn;
    }

    if (continuousTestActive)
    {
        // Both needles: random targets, interpolated and delivered at 50Hz via
        // the real moveNeedle() API, just like actual CAN telemetry would drive
        // them. Targets span the calibrated range plus a little margin on each
        // side, so both clamps get exercised too.
        if (millis() - lastDeliveryMillis >= kDeliveryIntervalMs)
        {
            lastDeliveryMillis += kDeliveryIntervalMs;

            for (uint8_t i = 0; i < needleCount; i++)
            {
                const Needle needle = static_cast<Needle>(i);
                NeedleMove& move = moves[i];

                uint32_t elapsed = millis() - move.startMillis;
                if (elapsed >= move.durationMillis)
                {
                    move.from = move.to;
                    const CalibrationTable& table = turnCoordinator->calibration(needle);
                    const long minValue = (long)calibrationMinValue(table);
                    const long maxValue = (long)calibrationMaxValue(table);
                    // Margin scaled to the needle: a few degrees past the slip
                    // ball's +-8, a good deal past the turn needle's +-90.
                    const long margin = (needle == sideSlip) ? 2L : 20L;
                    move.to = random(minValue - margin, maxValue + margin + 1);
                    move.durationMillis = random(500, 2000);
                    move.startMillis = millis();
                    elapsed = 0;
                }

                float progress = static_cast<float>(elapsed) / static_cast<float>(move.durationMillis);
                float interpolated = move.from + (move.to - move.from) * progress;
                turnCoordinator->moveNeedle(needle, interpolated);
            }
        }
    }

    handleUserInput();
}
#endif
```

- [ ] **Step 3: Bench-Build prüfen**

In `Configuration.h` `#define BENCHDEBUG 1` setzen.

Run: `cd TurnCoordinatorCAN && pio run -e nano`
Expected: `SUCCESS`.

Dann `#define BENCHDEBUG 0` zurücksetzen und erneut bauen:

Run: `cd TurnCoordinatorCAN && pio run -e nano`
Expected: `SUCCESS`.

- [ ] **Step 4: Commit**

```bash
git add TurnCoordinatorCAN/src/BenchDebug.h TurnCoordinatorCAN/src/BenchDebug.cpp
git commit -m "feat(turn-coordinator-can): serial bench console for both needles

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 5: DCU — Serial `0x10` → CAN `0x103`, Bench-Befehl `tc`

**Files:**
- Modify: `DCU/src/DCUReceiver.h`
- Modify: `DCU/src/DCUReceiver.cpp`
- Modify: `DCU/src/BenchDebug.h`
- Modify: `DCU/src/BenchDebug.cpp`
- Modify: `DCU/CLAUDE.md`

**Interfaces:**
- Consumes: Task 1 (`MessageType::SerialMessageTurnCoordinator`, `TurnCoordinatorMessage`, `CanMessageId::turnCoordinator`), `attitudeDeg100()` aus `AttitudeWire.h` (bereits inkludiert), `packBE16` aus `WireEncoding.h`.
- Produces: `DCUReceiver::sendTurnCoordinator()`, `turnCoordinatorMeta`; Bench-Befehl `tc<turn>,<slip>`.

- [ ] **Step 1: `DCUReceiver.h`**

Nach `void sendAttitude();` einfügen:

```cpp
        void sendTurnCoordinator();
```

Nach dem Block `// Attitude indicator (CAN 0x101)` … `int16_t attitudeRoll100 = 0;` einfügen:

```cpp
        // Turn coordinator (CAN 0x103)
        int16_t turnRate100 = 0;
        int16_t slip100 = 0;
```

Nach `MessageMeta attitudeMeta;` einfügen:

```cpp
        MessageMeta turnCoordinatorMeta;
```

- [ ] **Step 2: `DCUReceiver.cpp` — Konstruktor, Case, Sender, Resync**

Im Konstruktor nach `attitudeMeta = {0, 5000};`:

```cpp
  turnCoordinatorMeta = {0, 5000};
```

In `handleFrame` nach dem `case MessageType::SerialMessageAttitude: { ... break; }`-Block und vor `default:` einfügen:

```cpp
  case MessageType::SerialMessageTurnCoordinator:
  {
    if (len != sizeof(TurnCoordinatorMessage))
      return;

    TurnCoordinatorMessage msg;
    memcpy(&msg, payload, sizeof(msg));

    // Same conversion as the attitude frame: signed, rounded away from zero,
    // clamped to int16, NaN -> 0.
    const int16_t turn100 = attitudeDeg100(msg.turnRateDeg);
    const int16_t s100 = attitudeDeg100(msg.slipDeg);

    if (turn100 != turnRate100 || s100 != slip100)
    {
      turnRate100 = turn100;
      slip100 = s100;
      sendTurnCoordinator();
    }
    break;
  }
```

Nach `void DCUReceiver::sendAttitude() { ... }` einfügen:

```cpp
void DCUReceiver::sendTurnCoordinator()
{
  byte data[8] = {0};

  // [0..1] turn rate, [2..3] slip, both int16 degrees * 100, [4..7] reserved.
  packBE16(data + 0, static_cast<uint16_t>(turnRate100));
  packBE16(data + 2, static_cast<uint16_t>(slip100));

  canBus->sendMessage(CanMessageId::turnCoordinator, 8, data);

  turnCoordinatorMeta.lastSendTimestamp = millis();
}
```

In `checkMaxAgeResync()` nach dem `attitudeMeta`-Block (vor der schließenden Klammer der Funktion):

```cpp
  if (isStale(turnCoordinatorMeta.lastSendTimestamp, now, turnCoordinatorMeta.maxAgeMs))
  {
    DEBUGLOG_PRINTLN(String(F("MaxAge resync for turnCoordinator")));
    sendTurnCoordinator();
  }
```

- [ ] **Step 3: `BenchDebug.h` (DCU)**

Nach `void sendCompass();` in der Methodenliste:

```cpp
        void sendTurnCoordinator();
```

Nach `float compassHeadingDeg = 0.;`:

```cpp
        // Turn coordinator: one frame carries both needles, so one command
        // sets both (tc<turn>,<slip>).
        float turnRateDeg = 0.;
        float slipDeg = 0.;
```

- [ ] **Step 4: `BenchDebug.cpp` (DCU)**

Nach `void BenchDebug::sendCompass() { ... }` einfügen:

```cpp
void BenchDebug::sendTurnCoordinator() {
    byte data[8] = {0};
    // [0..1] turn rate, [2..3] slip, both int16 degrees * 100. [4..7] reserved.
    packBE16(data + 0, static_cast<uint16_t>(attitudeDeg100(turnRateDeg)));
    packBE16(data + 2, static_cast<uint16_t>(attitudeDeg100(slipDeg)));

    Serial.println(String(F("Send Turn Coordinator: turn ")) + turnRateDeg + F(" deg, slip ") + slipDeg + F(" deg"));

    canBus->sendMessage(CanMessageId::turnCoordinator, 8, data);
}
```

Oben in der Datei bei den Includes `#include "AttitudeWire.h"` ergänzen (falls noch nicht vorhanden; `WireEncoding.h` ist für `packBE16` schon da).

In `handleAltimeterInput` nach dem `else if (command.startsWith("co")) { ... return true; }`-Block einfügen:

```cpp
    } else if (command.startsWith("tc")) {
        // tc<turn>,<slip> - e.g. tc-45,3.5. Missing slip keeps the old slip.
        String rString = command.substring(2);
        rString.trim();
        const int comma = rString.indexOf(',');
        if (comma < 0) {
            turnRateDeg = rString.toFloat();
        } else {
            turnRateDeg = rString.substring(0, comma).toFloat();
            slipDeg = rString.substring(comma + 1).toFloat();
        }
        Serial.println(String(F("Turn coordinator set to turn ")) + turnRateDeg + F(" deg, slip ") + slipDeg + F(" deg"));
        sendTurnCoordinator();
        return true;
```

In der `?`-Hilfe nach der `co<deg>`-Zeile:

```cpp
        Serial.println(F("tc<turn>,<slip>: set turn coordinator, degrees, negative = left"));
```

- [ ] **Step 5: Builds + Tests**

Run: `cd DCU && pio run -e megaatmega2560`
Expected: `SUCCESS`.

In `DCU/src/Configuration.h` `BENCHDEBUG 1` setzen, `pio run -e megaatmega2560` → `SUCCESS`, zurück auf `0`, erneut bauen → `SUCCESS`.

Run: `cd DCU && pio test -e native`
Expected: alle Suiten `PASSED` (unverändert; `attitudeDeg100` ist durch `test_attitude_wire` abgedeckt).

- [ ] **Step 6: `DCU/CLAUDE.md`**

In der `DCUReceiver`-Aufzählung `SerialMessageAttitude)` → `SerialMessageAttitude`, `SerialMessageTurnCoordinator)`.

Nach dem Absatz, der mit `test_attitude_wire).` endet, einfügen:

```markdown
  `SerialMessageTurnCoordinator` (`0x10`) carries `TurnCoordinatorMessage` (two floats, turn rate
  then slip); `DCUReceiver` repacks it into CAN `0x103` as two big-endian `int16` of degrees × 100
  through the same `attitudeDeg100()` — one helper for every signed-degrees frame.
```

Im Abschnitt `## BenchDebug mode` die Befehlsliste `(`lt`/`rt`/.../`hb`)` um `` `tc` `` ergänzen und nach dem Satz über `co<deg>` einfügen:

```markdown
`tc<turn>,<slip>` sends a `turnCoordinator` (0x103) frame to TurnCoordinatorCAN — one frame
carries both needles, so one command sets both.
```

- [ ] **Step 7: Commit**

```bash
git add DCU/src/DCUReceiver.h DCU/src/DCUReceiver.cpp DCU/src/BenchDebug.h DCU/src/BenchDebug.cpp DCU/CLAUDE.md
git commit -m "feat(dcu): relay serial 0x10 turn coordinator to CAN 0x103

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 6: Plugin — Datarefs + 50-Hz-Sender

**Files:**
- Modify: `DCUProviderPlugin/src/DataRefManager.h`
- Modify: `DCUProviderPlugin/src/DataRefManager.cpp`
- Modify: `DCUProviderPlugin/src/DCUProvider.h`
- Modify: `DCUProviderPlugin/src/DCUProvider.cpp`

**Interfaces:**
- Consumes: Task 1 (`MessageType::SerialMessageTurnCoordinator`, `TurnCoordinatorMessage`), `DataRefManager::readFloat`, `MessageQueue::enqueueTx`.
- Produces: `DataRefManager::getTurnRateDeg()`, `getSlipDeg()`.

- [ ] **Step 1: `DataRefManager.h`**

Nach `float getRollVacuumDeg() const;` einfügen:

```cpp
    // Turn coordinator
    float getTurnRateDeg() const;  // turn_rate_roll_deg_pilot, degrees of the aircraft symbol, right turn positive
    float getSlipDeg() const;      // slip_deg, degrees of slip
```

Nach `XPLMDataRef dr_roll_vacuum = nullptr;` einfügen:

```cpp
    XPLMDataRef dr_turn_rate = nullptr;
    XPLMDataRef dr_slip = nullptr;
```

- [ ] **Step 2: `DataRefManager.cpp`**

Im Konstruktor nach der `dr_roll_vacuum = XPLMFindDataRef(...)`-Zeile:

```cpp
    // Turn coordinator. The gauge datarefs already follow electrical power in
    // the sim, so no separate bus-voltage flag goes over the wire.
    dr_turn_rate = XPLMFindDataRef("sim/cockpit2/gauges/indicators/turn_rate_roll_deg_pilot");
    dr_slip = XPLMFindDataRef("sim/cockpit2/gauges/indicators/slip_deg");
```

Nach `float DataRefManager::getRollVacuumDeg() const { ... }`:

```cpp
// Turn coordinator
float DataRefManager::getTurnRateDeg() const
{
    return readFloat(dr_turn_rate, 0.0f);
}

float DataRefManager::getSlipDeg() const
{
    return readFloat(dr_slip, 0.0f);
}
```

- [ ] **Step 3: `DCUProvider.h`**

Nach `float attitudeAccumulator_ = 0.0f;`:

```cpp
    float turnCoordinatorAccumulator_ = 0.0f;
```

Nach `static constexpr float ATTITUDE_RATE = 50.0f;    // Hz`:

```cpp
    static constexpr float TURN_COORDINATOR_RATE = 50.0f; // Hz
```

- [ ] **Step 4: `DCUProvider.cpp`**

Direkt nach dem `// ============ Attitude indicator (50 Hz) ============`-Block (nach `attitudeAccumulator_ = 0.0f; }`) einfügen:

```cpp
    // ============ Turn coordinator (50 Hz) ============
    turnCoordinatorAccumulator_ += dt;
    float turnCoordinatorRate = 1.0f / TURN_COORDINATOR_RATE;

    if (turnCoordinatorAccumulator_ >= turnCoordinatorRate)
    {
        TurnCoordinatorMessage tc;
        tc.turnRateDeg = dataRefMgr_->getTurnRateDeg();
        tc.slipDeg = dataRefMgr_->getSlipDeg();

        msgQueue_->enqueueTx(MessageType::SerialMessageTurnCoordinator, &tc, sizeof(tc));

        turnCoordinatorAccumulator_ = 0.0f;
    }
```

- [ ] **Step 5: Build**

Run: `cd DCUProviderPlugin && ./build-macos.sh`
Expected: Build endet ohne Fehler, `mac.xpl` wird erzeugt. (Sandbox-Hinweise zu cmake/clang: Memory `sandbox-toolchain-setup`.)

- [ ] **Step 6: Commit**

```bash
git add DCUProviderPlugin/src/DataRefManager.h DCUProviderPlugin/src/DataRefManager.cpp DCUProviderPlugin/src/DCUProvider.h DCUProviderPlugin/src/DCUProvider.cpp
git commit -m "feat(plugin): stream turn rate and slip to the DCU at 50 Hz

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 7: Dokumentation + Bench-/Rig-Checkliste

**Files:**
- Modify: `CLAUDE.md`
- Create: `TurnCoordinatorCAN/README.md`

- [ ] **Step 1: Root `CLAUDE.md` — Board-Liste**

In der Board-Aufzählung `` `RudderCAN`, `ServoBoard`, `StepperBoard`, `TransponderBoard`, `` → `` `RudderCAN`, `ServoBoard`, `StepperBoard`, `TransponderBoard`, `TurnCoordinatorCAN`, ``.

Den Satz über `StepperBoard`/`GyroDrive` erweitern — ersetzen:

```
  `StepperBoard` still carries the pre-CAN attitude-indicator driver (`GyroDrive`) next to its
  X25 motors; `AttitudeCAN` supersedes that part. `StepperBoard` stays as the nominal fallback
  until the CAN board is confirmed on the rig and keeps driving the X25 gauges regardless.
```

durch:

```
  `StepperBoard` still carries the pre-CAN attitude-indicator driver (`GyroDrive`) next to its
  X25 motors; `AttitudeCAN` supersedes that part, and `TurnCoordinatorCAN` supersedes the two
  X25 channels for turn rate and slip. `StepperBoard` stays as the nominal fallback for both
  until the CAN boards are confirmed on the rig and keeps driving the remaining X25 gauges
  regardless.
```

- [ ] **Step 2: Root `CLAUDE.md` — Test-Liste + Header-Liste**

Im `pio test`-Kommentar:

```
                          # AirspeedCAN, AltimeterCAN, AttitudeCAN, DCU, HSICAN, RudderCAN,
                          # TurnCoordinatorCAN, VerticalSpeedCAN and WhiskeyCompassCAN have real
                          # Unity tests, run natively (no device needed):
```

In der Header-Aufzählung `` `AttitudeGeometry.h`); `` → `` `AttitudeGeometry.h`, `TurnCoordinatorCalibration.h`); ``.

Im Architektur-Absatz nach dem `AttitudeCAN`-Beispiel (endet mit `2026-10-09-attitude-can-design.md`.`) anfügen:

```
  `TurnCoordinatorCAN` is the two-needle X27 variant: CAN `0x103` carries signed turn rate and
  slip, one AX1201728SG drives both motors through the vendored `vid6608` fork, and each needle
  has its own taught calibration table (VSI-style, 0 is a taught mark, not the home stop).
  See `docs/superpowers/specs/2026-10-09-turn-coordinator-can-design.md`.
```

- [ ] **Step 3: `TurnCoordinatorCAN/README.md`**

```markdown
# TurnCoordinatorCAN

Turn coordinator (turn rate needle + slip ball) as a CAN instrument: Arduino Nano, MCP2515,
one AX1201728SG driving two X27.168. Design: `docs/superpowers/specs/2026-10-09-turn-coordinator-can-design.md`.

## Build / test

    pio run -e nano            # firmware (BENCHDEBUG 0 in src/Configuration.h)
    pio test -e native         # TurnCoordinatorCalibration.h unit tests
    pio device monitor -b 115200

## Bench checklist (BENCHDEBUG 1)

1. Power-up: both needles sweep to their home stop, LED blinks three times.
2. `ho` twice — both needles return to the stop, no skipped steps.
3. `cw`, reset the board, `?` — both tables show 0 points, needles sit on the stop.
4. Calibrate turn rate: `ct<deg>` until the aircraft symbol sits on the left mark → `pt-90`
   (or whatever the dial says), centre → `pt0`, right mark → `pt90`. `?` lists the points.
5. Calibrate slip: `cs<deg>` onto the left mark → `ps-8`, centre → `ps0`, right → `ps8`.
6. `tr0 sl0` — both needles on centre. `tr90`, `tr-90`, `sl8`, `sl-8` on their marks.
   `tr200` parks on the outermost taught point, no stop is hit.
7. `co` — both needles follow smoothly, no stutter; `cx` stops.
8. `br0` / `br255`.

Direction is not configured in code: if a needle runs the wrong way, teach the points the
other way round.

## Rig checklist (BENCHDEBUG 0)

1. Gateway heartbeat: LED on; both needles on their taught centre.
2. Right turn in X-Plane: aircraft symbol banks right; slip in a skid moves the ball the
   right way. Wrong direction → re-teach points, no code change.
3. From the DCU bench console, `tc45,-4` moves both needles; `tc` alone keeps the slip.
4. From `CANDebugNode`, send `0x103` with len 2: the board must ignore it (no move).
5. Pull the gateway: LED off; reconnect: LED on, needles follow the next frame.
```

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md TurnCoordinatorCAN/README.md
git commit -m "docs: record TurnCoordinatorCAN in the repo guide with bench and rig checklists

Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

## Abschluss

Nach Task 7: `superpowers:finishing-a-development-branch` (Arbeit liegt bereits auf `main`, also nur Abnahme, kein Merge). Build-Matrix als Abnahme:

```bash
cd TurnCoordinatorCAN && pio run -e nano && pio test -e native
cd ../DCU && pio run -e megaatmega2560 && pio test -e native
cd ../VerticalSpeedCAN && pio run -e nano
cd ../AttitudeCAN && pio run -e nano
cd ../DCUProviderPlugin && ./build-macos.sh
```

Offen bleibt die Hardware-Abnahme (Bench- und Rig-Checkliste im README) — das kann die Sandbox nicht. Nach Bestätigung am Rig: Fallback-Hinweis in `CLAUDE.md` zu `StepperBoard` zurücknehmen.
