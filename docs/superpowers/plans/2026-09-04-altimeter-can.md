# AltimeterCAN Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the altimeter off AirManager onto the CAN bus as its own board `AltimeterCAN`, with an EEPROM-taught needle zero and a barometer knob that reaches X-Plane.

**Architecture:** `AltimeterCAN` is a copy of `AltimeterDriver` with `AirManager` stripped and a `CAN` class added in the shape of `VerticalSpeedCAN`. Altitude arrives on the existing `0x102` frame; the baro pot goes out on a new `0x340`, through the DCU, over the USB serial link, into `DataRefManager::setBarometerSetting()`. All new arithmetic lives in Arduino-free headers under `include/` and is covered by native Unity tests.

**Tech Stack:** PlatformIO / Arduino (ATmega328 Nano), MCP2515 via `coryjfowler/mcp_can`, MCP23017 port extender, Unity for native tests, C++/CMake for the X-Plane plugin.

**Spec:** `docs/superpowers/specs/2026-09-04-altimeter-can-design.md`

## Global Constraints

- Board target: `nanoatmega328new`, framework `arduino`, platform `atmelavr`, `monitor_speed = 115200`.
- `platformio.ini` uses the `[avr]` + `extends = avr` shape from `AirspeedCAN`/`VerticalSpeedCAN`, plus an `[env:native]` for the Unity tests.
- Shared library is pulled in with `lib_extra_dirs = ../shared`.
- CAN payloads are **big endian**. DLC is 8 for every frame this plan adds.
- No dynamic allocation in steady state; no `String` in CAN or ISR paths (heap churn there previously froze the CAN link on other AVR nodes).
- Arduino-free logic goes in `include/`, is `inline` in a header, and is tested via `pio test -e native`.
- `Altimeter`'s homing, stepper and axis maths stay behaviourally unchanged. The only edits to them are the ones this plan names explicitly.
- `AltimeterDriver` is not modified and not deleted.
- Commit after every task. Branch is `AltimeterCAN`.

**Pin assignment (final):**

| Pin | Function |
|---|---|
| D2 | Hall 100s, `INPUT_PULLUP`, active LOW |
| D4 | CAN `/INT` |
| D5 | Panel light, PWM |
| D7 | Hall 10k (moved from D10) |
| D8 | Hall 1000s |
| D9 | Flag servo |
| D10 | CAN `/CS` |
| D11 / D12 / D13 | MOSI / MISO / SCK |
| A0 | Barometer pot |
| A4 / A5 | I2C to MCP23017 |

**Hardware prerequisite:** the 10k hall sensor must be rewired from D10 to D7 before the board is flashed. D10 is the AVR `/SS` pin; held low by an input it drops the SPI out of master mode.

---

### Task 1: DCU instrument liveness helper

Pulls heartbeat liveness out of the CAN error table and into two bitmasks. Pure arithmetic, so it gets tests first.

**Files:**
- Create: `DCU/include/InstrumentLiveness.h`
- Create: `DCU/test/test_instrument_liveness/test_instrument_liveness.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces: `kLivenessMaxNodes`, `nodeBit(uint8_t) -> uint16_t`, `instrumentMarkSeen(uint16_t&, uint8_t)`, `instrumentSetAlive(uint16_t&, uint8_t, bool)`, `instrumentIsAlive(uint16_t, uint8_t) -> bool`, `silentInstruments(uint16_t seen, uint16_t alive) -> uint16_t`, `anyKnownInstrumentSilent(uint16_t seen, uint16_t alive) -> bool`.

- [ ] **Step 1: Write the failing test**

Create `DCU/test/test_instrument_liveness/test_instrument_liveness.cpp`:

```cpp
#include <unity.h>
#include "InstrumentLiveness.h"

void setUp(void) {}
void tearDown(void) {}

void test_a_node_that_never_reported_is_not_silent(void) {
    uint16_t seen = 0;
    uint16_t alive = 0;
    TEST_ASSERT_FALSE(anyKnownInstrumentSilent(seen, alive));
    TEST_ASSERT_EQUAL_UINT16(0, silentInstruments(seen, alive));
}

void test_a_node_that_reported_and_is_alive_is_not_silent(void) {
    uint16_t seen = 0;
    uint16_t alive = 0;
    instrumentMarkSeen(seen, 7);
    instrumentSetAlive(alive, 7, true);
    TEST_ASSERT_FALSE(anyKnownInstrumentSilent(seen, alive));
}

void test_a_node_that_reported_and_went_quiet_is_silent(void) {
    uint16_t seen = 0;
    uint16_t alive = 0;
    instrumentMarkSeen(seen, 7);
    instrumentSetAlive(alive, 7, true);
    instrumentSetAlive(alive, 7, false);
    TEST_ASSERT_TRUE(anyKnownInstrumentSilent(seen, alive));
    TEST_ASSERT_EQUAL_UINT16(1u << 7, silentInstruments(seen, alive));
}

void test_one_silent_node_does_not_hide_the_others(void) {
    uint16_t seen = 0;
    uint16_t alive = 0;
    instrumentMarkSeen(seen, 2);
    instrumentMarkSeen(seen, 9);
    instrumentSetAlive(alive, 2, true);
    instrumentSetAlive(alive, 9, false);
    TEST_ASSERT_EQUAL_UINT16(1u << 9, silentInstruments(seen, alive));
}

void test_alive_can_be_asserted_and_cleared_repeatedly(void) {
    uint16_t alive = 0;
    instrumentSetAlive(alive, 3, true);
    TEST_ASSERT_TRUE(instrumentIsAlive(alive, 3));
    instrumentSetAlive(alive, 3, false);
    TEST_ASSERT_FALSE(instrumentIsAlive(alive, 3));
    instrumentSetAlive(alive, 3, true);
    TEST_ASSERT_TRUE(instrumentIsAlive(alive, 3));
}

// A malformed heartbeat could carry any nodeId byte. It must not shift past
// the width of the mask, which is undefined behaviour.
void test_node_ids_past_the_mask_width_are_ignored(void) {
    uint16_t seen = 0;
    uint16_t alive = 0;
    instrumentMarkSeen(seen, 16);
    instrumentMarkSeen(seen, 200);
    instrumentSetAlive(alive, 16, true);
    TEST_ASSERT_EQUAL_UINT16(0, seen);
    TEST_ASSERT_EQUAL_UINT16(0, alive);
    TEST_ASSERT_FALSE(instrumentIsAlive(alive, 16));
    TEST_ASSERT_EQUAL_UINT16(0, nodeBit(16));
}

void test_every_valid_node_gets_its_own_bit(void) {
    for (uint8_t nodeId = 0; nodeId < kLivenessMaxNodes; nodeId++) {
        TEST_ASSERT_EQUAL_UINT16((uint16_t)(1u << nodeId), nodeBit(nodeId));
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_a_node_that_never_reported_is_not_silent);
    RUN_TEST(test_a_node_that_reported_and_is_alive_is_not_silent);
    RUN_TEST(test_a_node_that_reported_and_went_quiet_is_silent);
    RUN_TEST(test_one_silent_node_does_not_hide_the_others);
    RUN_TEST(test_alive_can_be_asserted_and_cleared_repeatedly);
    RUN_TEST(test_node_ids_past_the_mask_width_are_ignored);
    RUN_TEST(test_every_valid_node_gets_its_own_bit);
    return UNITY_END();
}
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd DCU && pio test -e native -f test_instrument_liveness
```

Expected: build failure, `InstrumentLiveness.h: No such file or directory`.

- [ ] **Step 3: Write the header**

Create `DCU/include/InstrumentLiveness.h`:

```cpp
#ifndef INSTRUMENTLIVENESS_H
#define INSTRUMENTLIVENESS_H
#include <stdint.h>

// Liveness of the instrument nodes, held as two bitmasks rather than one entry
// per node in the CAN error table. Bit N is node N.
//
// `seen` is sticky: a node enters it the first time it reports and never leaves.
// `alive` tracks whether its heartbeat is currently fresh. A node the rig does
// not have is in neither mask, which is what keeps a half-populated bus from
// lighting the alarm LED.
//
// Deliberately Arduino-free so the native Unity tests can exercise it.

static const uint8_t kLivenessMaxNodes = 16;

inline uint16_t nodeBit(uint8_t nodeId)
{
    return (nodeId < kLivenessMaxNodes) ? (uint16_t)(1u << nodeId) : (uint16_t)0u;
}

inline void instrumentMarkSeen(uint16_t &seen, uint8_t nodeId)
{
    seen |= nodeBit(nodeId);
}

inline void instrumentSetAlive(uint16_t &alive, uint8_t nodeId, bool isAlive)
{
    const uint16_t bit = nodeBit(nodeId);
    if (isAlive)
    {
        alive |= bit;
    }
    else
    {
        alive &= (uint16_t)~bit;
    }
}

inline bool instrumentIsAlive(uint16_t alive, uint8_t nodeId)
{
    return (alive & nodeBit(nodeId)) != 0;
}

// Nodes that reported at least once and are not reporting now.
inline uint16_t silentInstruments(uint16_t seen, uint16_t alive)
{
    return (uint16_t)(seen & (uint16_t)~alive);
}

inline bool anyKnownInstrumentSilent(uint16_t seen, uint16_t alive)
{
    return silentInstruments(seen, alive) != 0;
}

#endif // INSTRUMENTLIVENESS_H
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
cd DCU && pio test -e native -f test_instrument_liveness
```

Expected: `7 test cases: 7 succeeded`.

- [ ] **Step 5: Run the whole native suite**

```bash
cd DCU && pio test -e native
```

Expected: all suites pass, 36 cases total (29 existing + 7 new).

- [ ] **Step 6: Commit**

```bash
git add DCU/include/InstrumentLiveness.h DCU/test/test_instrument_liveness/
git commit -m "feat(dcu): bitmask helper for instrument liveness"
```

---

### Task 2: Wire liveness into the DCU and shrink the error table

**Files:**
- Modify: `DCU/src/CAN.h` (member declarations, `kMaxCanIdErrors`)
- Modify: `DCU/src/CAN.cpp` (`updateInstrumentHeartbeat`, `checkInstrumentHeartbeats`, `updateAlarmLED`)
- Modify: `DCU/src/BenchDebug.h`, `DCU/src/BenchDebug.cpp` (new `hb` command)

**Interfaces:**
- Consumes: `InstrumentLiveness.h` from Task 1.
- Produces: `CAN::silentInstrumentMask() const -> uint16_t` for BenchDebug.

- [ ] **Step 1: Replace the member declarations**

In `DCU/src/CAN.h`, add the include next to the existing `#include "CanIdError.h"`:

```cpp
#include "InstrumentLiveness.h"
```

Add a public accessor after `void setDCUSender(DCUSender* sender);`:

```cpp
        // Nodes that reported at least once and have since gone quiet.
        // Bit N is node N. Used by BenchDebug and the alarm LED.
        uint16_t silentInstrumentMask() const;
```

Replace this block:

```cpp
        // Instrument heartbeat monitoring (nodeId -> last seen)
        static constexpr uint8_t kMaxInstrumentNodes = 16; // 0..15
        uint32_t lastInstrumentHeartbeatMs[kMaxInstrumentNodes] = {0};
        bool instrumentAlive[kMaxInstrumentNodes] = {false};
```

with:

```cpp
        // Instrument heartbeat monitoring (nodeId -> last seen)
        static constexpr uint8_t kMaxInstrumentNodes = kLivenessMaxNodes; // 0..15
        uint32_t lastInstrumentHeartbeatMs[kMaxInstrumentNodes] = {0};

        // Liveness lives in two bitmasks instead of one canIdErrors[] entry per
        // node. The old scheme keyed those entries on a fake CAN id (0x301 +
        // nodeId), which aliased real bus ids - 0x303 is both the rudder frame
        // and node 2's pseudo-id - and could fill the table on its own.
        uint16_t instrumentSeenMask = 0;
        uint16_t instrumentAliveMask = 0;
```

Then replace the error table comment and size:

```cpp
        // CAN ID error tracking: tracks TX/RX error status per CAN ID.
        // Only real bus ids land here now (8 transmitted ids today), so the
        // table no longer has to absorb one entry per monitored node.
        static constexpr uint8_t kMaxCanIdErrors = 12;
```

- [ ] **Step 2: Mark nodes as seen on arrival**

In `DCU/src/CAN.cpp`, replace the body of `updateInstrumentHeartbeat`:

```cpp
void CAN::updateInstrumentHeartbeat(uint8_t len, const uint8_t *data)
{
    if (len < 8)
        return;

    const uint8_t nodeId = data[0];
    if (nodeId >= kMaxInstrumentNodes)
        return;

    lastInstrumentHeartbeatMs[nodeId] = millis();
    instrumentMarkSeen(instrumentSeenMask, nodeId);
}
```

- [ ] **Step 3: Rewrite the timeout check**

Replace the whole body of `checkInstrumentHeartbeats`:

```cpp
void CAN::checkInstrumentHeartbeats()
{
    const uint32_t now = millis();
    const uint32_t timeoutMs = 1500;

    for (uint8_t nodeId = 0; nodeId < kMaxInstrumentNodes; ++nodeId)
    {
        if (nodeId == static_cast<uint8_t>(fwInfo.nodeId))
            continue; // skip gateway itself

        const bool alive = heartbeatAlive(lastInstrumentHeartbeatMs[nodeId], now, timeoutMs);
        if (alive != instrumentIsAlive(instrumentAliveMask, nodeId))
        {
            instrumentSetAlive(instrumentAliveMask, nodeId, alive);
            DEBUGLOG_PRINTLN(String(F("Instrument HB node ")) + String(nodeId) + (alive ? F(" OK") : F(" TIMEOUT")));
        }
    }
}

uint16_t CAN::silentInstrumentMask() const
{
    return silentInstruments(instrumentSeenMask, instrumentAliveMask);
}
```

A node that has never reported stays out of both masks: `heartbeatAlive()` returns false for `lastSeenMs == 0`, and `instrumentIsAlive()` is false too, so there is no state change and nothing is logged.

- [ ] **Step 4: Fold liveness into the alarm LED**

Replace the `else` branch of `updateAlarmLED`:

```cpp
    else
    {
        ledOn = anyCanIdHasError(canIdErrors, canIdErrorCount) ||
                anyKnownInstrumentSilent(instrumentSeenMask, instrumentAliveMask);
    }
```

- [ ] **Step 5: Add the BenchDebug command**

In `DCU/src/BenchDebug.cpp`, add a branch before the `?` branch in `handleAltimeterInput`:

```cpp
    } else if (command.startsWith("hb")) {
        const uint16_t silent = canBus->silentInstrumentMask();
        if (silent == 0) {
            Serial.println(F("All known instruments alive."));
            return true;
        }
        Serial.print(F("Silent nodes:"));
        for (uint8_t nodeId = 0; nodeId < 16; nodeId++) {
            if (silent & (1u << nodeId)) {
                Serial.print(' ');
                Serial.print(nodeId);
            }
        }
        Serial.println();
        return true;
```

And a help line next to the others:

```cpp
        Serial.println(F("hb: list instrument nodes that went quiet"));
```

- [ ] **Step 6: Build both modes**

```bash
cd DCU && pio run -e megaatmega2560
sed -i '' 's/#define BENCHDEBUG 0/#define BENCHDEBUG 1/' src/Configuration.h
pio run -e megaatmega2560
sed -i '' 's/#define BENCHDEBUG 1/#define BENCHDEBUG 0/' src/Configuration.h
grep -n BENCHDEBUG src/Configuration.h
```

Expected: both builds SUCCESS, and the flag is back to `0`. RAM should drop slightly versus before (16 bytes of bool array and 12 error slots freed, 4 bytes of masks added).

- [ ] **Step 7: Run the native suite**

```bash
cd DCU && pio test -e native
```

Expected: 36 cases pass.

- [ ] **Step 8: Commit**

```bash
git add DCU/src/CAN.h DCU/src/CAN.cpp DCU/src/BenchDebug.h DCU/src/BenchDebug.cpp
git commit -m "refactor(dcu): track instrument liveness in bitmasks, not the error table"
```

---

### Task 3: Shared protocol identifiers

**Files:**
- Modify: `shared/CANBase/include/CanNodeId.h`
- Modify: `shared/CANBase/include/CanMessageId.h`
- Modify: `shared/CANBase/include/SerialMessageId.h`

**Interfaces:**
- Produces: `CanNodeId::altimeterNodeId` (`0x09`), `CanMessageId::altimeterBaro` (`0x340`), `MessageType::SerialMessageBaro` (`0x0A`).

- [ ] **Step 1: Add the node id**

In `shared/CANBase/include/CanNodeId.h`, replace the tail of the enum:

```cpp
  asiNodeId = 0x07,
  vsiNodeId = 0x08,
  altimeterNodeId = 0x09
};
```

The comment above `vsiNodeId` that predicts `0x09` for the altimeter can go — the id is real now.

- [ ] **Step 2: Add the CAN message id**

In `shared/CANBase/include/CanMessageId.h`, add after `handbrakeStatus = 0x330`:

```cpp
  handbrakeStatus = 0x330,

  // 0x340..0x34F: cluster inputs (Instruments -> Gateway, onChange).
  // The DCU covers this whole block with a single range filter (mask 0x7F0),
  // so new clusters in here need no filter change on the gateway.
  //
  // 0x340: Altimeter barometer knob
  // [0..1] baro   uint16, inHg * 100 (sim/cockpit2/gauges/actuators/barometer_setting_in_hg_pilot)
  // [2]    unit   uint8, 0 = hPa, 1 = inHg — AltimeterCAN always sends 1
  // [3..7] reserved
  altimeterBaro = 0x340
};
```

- [ ] **Step 3: Add the serial message type**

In `shared/CANBase/include/SerialMessageId.h`, add after `SerialMessageAltimeterVsi = 0x09,`:

```cpp
    // DCU -> Plugin. Payload: float inHg (4 bytes, host order).
    // Decoded from CAN 0x340, which carries inHg * 100 as a big-endian uint16.
    SerialMessageBaro = 0x0A,
```

- [ ] **Step 4: Verify every existing board still builds**

```bash
for d in AirspeedCAN VerticalSpeedCAN FuelGaugeCAN HandbrakeCAN RPMGaugeCAN RudderCAN TransponderCAN CANDebugNode; do
  [ -d "$d" ] && (cd "$d" && echo "== $d" && pio run -e nano 2>&1 | tail -2)
done
cd DCU && pio run -e megaatmega2560 2>&1 | tail -2
```

Expected: every present project reports SUCCESS. `TransponderCAN` does not exist (the board is `TransponderBoard`) and is skipped by the `-d` guard; build it explicitly if you want it covered.

- [ ] **Step 5: Commit**

```bash
git add shared/CANBase/include/CanNodeId.h shared/CANBase/include/CanMessageId.h shared/CANBase/include/SerialMessageId.h
git commit -m "feat(shared): reserve altimeter node id, 0x340 baro frame and its serial type"
```

---

### Task 4: AltimeterCAN project skeleton

Copies the project, strips AirManager, sets the new pins, and injects `Altimeter` into `BenchDebug` instead of having it construct its own. Ends with a board that builds and behaves exactly like `AltimeterDriver` on the bench — no CAN yet.

**Files:**
- Create: `AltimeterCAN/` (copy of `AltimeterDriver/`)
- Delete: `AltimeterCAN/src/AirManager.cpp`, `AltimeterCAN/src/AirManager.h`, `AltimeterCAN/lib/SiMessagePort/`, `AltimeterCAN/src/CockpitDrivers.code-workspace`
- Modify: `AltimeterCAN/platformio.ini`, `AltimeterCAN/src/Configuration.h`, `AltimeterCAN/src/main.cpp`, `AltimeterCAN/src/BenchDebug.h`, `AltimeterCAN/src/BenchDebug.cpp`, `AltimeterCAN/src/Altimeter.cpp`

**Interfaces:**
- Consumes: `CanNodeId::altimeterNodeId` from Task 3.
- Produces: `kNodeId`, `kCanIntPin`, `kCanCSPin`, `kLightPin`, `kHallPins`, `kDefaultZeroAdjustDegree`, `kDefaultBaroLowRaw`/`kDefaultBaroLowInHg100`/`kDefaultBaroHighRaw`/`kDefaultBaroHighInHg100`, `MASK_EXACT`; `BenchDebug(Altimeter*)`.

- [ ] **Step 1: Copy and strip**

```bash
cp -R AltimeterDriver AltimeterCAN
rm -rf AltimeterCAN/.pio AltimeterCAN/lib/SiMessagePort
rm -f AltimeterCAN/src/AirManager.cpp AltimeterCAN/src/AirManager.h
rm -f AltimeterCAN/src/CockpitDrivers.code-workspace
find AltimeterCAN -type f | sort
```

Note: in a sandboxed shell `cp -R` may fail to copy `AltimeterCAN/.vscode/*` with "Operation not permitted". That is harmless — the directory is a VS Code convenience, and `.gitignore` already excludes everything in it except `extensions.json`. If the empty directory is left behind, ignore it.

- [ ] **Step 2: Rewrite platformio.ini**

Replace `AltimeterCAN/platformio.ini` entirely:

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
    blemasle/MCP23017@^2.0.0
    arduino-libraries/Servo@^1.2.2
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

- [ ] **Step 3: Rewrite Configuration.h**

Replace `AltimeterCAN/src/Configuration.h` entirely:

```cpp
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <CanNodeId.h>

#define BENCHDEBUG 1
#define COUPLED_MODE 0

const CanNodeId kNodeId = CanNodeId::altimeterNodeId;

// CAN (MCP2515 on hardware SPI: D11 MOSI, D12 MISO, D13 SCK).
// /CS sits on D10 deliberately. D10 is the AVR /SS pin and must be an output in
// master mode; making it the chip select means SPI.begin() drives it for us.
// Left unused it would be a floating input, and a low level there drops the SPI
// out of master mode.
const uint8_t kCanIntPin = 4;
const uint8_t kCanCSPin = 10;

// Panel light. D5 is Timer0-backed PWM; the servo library owns Timer1 and takes
// PWM on D9/D10 with it, so D5 is the one that can still dim.
const uint8_t kLightPin = 5;

const uint8_t kMCP23017Address = 0x20;

enum ServoId {
    flagServo = 0,
    servoCount
};

enum AltimeterAxis {
    hundred = 0,
    thousand,
    tenshousand,
    altimeterAxisCount
};

enum RpmKeys {
    maxRpm = 0,
    minRpm,
    rpmKeyCount
};

const uint8_t kServoPins[servoCount] = {
    9
};

// The 10k sensor moved off D10 for the /SS reason above: it is INPUT_PULLUP and
// active LOW, so every pass over its magnet would have killed the CAN link.
const uint8_t kHallPins[altimeterAxisCount] = {
    2, // 100s Hall sensor
    8, // 1000s Hall sensor
    7  // 10ks Hall sensor
};

const uint8_t kPotentiometerPin = A0;

const uint32_t kTotalSteps[altimeterAxisCount] = {
    4096L, 4096L, 4096L
};

// Needle zero adjust now lives in EEPROM (see include/AltimeterCalibration.h).
// These are only the factory defaults, and they are the values the AirManager
// firmware had compiled in, so a freshly flashed board behaves like the old one.
const int16_t kDefaultZeroAdjustDegree[altimeterAxisCount] = {
    5, // 100s
    2, // 1000s
    15 // 10ks
};

// Barometer defaults: full pot travel spans the usual Kollsman range.
const uint16_t kDefaultBaroLowRaw = 0;
const uint16_t kDefaultBaroLowInHg100 = 2810;
const uint16_t kDefaultBaroHighRaw = 1023;
const uint16_t kDefaultBaroHighInHg100 = 3100;

const double kServoAdjustDegree[servoCount] = {
    0. // Flag Servo
};

const double kServoMinimumDegree[servoCount] = {
    0. // Flag Servo
};

const double kServoMaximumDegree[servoCount] = {
    90., // Flag Servo
};

const double kRpmLimits[altimeterAxisCount][rpmKeyCount] = {
    // 100s
    {14.0, 10.0},
    // 1000s
    {14.0, 10.0},
    // 10ks
    {14.0, 10.0}
};

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

#endif // CONFIGURATION_H
```

`kZeroPressure` and `kHundredPercentPressure` are gone on purpose — Task 8 replaces the ratio maths with the calibrated conversion.

- [ ] **Step 4: Rewrite main.cpp**

Replace `AltimeterCAN/src/main.cpp` entirely:

```cpp
#include <Arduino.h>
#include "Configuration.h"
#include "Altimeter.h"
#include "DebugLog.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "CAN.h"
CAN* canBus;
#endif

Altimeter* altimeter;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("Altimeter initializing..."));

  altimeter = new Altimeter();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(altimeter);
  #else
  canBus = new CAN(altimeter);
  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("Altimeter started up"));
  }
  #endif
}

void loop() {
  #if BENCHDEBUG
  benchDebug->loop();
  #else
  canBus->loop();
  #endif
  altimeter->loop();
}
```

This will not compile until Step 5 changes the `BenchDebug` constructor, and until Task 5 adds `CAN` — but `CAN.h` is behind `#if BENCHDEBUG`, so the bench build works after Step 5.

- [ ] **Step 5: Inject Altimeter into BenchDebug and drop the D13 heartbeat LED**

In `AltimeterCAN/src/BenchDebug.h`, change the constructor declaration:

```cpp
        BenchDebug(Altimeter* altimeter);
```

and delete these two members, which drove the onboard LED that now shares its pin with SPI SCK:

```cpp
        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;
```

In `AltimeterCAN/src/BenchDebug.cpp`, replace the top of the file down to the end of the constructor:

```cpp
#include <BenchDebug.h>
#if BENCHDEBUG

// No heartbeat LED here: the Nano's onboard LED is on D13, which is SPI SCK on
// this board. Driving it would fight the CAN link.

BenchDebug::BenchDebug(Altimeter* altimeter)
: altimeter(altimeter)
{
    Serial.begin(115200);
    Serial.println(F("Altimeter BenchDebug"));

    inputBuffer = "";

    altimeter->moveServo(flagServo, 0);

    Serial.println(F("System running!"));
}
```

Delete the `delete altimeter;` line from the destructor — `BenchDebug` no longer owns it:

```cpp
BenchDebug::~BenchDebug()
{
}
```

In `BenchDebug::loop()`, delete the block that toggles `kLedPin`:

```cpp
    if (millis() - heartbeat > 1000L)
    {
        heartbeat = millis();
        digitalWrite(kLedPin, heartbeatLedOn ? HIGH : LOW);
        heartbeatLedOn = !heartbeatLedOn;
    }
```

- [ ] **Step 6: Delete the dead flag block in Altimeter::loop()**

In `AltimeterCAN/src/Altimeter.cpp`, delete the commented-out block at the end of `loop()`:

```cpp
    // double heightInFeet = currentHeightInFeet();
    // if (currentFlagState == Flag::off && heightInFeet < 10000.0)
    // {
    //     setFlag(on);
    // }
    // else if (currentFlagState == Flag::on && heightInFeet >= 10100.0)
    // {
    //     setFlag(off);
    // }
```

The live flag logic is in `moveToHeight()`, which fades the flag out between 9000 and 10000 ft. This block was a second, contradicting design that never ran.

- [ ] **Step 7: Build**

```bash
cd AltimeterCAN && pio run -e nano
```

Expected: SUCCESS. If the linker complains about `kLedPin`, a reference to the deleted heartbeat block survived in `BenchDebug.cpp`.

- [ ] **Step 8: Commit**

```bash
git add AltimeterCAN
git commit -m "feat(altimeter): fork AltimeterDriver into AltimeterCAN without AirManager"
```

---

### Task 5: CAN class — altitude and panel light

**Files:**
- Create: `AltimeterCAN/src/CAN.h`, `AltimeterCAN/src/CAN.cpp`
- Modify: `AltimeterCAN/src/Altimeter.h`, `AltimeterCAN/src/Altimeter.cpp` (add `setBrightness`)

**Interfaces:**
- Consumes: `Altimeter::moveToHeight(double)`, `Altimeter::isHomed`, `kNodeId`, `kCanCSPin`, `kCanIntPin`, `MASK_EXACT`, `CanMessageId::altimeterVsi`, `CanMessageId::lights`.
- Produces: `CAN(Altimeter*)`, `Altimeter::setBrightness(uint8_t)`.

- [ ] **Step 1: Add brightness control to Altimeter**

In `AltimeterCAN/src/Altimeter.h`, add to the public section next to `moveServo`. The result
type is `AltimeterDriveResult` — this class has no `APIResult`, unlike `VerticalSpeedIndicator`:

```cpp
    AltimeterDriveResult setBrightness(uint8_t brightness);
```

In `AltimeterCAN/src/Altimeter.cpp`, add the implementation next to `moveServo`:

```cpp
Altimeter::AltimeterDriveResult Altimeter::setBrightness(uint8_t brightness)
{
    analogWrite(kLightPin, brightness);
    return success;
}
```

And in the constructor, after the `pinMode(kPotentiometerPin, INPUT);` line, add:

```cpp
    pinMode(kLightPin, OUTPUT);
    analogWrite(kLightPin, 0);
```

- [ ] **Step 2: Write CAN.h**

Create `AltimeterCAN/src/CAN.h`:

```cpp
#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <InstrumentCAN.h>
#include "Altimeter.h"
#include <CanMessageId.h>
#include <CanNodeId.h>

class CAN : public InstrumentCAN {
    public:
        CAN(Altimeter* altimeter);

    protected:
        bool instrumentBegin() override;
        void onStartupFail() override;
        void handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t* data) override;
        void onGatewayHeartbeatTimeout() override;
        void onGatewayHeartbeatDiscovered() override;

    private:
        Altimeter* altimeter;
};

#endif
```

- [ ] **Step 3: Write CAN.cpp**

Create `AltimeterCAN/src/CAN.cpp`:

```cpp
#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

CAN::CAN(Altimeter *altimeter)
    : InstrumentCAN(kCanCSPin, kCanIntPin, CANFirmwareInfo{static_cast<uint16_t>(kNodeId), 1, 0}),
      altimeter(altimeter)
{
    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

void CAN::onStartupFail()
{
    DEBUGLOG_PRINTLN(F("CAN startup FAIL"));
    altimeter->setBrightness(0);
}

bool CAN::instrumentBegin()
{
    // Beide RX-Buffer vergleichen alle ID-Bits
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1

    // RXB0: Altimeter/VSI
    canBus->init_Filt(0, 0, CAN_STD_ID(CanMessageId::altimeterVsi));
    canBus->init_Filt(1, 0, CAN_STD_ID(CanMessageId::altimeterVsi));

    // RXB1: Lights und Gateway Heartbeat
    canBus->init_Filt(2, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(3, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(4, 0, CAN_STD_ID(CanMessageId::lights));
    canBus->init_Filt(5, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));

    canBus->setMode(MCP_NORMAL);

    altimeter->setBrightness(0);

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
    case CanMessageId::altimeterVsi:
    {
        if (len >= 4)
        {
            // [0..3] altitude in feet, signed. Bytes 4..5 carry the VSI, which
            // this board ignores — that is VerticalSpeedCAN's half of the frame.
            const int32_t altitudeFt = static_cast<int32_t>(
                (static_cast<uint32_t>(data[0]) << 24) |
                (static_cast<uint32_t>(data[1]) << 16) |
                (static_cast<uint32_t>(data[2]) << 8) |
                static_cast<uint32_t>(data[3]));

            // Before homing the axis positions mean nothing, so a setpoint would
            // send the needles somewhere arbitrary.
            if (altimeter->isHomed)
            {
                altimeter->moveToHeight(static_cast<double>(altitudeFt));
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
            altimeter->setBrightness(pwm);
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
    altimeter->setBrightness(0);
}

void CAN::onGatewayHeartbeatDiscovered()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat OK"));
    altimeter->setBrightness(255);
}
```

- [ ] **Step 4: Build both modes**

```bash
cd AltimeterCAN && pio run -e nano
sed -i '' 's/#define BENCHDEBUG 1/#define BENCHDEBUG 0/' src/Configuration.h
pio run -e nano
sed -i '' 's/#define BENCHDEBUG 0/#define BENCHDEBUG 1/' src/Configuration.h
grep -n BENCHDEBUG src/Configuration.h
```

Expected: both SUCCESS, flag back to `1`.

- [ ] **Step 5: Commit**

```bash
git add AltimeterCAN/src/CAN.h AltimeterCAN/src/CAN.cpp AltimeterCAN/src/Altimeter.h AltimeterCAN/src/Altimeter.cpp
git commit -m "feat(altimeter): receive altitude and panel light over CAN"
```

---

### Task 6: Calibration maths

**Files:**
- Create: `AltimeterCAN/include/AltimeterCalibration.h`
- Create: `AltimeterCAN/test/test_calibration/test_calibration.cpp`

**Interfaces:**
- Produces: `normalizeZeroAdjust(int32_t) -> int16_t`, `accumulateZeroAdjust(int16_t, int32_t) -> int16_t`, `jogDegreesFromPosition(uint32_t position, uint32_t totalSteps) -> int32_t`, `struct BaroCalibrationPoint { uint16_t raw; uint16_t inHg100; }`, `struct BaroCalibration { BaroCalibrationPoint low, high; }`, `baroCalibrationDefaults(BaroCalibration&, uint16_t, uint16_t, uint16_t, uint16_t)`, `baroInHg100(const BaroCalibration&, uint16_t raw) -> uint16_t`.

- [ ] **Step 1: Write the failing test**

Create `AltimeterCAN/test/test_calibration/test_calibration.cpp`:

```cpp
#include <unity.h>
#include "AltimeterCalibration.h"

void setUp(void) {}
void tearDown(void) {}

// --- zero adjust ------------------------------------------------------------

void test_small_values_pass_through_unchanged(void) {
    TEST_ASSERT_EQUAL_INT16(5, normalizeZeroAdjust(5));
    TEST_ASSERT_EQUAL_INT16(-5, normalizeZeroAdjust(-5));
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(0));
}

void test_full_turns_collapse_to_zero(void) {
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(360));
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(-360));
    TEST_ASSERT_EQUAL_INT16(0, normalizeZeroAdjust(3600));
}

void test_values_fold_into_minus180_to_179(void) {
    TEST_ASSERT_EQUAL_INT16(-170, normalizeZeroAdjust(190));
    TEST_ASSERT_EQUAL_INT16(170, normalizeZeroAdjust(-190));
    TEST_ASSERT_EQUAL_INT16(-180, normalizeZeroAdjust(180));
    TEST_ASSERT_EQUAL_INT16(-180, normalizeZeroAdjust(-180));
}

// The point of the whole exercise: after homing the axis sits at 0 with the
// stored offset already applied, so a jog is added to it, not substituted.
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
    // 4096 steps per turn, so a quarter turn is 1024 steps = 90 degrees.
    TEST_ASSERT_EQUAL_INT32(90, jogDegreesFromPosition(1024, 4096));
    TEST_ASSERT_EQUAL_INT32(0, jogDegreesFromPosition(0, 4096));
}

// getPosition() normalises into 0..total-1, so a small backwards jog comes back
// as almost a whole turn. Taking that at face value would store a ~350 degree
// offset for a nudge of ten.
void test_a_backward_jog_reads_as_negative_degrees(void) {
    TEST_ASSERT_EQUAL_INT32(-90, jogDegreesFromPosition(4096 - 1024, 4096));
    TEST_ASSERT_EQUAL_INT32(-1, jogDegreesFromPosition(4096 - 12, 4096));
}

void test_a_zero_step_axis_cannot_divide_by_zero(void) {
    TEST_ASSERT_EQUAL_INT32(0, jogDegreesFromPosition(100, 0));
}

// --- barometer --------------------------------------------------------------

static BaroCalibration defaultBaro(void) {
    BaroCalibration c;
    baroCalibrationDefaults(c, 0, 2810, 1023, 3100);
    return c;
}

void test_the_calibrated_endpoints_map_to_their_own_values(void) {
    BaroCalibration c = defaultBaro();
    TEST_ASSERT_EQUAL_UINT16(2810, baroInHg100(c, 0));
    TEST_ASSERT_EQUAL_UINT16(3100, baroInHg100(c, 1023));
}

void test_the_midpoint_interpolates_linearly(void) {
    BaroCalibration c;
    baroCalibrationDefaults(c, 0, 2800, 1000, 3000);
    TEST_ASSERT_EQUAL_UINT16(2900, baroInHg100(c, 500));
    TEST_ASSERT_EQUAL_UINT16(2850, baroInHg100(c, 250));
}

void test_raw_values_outside_the_endpoints_are_clamped(void) {
    BaroCalibration c;
    baroCalibrationDefaults(c, 100, 2800, 900, 3000);
    TEST_ASSERT_EQUAL_UINT16(2800, baroInHg100(c, 0));
    TEST_ASSERT_EQUAL_UINT16(2800, baroInHg100(c, 100));
    TEST_ASSERT_EQUAL_UINT16(3000, baroInHg100(c, 1023));
}

// A pot wired the other way round gives a falling raw value for rising
// pressure. Calibration should still work rather than clamp everything.
void test_a_reversed_pot_still_interpolates(void) {
    BaroCalibration c;
    baroCalibrationDefaults(c, 1000, 2800, 0, 3000);
    TEST_ASSERT_EQUAL_UINT16(2800, baroInHg100(c, 1000));
    TEST_ASSERT_EQUAL_UINT16(3000, baroInHg100(c, 0));
    TEST_ASSERT_EQUAL_UINT16(2900, baroInHg100(c, 500));
    TEST_ASSERT_EQUAL_UINT16(2800, baroInHg100(c, 1023));
}

// Both points taught at the same pot position would divide by zero.
void test_two_points_at_the_same_raw_value_do_not_divide_by_zero(void) {
    BaroCalibration c;
    baroCalibrationDefaults(c, 500, 2900, 500, 3100);
    TEST_ASSERT_EQUAL_UINT16(2900, baroInHg100(c, 0));
    TEST_ASSERT_EQUAL_UINT16(2900, baroInHg100(c, 500));
    TEST_ASSERT_EQUAL_UINT16(2900, baroInHg100(c, 1023));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_small_values_pass_through_unchanged);
    RUN_TEST(test_full_turns_collapse_to_zero);
    RUN_TEST(test_values_fold_into_minus180_to_179);
    RUN_TEST(test_a_jog_is_added_to_the_stored_offset);
    RUN_TEST(test_repeated_calibration_passes_converge_instead_of_drifting);
    RUN_TEST(test_accumulated_offsets_never_exceed_half_a_turn);
    RUN_TEST(test_a_forward_jog_reads_as_positive_degrees);
    RUN_TEST(test_a_backward_jog_reads_as_negative_degrees);
    RUN_TEST(test_a_zero_step_axis_cannot_divide_by_zero);
    RUN_TEST(test_the_calibrated_endpoints_map_to_their_own_values);
    RUN_TEST(test_the_midpoint_interpolates_linearly);
    RUN_TEST(test_raw_values_outside_the_endpoints_are_clamped);
    RUN_TEST(test_a_reversed_pot_still_interpolates);
    RUN_TEST(test_two_points_at_the_same_raw_value_do_not_divide_by_zero);
    return UNITY_END();
}
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd AltimeterCAN && pio test -e native
```

Expected: build failure, `AltimeterCalibration.h: No such file or directory`.

- [ ] **Step 3: Write the header**

Create `AltimeterCAN/include/AltimeterCalibration.h`:

```cpp
#ifndef ALTIMETERCALIBRATION_H
#define ALTIMETERCALIBRATION_H

#include <stdint.h>

// Calibration maths for AltimeterCAN: the per-axis needle zero offset and the
// barometer pot conversion. Deliberately Arduino-free so both can be exercised
// by the native Unity tests in test/test_calibration (pio test -e native).

// --- Needle zero adjust -----------------------------------------------------

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

// After homing, an axis sits at position 0 with the stored offset already
// applied. Jogging it by `jogDegrees` onto the true mark therefore means the new
// offset is the sum of the two — replacing instead of adding throws away every
// previous pass and makes the needle wander a little further each time.
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

// --- Barometer pot ----------------------------------------------------------

struct BaroCalibrationPoint
{
    uint16_t raw;      // raw ADC reading
    uint16_t inHg100;  // inches of mercury * 100
};

struct BaroCalibration
{
    BaroCalibrationPoint low;
    BaroCalibrationPoint high;
};

inline void baroCalibrationDefaults(BaroCalibration &table,
                                    uint16_t lowRaw, uint16_t lowInHg100,
                                    uint16_t highRaw, uint16_t highInHg100)
{
    table.low.raw = lowRaw;
    table.low.inHg100 = lowInHg100;
    table.high.raw = highRaw;
    table.high.inHg100 = highInHg100;
}

// Linear interpolation between the two taught points, clamped beyond them. The
// pot may be wired either way round, so the direction is taken from the points
// rather than assumed. Two points taught at the same raw value would divide by
// zero and instead park on the low point.
inline uint16_t baroInHg100(const BaroCalibration &table, uint16_t raw)
{
    if (table.high.raw == table.low.raw)
    {
        return table.low.inHg100;
    }

    const float lowRaw = (float)table.low.raw;
    const float highRaw = (float)table.high.raw;
    const float value = (float)raw;
    const bool ascending = highRaw > lowRaw;

    if (ascending ? (value <= lowRaw) : (value >= lowRaw))
    {
        return table.low.inHg100;
    }
    if (ascending ? (value >= highRaw) : (value <= highRaw))
    {
        return table.high.inHg100;
    }

    const float ratio = (value - lowRaw) / (highRaw - lowRaw);
    const float inHg100 = (float)table.low.inHg100 +
                          ratio * ((float)table.high.inHg100 - (float)table.low.inHg100);
    return (uint16_t)(inHg100 + 0.5f);
}

#endif // ALTIMETERCALIBRATION_H
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
cd AltimeterCAN && pio test -e native
```

Expected: `14 test cases: 14 succeeded`.

- [ ] **Step 5: Commit**

```bash
git add AltimeterCAN/include/AltimeterCalibration.h AltimeterCAN/test/test_calibration/
git commit -m "feat(altimeter): calibration maths for needle zero and baro pot"
```

---

### Task 7: EEPROM configuration and the zero-adjust commands

**Files:**
- Modify: `AltimeterCAN/src/Altimeter.h`, `AltimeterCAN/src/Altimeter.cpp`
- Modify: `AltimeterCAN/src/BenchDebug.cpp`

**Interfaces:**
- Consumes: Task 6's header, `kDefaultZeroAdjustDegree`, the `kDefaultBaro*` constants.
- Produces: `Altimeter::calibrateZero(AltimeterAxis) -> bool`, `Altimeter::wipeCalibration()`, `Altimeter::zeroAdjustDegree(AltimeterAxis) const -> int16_t`, `Altimeter::baroCalibration() const -> const BaroCalibration&`, `Altimeter::setBaroCalibrationPoint(bool isHigh, uint16_t inHg100) -> bool`.

- [ ] **Step 1: Declare the config block**

In `AltimeterCAN/src/Altimeter.h`, add the include below the existing ones:

```cpp
#include "AltimeterCalibration.h"
```

Add to the public section, after `wipeCalibration` would sit — next to `homeAxis`:

```cpp
    // Records the current position of `axis` as its true zero and persists it.
    // Must be homed first; the axis is re-zeroed on success.
    bool calibrateZero(AltimeterAxis axis);

    // Factory reset: needle offsets and baro endpoints back to the compiled-in
    // defaults.
    void wipeCalibration();

    int16_t zeroAdjustDegree(AltimeterAxis axis) const { return config.zeroAdjustDegree[axis]; }
    const BaroCalibration &baroCalibration() const { return config.baro; }

    // Stores the pot's current raw reading against `inHg100` as the low or high
    // endpoint. Returns false if the reading is unusable.
    bool setBaroCalibrationPoint(bool isHigh, uint16_t inHg100);
```

Add to the private section, above `MCP23017 *mcp;`:

```cpp
    struct Config
    {
        uint32_t magic;
        uint16_t version;
        int16_t zeroAdjustDegree[altimeterAxisCount];
        BaroCalibration baro;
    };

    void loadConfig();
    void saveConfig();
    void applyConfigDefaults();

    Config config;
```

- [ ] **Step 2: Implement load, save and defaults**

In `AltimeterCAN/src/Altimeter.cpp`, add `#include <EEPROM.h>` under the existing includes, then add these four functions above the constructor:

```cpp
static const uint32_t kAltimeterConfigMagic = 0x414C5431; // 'A','L','T','1'
static const uint16_t kAltimeterConfigVersion = 1;
static const uint16_t kAltimeterEepromAddress = 0;

void Altimeter::applyConfigDefaults()
{
    config.magic = kAltimeterConfigMagic;
    config.version = kAltimeterConfigVersion;
    for (int axis = 0; axis < altimeterAxisCount; axis++)
    {
        config.zeroAdjustDegree[axis] = kDefaultZeroAdjustDegree[axis];
    }
    baroCalibrationDefaults(config.baro,
                            kDefaultBaroLowRaw, kDefaultBaroLowInHg100,
                            kDefaultBaroHighRaw, kDefaultBaroHighInHg100);
}

void Altimeter::loadConfig()
{
    EEPROM.get(kAltimeterEepromAddress, config);
    if (config.magic != kAltimeterConfigMagic || config.version != kAltimeterConfigVersion)
    {
        DEBUGLOG_PRINTLN(String(F("ALT: No valid EEPROM config, writing defaults")));
        applyConfigDefaults();
        EEPROM.put(kAltimeterEepromAddress, config);
    }
    else
    {
        DEBUGLOG_PRINTLN(String(F("ALT: EEPROM config loaded")));
    }
}

void Altimeter::saveConfig()
{
    EEPROM.put(kAltimeterEepromAddress, config);
    DEBUGLOG_PRINTLN(String(F("ALT: config saved")));
}

void Altimeter::wipeCalibration()
{
    applyConfigDefaults();
    saveConfig();
}
```

At the end of the `Altimeter` constructor, just before `instance = this;`, add:

```cpp
    loadConfig();
```

- [ ] **Step 3: Replace the compiled-in zero adjust**

In `AltimeterCAN/src/Altimeter.cpp` there are exactly two uses of `kZeroAdjustDegree`. In `nextHomingState`, case `moveToTrueZero`:

```cpp
        DEBUGLOG_PRINTLN(String(axisName(axis)) + String(F("- Move to adjusted zero position, adjustment degree: ")) + String(config.zeroAdjustDegree[axis]));
        axes[axis]->resetPosition();
        moveDegree(axis, config.zeroAdjustDegree[axis]);
```

and in the synchronous `homeAxis`:

```cpp
    moveDegree(axis, config.zeroAdjustDegree[axis], true);
```

Verify none are left:

```bash
grep -n kZeroAdjustDegree AltimeterCAN/src/*.cpp AltimeterCAN/src/*.h
```

Expected: no output.

- [ ] **Step 4: Implement calibrateZero and the baro endpoint setter**

Add to `AltimeterCAN/src/Altimeter.cpp`, next to `homeAxis`:

```cpp
bool Altimeter::calibrateZero(AltimeterAxis axis)
{
    if (!isHomed)
    {
        DEBUGLOG_PRINTLN(String(F("ALT: not homed, cannot store zero")));
        return false;
    }

    const int32_t jog = jogDegreesFromPosition(axes[axis]->getPosition(),
                                               axes[axis]->getTotalSteps());
    config.zeroAdjustDegree[axis] = accumulateZeroAdjust(config.zeroAdjustDegree[axis], jog);
    saveConfig();

    // The needle is now standing on what we just declared to be zero.
    axes[axis]->resetPosition();
    return true;
}

bool Altimeter::setBaroCalibrationPoint(bool isHigh, uint16_t inHg100)
{
    const uint16_t raw = static_cast<uint16_t>(analogRead(kPotentiometerPin));
    if (isHigh)
    {
        config.baro.high.raw = raw;
        config.baro.high.inHg100 = inHg100;
    }
    else
    {
        config.baro.low.raw = raw;
        config.baro.low.inHg100 = inHg100;
    }
    saveConfig();
    return true;
}
```

- [ ] **Step 5: Add the bench commands**

In `AltimeterCAN/src/BenchDebug.cpp`, add these branches to `handleAltimeterInput` before the `?` branch. All prefixes here are two distinct characters, so the order relative to the existing branches does not matter:

```cpp
    } else if (command.startsWith("zh")) {
        storeZero(hundred, F("100s"));
        return true;
    } else if (command.startsWith("zt")) {
        storeZero(thousand, F("1000s"));
        return true;
    } else if (command.startsWith("ze")) {
        storeZero(tenshousand, F("10ks"));
        return true;
    } else if (command.startsWith("cw")) {
        altimeter->wipeCalibration();
        Serial.println(F("Calibration wiped (needle zeros and baro)."));
        printCalibration();
        return true;
```

Add the two helpers to `BenchDebug.cpp`:

```cpp
void BenchDebug::storeZero(AltimeterAxis axis, const __FlashStringHelper* name)
{
    if (!altimeter->calibrateZero(axis)) {
        Serial.println(F("Not homed. Run 'ho' first."));
        return;
    }
    Serial.print(F("Stored zero for "));
    Serial.print(name);
    Serial.print(F(" axis, offset now "));
    Serial.print(altimeter->zeroAdjustDegree(axis));
    Serial.println(F(" degrees"));
}

void BenchDebug::printCalibration()
{
    Serial.println(F("Calibration:"));
    Serial.print(F("  zero 100s : ")); Serial.println(altimeter->zeroAdjustDegree(hundred));
    Serial.print(F("  zero 1000s: ")); Serial.println(altimeter->zeroAdjustDegree(thousand));
    Serial.print(F("  zero 10ks : ")); Serial.println(altimeter->zeroAdjustDegree(tenshousand));
    const BaroCalibration& baro = altimeter->baroCalibration();
    Serial.print(F("  baro low  : raw ")); Serial.print(baro.low.raw);
    Serial.print(F(" -> ")); Serial.print(baro.low.inHg100 / 100.);
    Serial.println(F(" inHg"));
    Serial.print(F("  baro high : raw ")); Serial.print(baro.high.raw);
    Serial.print(F(" -> ")); Serial.print(baro.high.inHg100 / 100.);
    Serial.println(F(" inHg"));
}
```

Declare them in `AltimeterCAN/src/BenchDebug.h` in the private section:

```cpp
        void storeZero(AltimeterAxis axis, const __FlashStringHelper* name);
        void printCalibration();
```

Extend the `?` help text with:

```cpp
        Serial.println(F("zh / zt / ze: store current position as true zero (100s / 1000s / 10ks)"));
        Serial.println(F("cw: wipe calibration back to defaults"));
```

and call `printCalibration();` at the end of the `?` branch, before `return true;`.

Extend the `st` branch with the calibration dump by adding `printCalibration();` before its `return true;`.

- [ ] **Step 6: Build**

```bash
cd AltimeterCAN && pio run -e nano
```

Expected: SUCCESS.

- [ ] **Step 7: Run the native suite**

```bash
cd AltimeterCAN && pio test -e native
```

Expected: 14 cases pass, unchanged.

- [ ] **Step 8: Commit**

```bash
git add AltimeterCAN/src/Altimeter.h AltimeterCAN/src/Altimeter.cpp AltimeterCAN/src/BenchDebug.h AltimeterCAN/src/BenchDebug.cpp
git commit -m "feat(altimeter): teach the needle zero on the bench and keep it in EEPROM"
```

---

### Task 8: Cooperative homing

**Files:**
- Modify: `AltimeterCAN/src/Altimeter.h`, `AltimeterCAN/src/Altimeter.cpp`
- Modify: `AltimeterCAN/src/CAN.cpp`

**Interfaces:**
- Produces: `Altimeter::beginHoming()`, `Altimeter::isHoming() const -> bool`. `Altimeter::loop()` drives the homing state machine when one is in progress.

- [ ] **Step 1: Declare the new entry points**

In `AltimeterCAN/src/Altimeter.h`, add to the public section next to `homeAllAxis`:

```cpp
    // Starts a homing run that is driven forward by loop(). Unlike homeAllAxis()
    // this returns immediately, so the CAN heartbeat keeps flowing in both
    // directions while the axes search — a blocking run takes several seconds,
    // well past the 1500ms both ends use to declare each other dead.
    void beginHoming();
    bool isHoming() const { return homingActive; }
```

Add to the private section:

```cpp
    bool homingActive = false;
    void runHomingStep();
```

- [ ] **Step 2: Implement the cooperative run**

In `AltimeterCAN/src/Altimeter.cpp`, add next to `homeAllAxis`:

```cpp
void Altimeter::beginHoming()
{
    if (homingActive)
    {
        return;
    }

    stopAllAxes();
    axesCoupled = false;
    isHomed = false;

    for (int axisIndex = 0; axisIndex < altimeterAxisCount; axisIndex++)
    {
        homingState[axisIndex] = unknown;
        nextHomingState(static_cast<AltimeterAxis>(axisIndex));
    }

    homingActive = true;
    DEBUGLOG_PRINTLN(String(F("ALT: homing started")));
}

void Altimeter::runHomingStep()
{
    for (int axisIndex = 0; axisIndex < altimeterAxisCount; axisIndex++)
    {
        AltimeterAxis axis = static_cast<AltimeterAxis>(axisIndex);
        AltimeterDriveResult result = nextHomingState(axis);
        if (result != success)
        {
            DEBUGLOG_PRINTLN(String(F("*** Error homing '")) + errorName(result) + String(F("' on axis ")) + axisName(axis));
            homingActive = false;
            return;
        }
    }

    if (checkAllHomed())
    {
        homingActive = false;
        moveServo(flagServo, kServoMaximumDegree[flagServo]); // Move flag down after homing
        isHomed = true;
#if COUPLED_MODE
        axesCoupled = true;
#endif
        DEBUGLOG_PRINTLN(String(F("ALT: homing complete")));
    }
}
```

- [ ] **Step 3: Drive it from loop()**

In `AltimeterCAN/src/Altimeter.cpp`, at the top of `Altimeter::loop()` after the `instance == NULL` guard, add:

```cpp
    if (homingActive)
    {
        runHomingStep();
    }
```

The existing body of `loop()` already calls `axes[axis]->run(uS)` for every axis and then `sendMotorData()`, which is what actually advances the motors — `runHomingStep()` only moves the state machine along.

- [ ] **Step 4: Trigger it on gateway discovery**

In `AltimeterCAN/src/CAN.cpp`, extend `onGatewayHeartbeatDiscovered`:

```cpp
void CAN::onGatewayHeartbeatDiscovered()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat OK"));
    altimeter->setBrightness(255);

    // First contact with the gateway is what starts the needles: homing before
    // that would drive the instrument with no sim running.
    if (!altimeter->isHomed && !altimeter->isHoming())
    {
        altimeter->beginHoming();
    }
}
```

A gateway timeout deliberately does not abort a run in progress — the axes are mid-search and stopping them there leaves the needles nowhere useful.

- [ ] **Step 5: Build both modes**

```bash
cd AltimeterCAN && pio run -e nano
sed -i '' 's/#define BENCHDEBUG 1/#define BENCHDEBUG 0/' src/Configuration.h
pio run -e nano
sed -i '' 's/#define BENCHDEBUG 0/#define BENCHDEBUG 1/' src/Configuration.h
grep -n BENCHDEBUG src/Configuration.h
```

Expected: both SUCCESS, flag back to `1`.

- [ ] **Step 6: Commit**

```bash
git add AltimeterCAN/src/Altimeter.h AltimeterCAN/src/Altimeter.cpp AltimeterCAN/src/CAN.cpp
git commit -m "feat(altimeter): home cooperatively so the heartbeat keeps running"
```

---

### Task 9: Barometer readout and the 0x340 uplink

**Files:**
- Modify: `AltimeterCAN/src/Altimeter.h`, `AltimeterCAN/src/Altimeter.cpp` (replace `fetchPressureRatio`)
- Modify: `AltimeterCAN/src/CAN.h`, `AltimeterCAN/src/CAN.cpp` (send path)
- Modify: `AltimeterCAN/src/BenchDebug.cpp`, `AltimeterCAN/src/BenchDebug.h` (`bn`, `bx`, `ba`)

**Interfaces:**
- Consumes: `baroInHg100`, `Altimeter::setBaroCalibrationPoint` from Tasks 6 and 7.
- Produces: `Altimeter::baroRaw() const -> uint16_t`, `Altimeter::baroInHg100Now() const -> uint16_t`, `CAN::loop()` override.

- [ ] **Step 1: Replace fetchPressureRatio with the calibrated readout**

In `AltimeterCAN/src/Altimeter.h`, replace the declaration:

```cpp
    float fetchPressureRatio();
```

with:

```cpp
    // Raw ADC reading of the barometer pot, and that reading run through the
    // stored calibration. inHg * 100, matching CAN 0x340.
    uint16_t baroRaw() const;
    uint16_t baroInHg100Now() const;
```

In `AltimeterCAN/src/Altimeter.cpp`, replace the whole `fetchPressureRatio` function:

```cpp
uint16_t Altimeter::baroRaw() const
{
    return static_cast<uint16_t>(analogRead(kPotentiometerPin));
}

uint16_t Altimeter::baroInHg100Now() const
{
    return baroInHg100(config.baro, baroRaw());
}
```

Check nothing else referenced the old name:

```bash
grep -rn fetchPressureRatio AltimeterCAN/src/
```

Expected: no output. If `BenchDebug.h` still declares `uint32_t fetchPressureRatio = 0L;` and `float lastPressureRatio = -1.0f;` as members, delete both — they were AirManager leftovers.

- [ ] **Step 2: Add the send path to CAN**

In `AltimeterCAN/src/CAN.h`, add to the public section:

```cpp
        void loop() override;
```

and to the private section:

```cpp
        void sendBaro(uint16_t inHg100);

        // The pot is polled at 20Hz and only sent on a changed inHg*100 value,
        // with an unconditional refresh every 5s so the sim recovers its baro
        // setting after a DCU or plugin restart without anyone touching the knob.
        static const uint32_t kBaroPollIntervalMs = 50;
        static const uint32_t kBaroRefreshMs = 5000;

        uint32_t lastBaroPollMs = 0;
        uint32_t lastBaroSendMs = 0;
        uint16_t lastBaroInHg100 = 0;
        bool baroSentOnce = false;
```

- [ ] **Step 3: Implement it**

In `AltimeterCAN/src/CAN.cpp`, add:

```cpp
void CAN::loop()
{
    InstrumentCAN::loop();

    const uint32_t now = millis();
    if (now - lastBaroPollMs < kBaroPollIntervalMs)
    {
        return;
    }
    lastBaroPollMs = now;

    const uint16_t inHg100 = altimeter->baroInHg100Now();
    const bool changed = !baroSentOnce || inHg100 != lastBaroInHg100;
    const bool due = (now - lastBaroSendMs) >= kBaroRefreshMs;

    if (changed || due)
    {
        lastBaroInHg100 = inHg100;
        lastBaroSendMs = now;
        baroSentOnce = true;
        sendBaro(inHg100);
    }
}

void CAN::sendBaro(uint16_t inHg100)
{
    // [0..1] inHg * 100 big endian, [2] unit (1 = inHg), [3..7] reserved.
    byte data[8] = {0};
    data[0] = static_cast<uint8_t>(inHg100 >> 8);
    data[1] = static_cast<uint8_t>(inHg100 & 0xFF);
    data[2] = 1;

    sendMessage(static_cast<uint16_t>(CanMessageId::altimeterBaro), 8, data);
}
```

Quantising to whole inHg×100 is what debounces the pot: ADC jitter below 0.01 inHg never produces a frame.

- [ ] **Step 4: Add the bench commands**

In `AltimeterCAN/src/BenchDebug.cpp`, add before the `?` branch:

```cpp
    } else if (command.startsWith("bn")) {
        String rString = command.substring(2);
        rString.trim();
        const uint16_t inHg100 = static_cast<uint16_t>(rString.toFloat() * 100. + 0.5);
        altimeter->setBaroCalibrationPoint(false, inHg100);
        Serial.print(F("Baro low point stored at raw "));
        Serial.println(altimeter->baroCalibration().low.raw);
        return true;
    } else if (command.startsWith("bx")) {
        String rString = command.substring(2);
        rString.trim();
        const uint16_t inHg100 = static_cast<uint16_t>(rString.toFloat() * 100. + 0.5);
        altimeter->setBaroCalibrationPoint(true, inHg100);
        Serial.print(F("Baro high point stored at raw "));
        Serial.println(altimeter->baroCalibration().high.raw);
        return true;
    } else if (command.startsWith("ba")) {
        Serial.print(F("Baro raw "));
        Serial.print(altimeter->baroRaw());
        Serial.print(F(" -> "));
        Serial.print(altimeter->baroInHg100Now() / 100.);
        Serial.println(F(" inHg"));
        return true;
```

Extend the `?` help text:

```cpp
        Serial.println(F("bn<inHg>: store current pot position as the low baro point"));
        Serial.println(F("bx<inHg>: store current pot position as the high baro point"));
        Serial.println(F("ba: show current baro raw value and inHg"));
```

- [ ] **Step 5: Build both modes**

```bash
cd AltimeterCAN && pio run -e nano
sed -i '' 's/#define BENCHDEBUG 1/#define BENCHDEBUG 0/' src/Configuration.h
pio run -e nano
sed -i '' 's/#define BENCHDEBUG 0/#define BENCHDEBUG 1/' src/Configuration.h
grep -n BENCHDEBUG src/Configuration.h
```

Expected: both SUCCESS, flag back to `1`.

- [ ] **Step 6: Run the native suite**

```bash
cd AltimeterCAN && pio test -e native
```

Expected: 14 cases pass.

- [ ] **Step 7: Commit**

```bash
git add AltimeterCAN/src
git commit -m "feat(altimeter): send the calibrated baro setting on CAN 0x340"
```

---

### Task 10: DCU range filters and the baro relay

**Files:**
- Modify: `DCU/src/CAN.h`, `DCU/src/CAN.cpp`
- Modify: `DCU/CLAUDE.md`

**Interfaces:**
- Consumes: `CanMessageId::altimeterBaro`, `MessageType::SerialMessageBaro` from Task 3.
- Produces: a `SerialMessageBaro` frame carrying `float inHg` towards the plugin.

- [ ] **Step 1: Widen the RXB1 mask**

In `DCU/src/CAN.cpp`, replace the filter block in `begin()`:

```cpp
    // RXB0 matches the instrument heartbeat exactly. RXB1 shares one mask across
    // all four of its filters, so it uses a 16-id range mask instead: that is
    // the only way to cover the growing 0x340..0x34F cluster-input block without
    // running out of filter slots. Ids that slip through a neighbouring window
    // land in handleFrame's default branch and cost a few cycles.
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0 exact match
    canBus->init_Mask(1, 0, MASK_RANGE); // RXB1 matches the high 7 id bits

    uint32_t instrumentHeartbeat = CAN_STD_ID(CanMessageId::instrumentHeartbeat);

    // RXB0: Instrument heartbeat
    canBus->init_Filt(0, 0, instrumentHeartbeat);
    canBus->init_Filt(1, 0, instrumentHeartbeat);

    // RXB1: instrument inputs, one filter per 16-id window.
    canBus->init_Filt(2, 0, CAN_STD_ID(CanMessageId::transponderInput)); // 0x310..0x31F
    canBus->init_Filt(3, 0, CAN_STD_ID(CanMessageId::handbrakeStatus));  // 0x330..0x33F
    canBus->init_Filt(4, 0, CAN_STD_ID(CanMessageId::rudder));           // 0x300..0x30F
    canBus->init_Filt(5, 0, CAN_STD_ID(CanMessageId::altimeterBaro));    // 0x340..0x34F
```

In `DCU/src/Configuration.h`, add next to `MASK_EXACT`:

```cpp
// Range match: compares the top 7 of the 11 id bits, so one filter covers a
// block of 16 consecutive ids. Same shape MotionGateway uses for its actors.
const uint32_t MASK_RANGE = 0x07F00000;
```

Note the shift: `CAN_STD_ID` moves the 11-bit id up by 16, so `MASK_EXACT` is `0x07FF0000` and the range mask that ignores the low 4 id bits is `0x07F00000`.

- [ ] **Step 2: Declare and route the baro frame**

In `DCU/src/CAN.h`, add to the private handler list next to `updateRudder`:

```cpp
        void updateBaro(uint8_t len, const uint8_t* data);
```

`handleFrame` switches on `static_cast<CanMessageId>(id)`. Add a case next to the existing
`CanMessageId::rudder` one:

```cpp
    case CanMessageId::rudder:
        updateRudder(len, data);
        break;
    case CanMessageId::altimeterBaro:
        updateBaro(len, data);
        break;
```

- [ ] **Step 3: Implement the relay**

Add to `DCU/src/CAN.cpp`, next to `updateRudder`:

```cpp
void CAN::updateBaro(uint8_t len, const uint8_t *data)
{
    // CAN 0x340: [0..1] inHg * 100 big endian, [2] unit (1 = inHg).
    // The serial side carries a plain float in host order, like the other
    // plugin-facing payloads.
    if (len < 3)
        return;

    const uint16_t inHg100 = unpackBE16(data + 0);
    const float inHg = static_cast<float>(inHg100) / 100.0f;

    if (dcuSender != nullptr)
    {
        DEBUGLOG_PRINTLN(String(F("Send Baro: ")) + String(inHg, 2) + String(F(" inHg")));
        dcuSender->sendFrame(MessageType::SerialMessageBaro,
                             sizeof(float),
                             reinterpret_cast<const uint8_t *>(&inHg));
    }
}
```

The unit byte is read but not acted on: `AltimeterCAN` always sends inHg, and the plugin's dataref is in inHg. If a future cluster sends hPa it will need a conversion here.

- [ ] **Step 4: Update the DCU docs**

In `DCU/CLAUDE.md`, in the "CAN side" section, replace the sentence about filters:

```markdown
- Filters/decodes `transponderInput` (0x311), `handbrakeStatus` (0x330), `rudder` (0x303) and the
  cluster-input block `0x340`–`0x34F` (currently just `altimeterBaro`, 0x340) from instruments and
  forwards them to the plugin via `DCUSender`. RXB0 matches the instrument heartbeat exactly; RXB1
  uses the range mask `MASK_RANGE` so each of its four filters covers 16 consecutive ids — that is
  what lets new cluster inputs arrive without touching the gateway.
```

- [ ] **Step 5: Build and test**

```bash
cd DCU && pio run -e megaatmega2560 && pio test -e native
```

Expected: build SUCCESS, 36 test cases pass.

- [ ] **Step 6: Commit**

```bash
git add DCU/src/CAN.h DCU/src/CAN.cpp DCU/src/Configuration.h DCU/CLAUDE.md
git commit -m "feat(dcu): range-filter the cluster inputs and relay the baro setting"
```

---

### Task 11: Plugin uplink into the barometer dataref

**Files:**
- Modify: `DCUProviderPlugin/src/DCUProvider.cpp` (`updateUplink` switch)
- Modify: `DCUProviderPlugin/CLAUDE.md`

**Interfaces:**
- Consumes: `MessageType::SerialMessageBaro` from Task 3, `DataRefManager::setBarometerSetting(float)` which already exists.

- [ ] **Step 1: Add the uplink case**

In `DCUProviderPlugin/src/DCUProvider.cpp`, in the `switch (msg->type)` inside `updateUplink()`, add after the `SerialMessageHandbrake` case:

```cpp
        case MessageType::SerialMessageBaro:
            // Gateway → Plugin: barometer knob on the altimeter (CAN 0x340)
            if (msg->payload.size() >= sizeof(float))
            {
                float inHg = 0.0f;
                std::memcpy(&inHg, msg->payload.data(), sizeof(float));
                dataRefMgr_->setBarometerSetting(inHg);
            }
            break;
```

`memcpy` rather than a `reinterpret_cast` deref: the payload buffer has no alignment guarantee, and a misaligned float read is undefined behaviour even where x86 tolerates it. Add `#include <cstring>` at the top of the file if it is not already there.

- [ ] **Step 2: Update the plugin docs**

In `DCUProviderPlugin/CLAUDE.md`, in the data-flow list, extend item 4:

```markdown
4. `DCUProvider::updateUplink()` — gateway → X-Plane. Drains `MessageQueue` RX, `switch`es on
   `MessageType`, writes into `DataRefManager` setters. Covers the transponder head, the handbrake,
   the rudder axes and the altimeter's barometer knob (`SerialMessageBaro`).
```

- [ ] **Step 3: Build for macOS**

```bash
cd DCUProviderPlugin && ./build-macos.sh 2>&1 | tail -5
```

Expected: `BUILD SUCCESSFUL`. Note the script installs the plugin into the configured X-Plane folder as a side effect.

- [ ] **Step 4: Cross-build for Windows**

```bash
cd DCUProviderPlugin && ./build-xc-windows.sh 2>&1 | tail -5
```

Expected: `BUILD SUCCESSFUL!` and a PE32+ DLL.

- [ ] **Step 5: Commit**

```bash
git add DCUProviderPlugin/src/DCUProvider.cpp DCUProviderPlugin/CLAUDE.md
git commit -m "feat(plugin): apply the altimeter baro setting to X-Plane"
```

---

### Task 12: Repo documentation

**Files:**
- Modify: `CLAUDE.md`

- [ ] **Step 1: List the new board**

In the top-level `CLAUDE.md`, add `AltimeterCAN` to the board-project list, keeping alphabetical order:

```markdown
- Board projects (`AirspeedCAN`, `AltimeterCAN`, `AltimeterDriver`, `CANDebugNode`, `DCU`, `FuelGaugeCAN`, `HSIDriver`,
```

- [ ] **Step 2: List the new tested header**

In the same file, extend the sentence about natively tested headers:

```markdown
`include/` (`AdaptiveFilter.h`, `AxisMapping.h`, `AirspeedCalibration.h`, `VerticalSpeedCalibration.h`,
`AltimeterCalibration.h`); `src/` stays Arduino-coupled.
```

- [ ] **Step 3: Note that AltimeterDriver is superseded**

Add a line after the board list:

```markdown
`AltimeterDriver` is the pre-CAN altimeter firmware, driven over USB by AirManager. It is superseded by
`AltimeterCAN` and kept only as a fallback until the CAN board is confirmed on the rig.
```

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: register AltimeterCAN in the repo guide"
```

---

## Rig verification (manual, after Task 12)

Not automatable — run these on the hardware before calling the port done.

1. **Rewire** the 10k hall sensor from D10 to D7. Confirm with `ho` in bench mode that the 10k axis still finds its magnet.
2. **Bench, `BENCHDEBUG 1`:** `ho`, then per axis `hu`/`th`/`te` onto the zero mark, then `zh`/`zt`/`ze`. Power-cycle and re-home: the needles must land on the marks without further jogging.
3. **Bench:** pot to one stop, `bn28.10`; to the other, `bx31.00`. `ba` at several positions should track the dial.
4. **Bus, `BENCHDEBUG 0` on the board, `BENCHDEBUG 1` on the DCU:** `al5280` on the DCU console must move the needles to 5,280 ft. `hb` on the DCU must list node 9 as alive, and as silent after unplugging the board.
5. **Sim:** with the plugin loaded, altitude tracks the aircraft and turning the physical baro knob moves the Kollsman window in X-Plane.
