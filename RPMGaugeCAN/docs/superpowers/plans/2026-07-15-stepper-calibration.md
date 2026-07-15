# RPM Gauge Stepper Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the RPM gauge's logical needle range (RPM 0..kMaxRPM) be calibrated to the stepper's actual absolute position range via two new BenchDebug commands (`mi`/`ma`), persisted in EEPROM, so the printed dial scale and the motor's absolute coordinate system line up.

**Architecture:** `RPMGauge` gains an EEPROM-backed `Config` struct (magic + version + `minStep`/`maxStep`, in native motor-step units) following the exact pattern already used by `HandbrakeCAN/src/Handbrake.cpp`. `calibrateMin()`/`calibrateMax()` capture the stepper's current absolute position (`vid6608::getPosition()`) into the config and persist it. `moveNeedle()`'s normal (non-`calibration`) path maps RPM linearly onto `[minStep, maxStep]` instead of the hardcoded `[kMinimumDegree*12, kMaximumDegree*12]`. The existing `cl` (direct-degree) calibration path is untouched and stays unclamped, since it's how a new physical endpoint gets driven to before capturing it with `mi`/`ma`. Two new BenchDebug commands wire `calibrateMin()`/`calibrateMax()` to the serial console, following `HandbrakeCAN/src/BenchDebug.cpp`'s existing `mi`/`ma` commands verbatim in naming and feedback style.

**Tech Stack:** PlatformIO/Arduino (C++), `vid6608` stepper library, Arduino `EEPROM.h`.

## Global Constraints

- EEPROM struct: `{ uint32_t magic; uint16_t version; uint16_t minStep; uint16_t maxStep; }`, stored at EEPROM address `0` (RPMGaugeCAN uses EEPROM nowhere else).
- `minStep`/`maxStep` are absolute `vid6608` motor-step positions (12 steps/degree), not degrees — matches `getPosition()`/`moveTo()` units directly, no rounding.
- On invalid/missing EEPROM data (magic or version mismatch): fall back to `minStep = kMinimumDegree * 12`, `maxStep = kMaximumDegree * 12`, and persist that default immediately (same behavior as `Handbrake::loadConfig()`).
- The `cl<degree>` BenchDebug command (`calibration = true` path of `moveNeedle`) stays unclamped and independent of `minStep`/`maxStep` — required to drive to new physical endpoints outside the currently-saved range.
- `mi`/`ma` are blocked while `continuousTestActive` is true, same as the existing `rp`/`cl`/`br`/`od`/`oh` commands.
- No native/host test environment exists in this project (`platformio.ini` only defines `nano` and `diecimilaatmega328` AVR envs) — verification is manual via `pio device monitor` on real hardware, not automated unit tests.
- Follow the per-board file layout from the repo's `CLAUDE.md`: device logic (including its own persistence) lives in `RPMGauge.cpp/h`, BenchDebug only wires serial commands to it.

---

### Task 1: EEPROM-backed calibration in `RPMGauge`

**Files:**
- Modify: `src/RPMGauge.h` (full contents replaced below)
- Modify: `src/RPMGauge.cpp` (full contents replaced below)

**Interfaces:**
- Produces: `RPMGauge::calibrateMin()`, `RPMGauge::calibrateMax()` — both `APIResult`, no args, callable from `BenchDebug` (Task 2).
- Produces: `RPMGauge::moveNeedle(uint16_t rpm, bool calibration = false)` — signature unchanged, internal mapping now uses the persisted config instead of `kMinimumDegree`/`kMaximumDegree`.

- [ ] **Step 1: Replace `src/RPMGauge.h`**

```cpp
#ifndef RPMGAUGE_H
#define RPMGAUGE_H
#include <Arduino.h>
#include <vid6608.h>
#include "Configuration.h"

class RPMGauge
{
public:
    enum APIResult
    {
        success = 0
    };

public:
    RPMGauge();
    ~RPMGauge();

    APIResult moveNeedle(uint16_t rpm, bool calibration = false);
    APIResult setBrightness(uint8_t brightness);
    APIResult calibrateMin();
    APIResult calibrateMax();
    APIResult loop();
    bool isMoving();

private:
    struct Config
    {
        uint32_t magic;
        uint16_t version;
        uint16_t minStep;
        uint16_t maxStep;
    };

    void loadConfig();
    void saveConfig();

    vid6608 *motor;
    Config config;
};

#endif
```

- [ ] **Step 2: Replace `src/RPMGauge.cpp`**

```cpp
#include "RPMGauge.h"
#include "DebugLog.h"
#include <EEPROM.h>

static const uint32_t kRPMGaugeConfigMagic = 0x52474331; // 'R','G','C','1'
static const uint16_t kRPMGaugeConfigVersion = 1;
static const uint16_t kRPMGaugeEepromAddress = 0;

RPMGauge::RPMGauge()
{
    pinMode(kRstPin, OUTPUT);
    digitalWrite(kRstPin, LOW);
    motor = new vid6608(kStepPin, kDirPin, kSteps);

    pinMode(kLightPin, OUTPUT);
    analogWrite(kLightPin, 0);

    for (int i = 0; i < 3; i++)
    {
        delay(250);
        analogWrite(kLightPin, 255);
        delay(500);
        analogWrite(kLightPin, 0);
    }

    digitalWrite(kRstPin, HIGH);
    motor->zero();

    loadConfig();
}

RPMGauge::~RPMGauge()
{
    delete motor;
}

void RPMGauge::loadConfig()
{
    EEPROM.get(kRPMGaugeEepromAddress, config);
    if (config.magic != kRPMGaugeConfigMagic || config.version != kRPMGaugeConfigVersion)
    {
        DEBUGLOG_PRINTLN(F("RPMGauge: No valid EEPROM config, writing defaults"));
        config.magic = kRPMGaugeConfigMagic;
        config.version = kRPMGaugeConfigVersion;
        config.minStep = kMinimumDegree * 12;
        config.maxStep = kMaximumDegree * 12;
        EEPROM.put(kRPMGaugeEepromAddress, config);
    }
    else
    {
        DEBUGLOG_PRINTLN(F("RPMGauge: EEPROM config loaded"));
    }
    DEBUGLOG_PRINT(F("RPMGauge: minStep="));
    DEBUGLOG_PRINT(config.minStep);
    DEBUGLOG_PRINT(F(" maxStep="));
    DEBUGLOG_PRINTLN(config.maxStep);
}

void RPMGauge::saveConfig()
{
    EEPROM.put(kRPMGaugeEepromAddress, config);
    DEBUGLOG_PRINT(F("RPMGauge: config saved — minStep="));
    DEBUGLOG_PRINT(config.minStep);
    DEBUGLOG_PRINT(F(" maxStep="));
    DEBUGLOG_PRINTLN(config.maxStep);
}

RPMGauge::APIResult RPMGauge::calibrateMin()
{
    config.minStep = motor->getPosition();
    saveConfig();
    return success;
}

RPMGauge::APIResult RPMGauge::calibrateMax()
{
    config.maxStep = motor->getPosition();
    saveConfig();
    return success;
}

RPMGauge::APIResult RPMGauge::setBrightness(uint8_t brightness)
{
    analogWrite(kLightPin, brightness);
    return success;
}

RPMGauge::APIResult RPMGauge::moveNeedle(uint16_t rpm, bool calibration)
{
    if (calibration)
    {
        DEBUGLOG_PRINTLN(String(F("Calibrate RPM needle to ")) + String(rpm));
        motor->moveTo(rpm * 12); // 12 steps per degree
        return success;
    }

    if (motor->isMoving())
    {
        return success;
    }

    // Map rpm to absolute motor step, where 0 rpm = config.minStep and kMaxRPM = config.maxStep
    const float ratio = constrain(rpm / kMaxRPM, 0., 1.);
    const uint16_t step = config.minStep + static_cast<uint16_t>(ratio * static_cast<float>(config.maxStep - config.minStep));
    DEBUGLOG_PRINTLN(String(F("Move RPM needle to ")) + String(rpm) + String(F(" adjusted to step ")) + String(step));
    motor->moveTo(step);
    return success;
}

RPMGauge::APIResult RPMGauge::loop()
{
    motor->loop();
    return success;
}

bool RPMGauge::isMoving()
{
    return motor->isMoving();
}
```

- [ ] **Step 3: Build to verify it compiles**

Run: `~/.platformio/penv/bin/pio run -e nano`
Expected: `SUCCESS` — no compiler errors (in particular, no complaints about `EEPROM.h` missing — it's part of the AVR Arduino core, no `lib_deps` entry needed).

- [ ] **Step 4: Commit**

```bash
git add src/RPMGauge.h src/RPMGauge.cpp
git commit -m "$(cat <<'EOF'
feat: persist stepper calibration range in EEPROM

RPMGauge now stores logical min/max absolute motor-step positions in
EEPROM (magic+version+payload, following the HandbrakeCAN pattern) and
maps RPM 0..kMaxRPM onto that range instead of the hardcoded
kMinimumDegree/kMaximumDegree constants.
EOF
)"
```

---

### Task 2: `mi`/`ma` BenchDebug commands

**Files:**
- Modify: `src/BenchDebug.cpp:32-87` (the `handleRPMGaugeInput` function)

**Interfaces:**
- Consumes: `RPMGauge::calibrateMin()`, `RPMGauge::calibrateMax()` from Task 1.

- [ ] **Step 1: Add `mi`/`ma` to the continuous-test blocklist**

In `src/BenchDebug.cpp`, the existing blocklist check (lines 33-41) is:

```cpp
    if (continuousTestActive &&
        (strncmp(command, "rp", 2) == 0 ||
         strncmp(command, "cl", 2) == 0 ||
         strncmp(command, "br", 2) == 0 ||
         strncmp(command, "od", 2) == 0 ||
         strncmp(command, "oh", 2) == 0)) {
        Serial.println(F("Continuous test running. Type 'cx' to stop."));
        return true;
    }
```

Replace it with:

```cpp
    if (continuousTestActive &&
        (strncmp(command, "rp", 2) == 0 ||
         strncmp(command, "cl", 2) == 0 ||
         strncmp(command, "br", 2) == 0 ||
         strncmp(command, "od", 2) == 0 ||
         strncmp(command, "oh", 2) == 0 ||
         strncmp(command, "mi", 2) == 0 ||
         strncmp(command, "ma", 2) == 0)) {
        Serial.println(F("Continuous test running. Type 'cx' to stop."));
        return true;
    }
```

- [ ] **Step 2: Add the `mi`/`ma` command branches**

The existing `cl` branch (lines 56-59) is:

```cpp
    } else if (strncmp(command, "cl", 2) == 0) {
        rpmGauge->moveNeedle(atof(command + 2), true);
        return true;
    } else if (strncmp(command, "br", 2) == 0) {
```

Insert two new branches between them, so it reads:

```cpp
    } else if (strncmp(command, "cl", 2) == 0) {
        rpmGauge->moveNeedle(atof(command + 2), true);
        return true;
    } else if (strncmp(command, "mi", 2) == 0) {
        Serial.println(F("Sampling min position..."));
        rpmGauge->calibrateMin();
        Serial.println(F("Min position calibrated."));
        return true;
    } else if (strncmp(command, "ma", 2) == 0) {
        Serial.println(F("Sampling max position..."));
        rpmGauge->calibrateMax();
        Serial.println(F("Max position calibrated."));
        return true;
    } else if (strncmp(command, "br", 2) == 0) {
```

- [ ] **Step 3: Add `mi`/`ma` to the `?` help text**

The existing help text (lines 75-84) is:

```cpp
    } else if (command[0] == '?') {
        Serial.println(F("RPM Gauge Commands:"));
        Serial.println(F("rp<value>: display given RPM"));
        Serial.println(F("od<value>: display given seconds as odometer value"));
        Serial.println(F("oh<value>: display given hours as odometer value"));
        Serial.println(F("br<0..255>: set light brightness"));
        Serial.println(F("cl<degree>: calibrate needle"));
        Serial.println(F("co<seconds>: start continuous motor/odometer test"));
        Serial.println(F("cx: stop continuous test"));
        return true;
    }
```

Replace it with:

```cpp
    } else if (command[0] == '?') {
        Serial.println(F("RPM Gauge Commands:"));
        Serial.println(F("rp<value>: display given RPM"));
        Serial.println(F("od<value>: display given seconds as odometer value"));
        Serial.println(F("oh<value>: display given hours as odometer value"));
        Serial.println(F("br<0..255>: set light brightness"));
        Serial.println(F("cl<degree>: calibrate needle"));
        Serial.println(F("mi: calibrate logical minimum to current needle position"));
        Serial.println(F("ma: calibrate logical maximum to current needle position"));
        Serial.println(F("co<seconds>: start continuous motor/odometer test"));
        Serial.println(F("cx: stop continuous test"));
        return true;
    }
```

- [ ] **Step 4: Build to verify it compiles**

Run: `~/.platformio/penv/bin/pio run -e nano`
Expected: `SUCCESS`

- [ ] **Step 5: Commit**

```bash
git add src/BenchDebug.cpp
git commit -m "$(cat <<'EOF'
feat: add mi/ma BenchDebug commands for stepper calibration

Wires RPMGauge::calibrateMin()/calibrateMax() to the serial console,
matching HandbrakeCAN's existing mi/ma command naming and feedback.
EOF
)"
```

---

### Task 3: Manual hardware verification

**Files:** none (verification only, no code changes expected).

**Interfaces:**
- Consumes: everything from Task 1 and Task 2, flashed onto real hardware.

- [ ] **Step 1: Flash the board in BenchDebug mode**

Ensure `src/Configuration.h` has `#define BENCHDEBUG 1`, then:

```bash
~/.platformio/penv/bin/pio run -t upload -e nano
~/.platformio/penv/bin/pio device monitor -b 115200
```

- [ ] **Step 2: Verify first-boot default range**

On first boot after flashing (fresh/erased EEPROM), the serial log should print `RPMGauge: No valid EEPROM config, writing defaults` followed by `minStep=0 maxStep=3840` (i.e. `kMinimumDegree*12`/`kMaximumDegree*12`). If the board was previously flashed with unrelated EEPROM content, erase it first (`pio run -t erase -e nano`) to reach this state.

- [ ] **Step 3: Drive to the two physical dial endpoints and calibrate**

Type `cl0` and confirm the needle sits at the mechanical position you want to represent RPM=0 on the physical dial; adjust with further `cl<degree>` values as needed, then type `mi`. Expect: `Sampling min position...` then `Min position calibrated.`, and a `RPMGauge: config saved — minStep=<N> maxStep=<M>` log line with the expected step value.

Repeat with `cl320` (or whatever degree drives the needle to the dial's max mark) then `ma`. Expect the same confirmation pattern with an updated `maxStep`.

- [ ] **Step 4: Verify RPM mapping uses the calibrated range**

Type `rp0` — needle should now land exactly on the dial's printed "0". Type `rp3500` (or `kMaxRPM`'s value) — needle should land exactly on the dial's printed max mark. Type a mid-range value (e.g. `rp1750`) and confirm the needle sits at the visually-correct midpoint between the two calibrated endpoints.

- [ ] **Step 5: Verify `cl` still bypasses calibration**

Type `cl0` again — needle should move to the motor's absolute 0-degree position (not necessarily the calibrated `minStep`, unless they happen to coincide), confirming the calibration path remains unclamped.

- [ ] **Step 6: Verify persistence across reboot**

Power-cycle (or reset) the board. On reboot, confirm the serial log shows `RPMGauge: EEPROM config loaded` (not "writing defaults") with the same `minStep`/`maxStep` values from Step 3, and that `rp0`/`rp3500` still land on the same calibrated positions without re-calibrating.

- [ ] **Step 7: Verify `mi`/`ma` are blocked during the continuous test**

Type `co10` to start the continuous test, then `mi` — expect `Continuous test running. Type 'cx' to stop.` and no calibration change. Type `cx` to stop the test.
