# BenchDebug Continuous Test Mode (co/cx) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `co<seconds>`/`cx` bench-console commands that drive the RPM needle
with random moves and the odometer with a real-time seconds counter, for
unattended burn-in testing.

**Architecture:** `BenchDebug` (the bench-console command dispatcher) owns all
new state and timing, exactly like it already owns `rp`/`od`/`cl`/`br`. The only
change outside `BenchDebug` is one passthrough method, `RPMGauge::isMoving()`,
so `BenchDebug` can detect when a scheduled random move has finished without
reaching into `RPMGauge`'s private motor. The random move itself reuses the
existing calibration path, `RPMGauge::moveNeedle(degree, true)`.

**Tech Stack:** PlatformIO / Arduino (AVR), C (no STL, no heap `String` — see
Global Constraints), `vid6608` motor library, existing `Odometer`/`RPMGauge`
classes.

## Global Constraints

- This project has no automated test framework — `test/` dirs are empty
  scaffolds (per repo `CLAUDE.md`). Verification for every task in this plan is:
  (1) `pio run` must succeed for both `nano` and `diecimilaatmega328` envs, and
  (2) for the final task, manual bench verification on real hardware. There is
  no unit-test step to write or run.
- `BenchDebug` code is heap-allocation-free (see prior session fix: AVR SRAM is
  ~2KB and already tight from the OLED framebuffers — any `String`/`malloc` use
  in this file silently breaks under memory pressure). Do not introduce
  `String`, `new`, or any other heap allocation in `BenchDebug.cpp`. Use the
  existing `char*`/`strncmp`/`atof`/`atoi` style already in that file.
- All new/changed code lives under `#if BENCHDEBUG` (already the case for the
  whole of `BenchDebug.h`/`.cpp`) — this feature does not touch CAN mode.
- Follow the spec exactly:
  `RPMGaugeCAN/docs/superpowers/specs/2026-07-09-benchdebug-continuous-test-design.md`

---

## File Structure

| File | Change |
|---|---|
| `RPMGaugeCAN/src/RPMGauge.h` | Add `bool isMoving();` declaration |
| `RPMGaugeCAN/src/RPMGauge.cpp` | Add `RPMGauge::isMoving()` definition |
| `RPMGaugeCAN/src/BenchDebug.h` | Add 5 new private state members |
| `RPMGaugeCAN/src/BenchDebug.cpp` | Add `co`/`cx` dispatch + blocking check + help text (Task 2); add odometer-tick + random-move scheduler to `loop()` (Task 3) |

No new files. All changes are to the 4 files above.

---

### Task 1: `RPMGauge::isMoving()` passthrough

**Files:**
- Modify: `RPMGaugeCAN/src/RPMGauge.h`
- Modify: `RPMGaugeCAN/src/RPMGauge.cpp`

**Interfaces:**
- Produces: `bool RPMGauge::isMoving()` — returns `true` while the needle motor
  has a scheduled move in progress, `false` when stopped. Task 3 depends on
  this exact signature.

- [ ] **Step 1: Add the declaration**

In `RPMGaugeCAN/src/RPMGauge.h`, change:

```cpp
    APIResult moveNeedle(uint16_t rpm, bool calibration = false);
    APIResult setBrightness(uint8_t brightness);
    APIResult loop();
```

to:

```cpp
    APIResult moveNeedle(uint16_t rpm, bool calibration = false);
    APIResult setBrightness(uint8_t brightness);
    APIResult loop();
    bool isMoving();
```

- [ ] **Step 2: Add the definition**

In `RPMGaugeCAN/src/RPMGauge.cpp`, change the end of the file from:

```cpp
RPMGauge::APIResult RPMGauge::loop()
{
    motor->loop();
    return success;
}
```

to:

```cpp
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

- [ ] **Step 3: Build to verify**

Run: `pio run`
Expected: `SUCCESS` for both `nano` and `diecimilaatmega328` environments, no
new warnings about `isMoving`.

- [ ] **Step 4: Commit**

```bash
git add RPMGaugeCAN/src/RPMGauge.h RPMGaugeCAN/src/RPMGauge.cpp
git commit -m "Add RPMGauge::isMoving() passthrough for bench continuous-test mode"
```

---

### Task 2: `co`/`cx` command dispatch, blocking, and help text

**Files:**
- Modify: `RPMGaugeCAN/src/BenchDebug.h`
- Modify: `RPMGaugeCAN/src/BenchDebug.cpp`

**Interfaces:**
- Consumes: nothing new from Task 1 yet (that's used starting Task 3).
- Produces: private members `continuousTestActive` (bool),
  `continuousTestStartSeconds` (float), `continuousTestElapsedSeconds`
  (uint32_t), `lastSecondTick` (uint32_t), `nextMoveTime` (uint32_t) — Task 3
  reads and updates all five inside `BenchDebug::loop()`.

- [ ] **Step 1: Add new state members**

In `RPMGaugeCAN/src/BenchDebug.h`, change:

```cpp
        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        RPMGauge* rpmGauge;
        Odometer* odometer;
```

to:

```cpp
        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        bool continuousTestActive = false;
        float continuousTestStartSeconds = 0;
        uint32_t continuousTestElapsedSeconds = 0;
        uint32_t lastSecondTick = 0;
        uint32_t nextMoveTime = 0;

        RPMGauge* rpmGauge;
        Odometer* odometer;
```

- [ ] **Step 2: Add the blocking guard, `co`/`cx` branches, and help text**

In `RPMGaugeCAN/src/BenchDebug.cpp`, change the entire `handleRPMGaugeInput`
function from:

```cpp
bool BenchDebug::handleRPMGaugeInput(char* command) {
    if (strncmp(command, "rp", 2) == 0) {
        rpmGauge->moveNeedle(atof(command + 2));
        return true;
    } else if (strncmp(command, "od", 2) == 0) {
        float digits[6];
        odometer->secondsToDigits(atof(command + 2), digits);
        odometer->displayNumber(digits);
        return true;
    } else if (strncmp(command, "cl", 2) == 0) {
        rpmGauge->moveNeedle(atof(command + 2), true);
        return true;
    } else if (strncmp(command, "br", 2) == 0) {
        rpmGauge->setBrightness(static_cast<uint8_t>(atoi(command + 2)));
        return true;
    } else if (command[0] == '?') {
        Serial.println(F("RPM Gauge Commands:"));
        Serial.println(F("rp<value>: display given RPM"));
        Serial.println(F("od<value>: display given seconds as odometer value"));
        Serial.println(F("br<0..255>: set light brightness"));
        Serial.println(F("cl<degree>: calibrate needle"));
        return true;
    }
    return false;
}
```

to:

```cpp
bool BenchDebug::handleRPMGaugeInput(char* command) {
    if (continuousTestActive &&
        (strncmp(command, "rp", 2) == 0 ||
         strncmp(command, "cl", 2) == 0 ||
         strncmp(command, "br", 2) == 0 ||
         strncmp(command, "od", 2) == 0)) {
        Serial.println(F("Continuous test running. Type 'cx' to stop."));
        return true;
    }

    if (strncmp(command, "rp", 2) == 0) {
        rpmGauge->moveNeedle(atof(command + 2));
        return true;
    } else if (strncmp(command, "od", 2) == 0) {
        float digits[6];
        odometer->secondsToDigits(atof(command + 2), digits);
        odometer->displayNumber(digits);
        return true;
    } else if (strncmp(command, "cl", 2) == 0) {
        rpmGauge->moveNeedle(atof(command + 2), true);
        return true;
    } else if (strncmp(command, "br", 2) == 0) {
        rpmGauge->setBrightness(static_cast<uint8_t>(atoi(command + 2)));
        return true;
    } else if (strncmp(command, "co", 2) == 0) {
        continuousTestActive = true;
        continuousTestStartSeconds = atof(command + 2);
        continuousTestElapsedSeconds = 0;
        lastSecondTick = millis();
        nextMoveTime = 0;
        float digits[6];
        odometer->secondsToDigits(continuousTestStartSeconds, digits);
        odometer->displayNumber(digits);
        return true;
    } else if (strncmp(command, "cx", 2) == 0) {
        continuousTestActive = false;
        return true;
    } else if (command[0] == '?') {
        Serial.println(F("RPM Gauge Commands:"));
        Serial.println(F("rp<value>: display given RPM"));
        Serial.println(F("od<value>: display given seconds as odometer value"));
        Serial.println(F("br<0..255>: set light brightness"));
        Serial.println(F("cl<degree>: calibrate needle"));
        Serial.println(F("co<seconds>: start continuous motor/odometer test"));
        Serial.println(F("cx: stop continuous test"));
        return true;
    }
    return false;
}
```

- [ ] **Step 3: Build to verify**

Run: `pio run`
Expected: `SUCCESS` for both environments.

- [ ] **Step 4: Manual smoke check on hardware (bench)**

Flash to the board, open the serial console, and check:
- `co0` → odometer immediately shows `000000`.
- `rp1500` (while `co` still active) → prints
  `Continuous test running. Type 'cx' to stop.`, needle does not move to a new
  RPM-driven position.
- `cx` → no more block message on `rp1500` afterward, `rp1500` moves the
  needle normally.
- `?` → help text includes the new `co<seconds>` and `cx` lines.

Expected: all four behaviors match. (Random moves and the 1×/sec odometer tick
are not wired up yet — that's Task 3. `co` only sets state and displays the
start value once at this point.)

- [ ] **Step 5: Commit**

```bash
git add RPMGaugeCAN/src/BenchDebug.h RPMGaugeCAN/src/BenchDebug.cpp
git commit -m "Add co/cx command dispatch, command blocking, and help text"
```

---

### Task 3: Odometer tick + random needle move scheduler in `loop()`

**Files:**
- Modify: `RPMGaugeCAN/src/BenchDebug.cpp`

**Interfaces:**
- Consumes: `RPMGauge::isMoving()` (Task 1), `RPMGauge::moveNeedle(uint16_t,
  bool)` (existing), `Odometer::secondsToDigits(float, float*)` and
  `Odometer::displayNumber(float*)` (existing), and all five
  `continuousTest*`/`nextMoveTime`/`lastSecondTick` members (Task 2).
- Produces: nothing new — this is the last task, it completes the feature.

- [ ] **Step 1: Add the runtime block to `BenchDebug::loop()`**

In `RPMGaugeCAN/src/BenchDebug.cpp`, change:

```cpp
void BenchDebug::loop()
{
    if (millis() - heartbeat > 1000L)
    {
        heartbeat = millis();
        digitalWrite(kLedPin, heartbeatLedOn ? HIGH : LOW);
        heartbeatLedOn = !heartbeatLedOn;
    }

    handleUserInput();
}
```

to:

```cpp
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
        if (millis() - lastSecondTick >= 1000L)
        {
            lastSecondTick += 1000L;
            continuousTestElapsedSeconds++;
            float digits[6];
            odometer->secondsToDigits(continuousTestStartSeconds + continuousTestElapsedSeconds, digits);
            odometer->displayNumber(digits);
        }

        if (!rpmGauge->isMoving())
        {
            if (nextMoveTime == 0)
            {
                nextMoveTime = millis() + random(500, 2000);
            }
            else if (millis() > nextMoveTime)
            {
                nextMoveTime = 0;
                rpmGauge->moveNeedle(random(0, kMaximumDegree), true);
            }
        }
    }

    handleUserInput();
}
```

- [ ] **Step 2: Build to verify**

Run: `pio run`
Expected: `SUCCESS` for both environments.

- [ ] **Step 3: Manual bench verification (full feature)**

Flash to the board, open the serial console, and check:
- `co0` → odometer starts at `000000` and visibly increments by 1 every real
  second (watch for ~5 seconds: `000000` → `000005`).
- Needle sweeps to a new random position every ~0.5-2 seconds after each move
  completes (matches the reference random-move sample's timing).
- `rp1500`, `cl100`, `br200`, `od30` all print
  `Continuous test running. Type 'cx' to stop.` and have no effect while
  `co` is active.
- `co1000` (re-issued while already active) → odometer immediately jumps to
  show `001000` and resumes counting up from there; needle scheduling
  continues uninterrupted.
- `cx` → odometer stops incrementing (freezes at its current value); needle
  finishes its current move (if any) and stops scheduling new ones.
- After `cx`, `rp1500` works normally again (moves the needle, no block
  message).

Expected: all behaviors match the spec
(`RPMGaugeCAN/docs/superpowers/specs/2026-07-09-benchdebug-continuous-test-design.md`).

- [ ] **Step 4: Commit**

```bash
git add RPMGaugeCAN/src/BenchDebug.cpp
git commit -m "Wire up co continuous-test odometer tick and random needle move"
```
