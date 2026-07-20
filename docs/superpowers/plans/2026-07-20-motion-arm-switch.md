# Motion Arm Switch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the software ARM button in MotionProviderPlugin with a physical arm switch on MotionGateway, reported over USB as a periodic heartbeat that drives the plugin's arm state directly.

**Architecture:** MotionGateway samples a GPIO switch and sends a 4-byte `'H''B' armed CR` heartbeat over USB every 500ms. MotionProviderPlugin gains a serial RX path (new thread + pure `HeartbeatDecoder`) and a pure `ArmGate` latch; the flight loop arms `ArmRamp` when the switch is closed and the signal is fresh, and force-disarms (latched until the switch is cycled) on fault or manual e-stop. The status window drops ARM/MANUAL buttons for a SIM/MANUAL mode toggle plus a DISARM e-stop, and shows arm state as text only.

**Tech Stack:** Arduino/AVR C++ (MotionGateway, PlatformIO), C++17 (MotionProviderPlugin, CMake), X-Plane SDK.

## Global Constraints

- MotionGateway arm pin: `kArmPin = 26`, `INPUT_PULLUP`, switch between pin and GND; `armed == (digitalRead(kArmPin) == LOW)`.
- Heartbeat send period: `kUsbHeartbeatIntervalMs = 500` (MotionGateway `Configuration.h`).
- Heartbeat wire frame: exactly 4 bytes — `'H'`, `'B'`, `<armed: 0x00|0x01>`, `0x0D`.
- Plugin freshness threshold: `1.5` seconds (3× the 500ms period). Named `kHeartbeatMaxAgeSec`.
- Heartbeat is its own protocol on the Motion USB link — do **not** add it to `shared/CANBase/include/SerialMessageId.h`.
- Plugin native tests use the repo's hand-rolled harness: `check(bool, const char*)`, `main()` returns non-zero on any failure. No external test framework.
- SIM/MANUAL mode toggle is only permitted while `ArmState::Disarmed`.
- Fault or manual e-stop latches disarm; the latch clears only when the hardware switch transitions armed→disarmed (cycled off).

---

## File Structure

MotionGateway (AVR firmware, PlatformIO):
- `MotionGateway/src/Configuration.h` — add `kArmPin`, `kUsbHeartbeatIntervalMs`.
- `MotionGateway/src/MotionGateway.h` — add `sendUsbHeartbeat()` + heartbeat timestamp member.
- `MotionGateway/src/MotionGateway.cpp` — arm-pin `pinMode`, periodic heartbeat send.

MotionProviderPlugin (C++17, CMake):
- `MotionProviderPlugin/src/HeartbeatDecoder.h` / `.cpp` — **new** pure RX frame decoder.
- `MotionProviderPlugin/src/ArmGate.h` — **new** pure header-only arm latch.
- `MotionProviderPlugin/src/ArmRamp.h` — add `requestArm()`.
- `MotionProviderPlugin/src/SerialLink.h` / `.cpp` — add RX thread + heartbeat accessors.
- `MotionProviderPlugin/src/MotionProvider.h` / `.cpp` — hardware-driven arm wiring, e-stop latch, mode toggle handling.
- `MotionProviderPlugin/src/StatusData.h` — `UiAction` enum change.
- `MotionProviderPlugin/src/StatusWindow.cpp` — UI overhaul.
- `MotionProviderPlugin/CMakeLists.txt` — add `HeartbeatDecoder.cpp` + new headers to `SOURCES`/`HEADERS`.
- `MotionProviderPlugin/tests/test_heartbeat.cpp` / `test_armramp.cpp` / `test_armgate.cpp` — **new** unit tests.
- `MotionProviderPlugin/tests/CMakeLists.txt` — register the new tests.

---

## Task 1: MotionGateway arm switch + USB heartbeat

**Files:**
- Modify: `MotionGateway/src/Configuration.h`
- Modify: `MotionGateway/src/MotionGateway.h`
- Modify: `MotionGateway/src/MotionGateway.cpp`

**Interfaces:**
- Consumes: nothing (leaf firmware change).
- Produces: the on-wire 4-byte heartbeat `'H' 'B' <armed 0x00|0x01> 0x0D`, emitted every 500ms — consumed by Task 2 (`HeartbeatDecoder`) and Task 5 (`SerialLink` RX).

This is AVR firmware; there is no native unit-test harness for MotionGateway (its `test/` is an empty scaffold per the repo CLAUDE.md). Verification is a compile with PlatformIO plus a bench check.

- [ ] **Step 1: Add configuration constants**

In `MotionGateway/src/Configuration.h`, after the existing mode-pin block (`kMode1Pin`/`kMode2Pin`, currently lines 16-17), add:

```cpp
const uint8_t kArmPin = 26; // Arm switch: switch between kArmPin and GND, INPUT_PULLUP

const uint32_t kUsbHeartbeatIntervalMs = 500L; // Arm heartbeat period to MotionProviderPlugin
```

- [ ] **Step 2: Declare the heartbeat sender and its timer**

In `MotionGateway/src/MotionGateway.h`, add the private method declaration alongside the other `send*` methods (near `sendHome()`/`sendStop()`):

```cpp
        void sendUsbHeartbeat();
```

And add the timestamp member alongside `lastModeCheckTimestampMs` / `lastDemandBatchSendTimestampMs`:

```cpp
        unsigned long lastUsbHeartbeatTimestampMs = 0;
```

- [ ] **Step 3: Configure the arm pin in the constructor**

In `MotionGateway/src/MotionGateway.cpp`, in the `MotionGateway` constructor, next to the existing `pinMode(kMode1Pin, INPUT_PULLUP); pinMode(kMode2Pin, INPUT_PULLUP);` calls, add:

```cpp
  pinMode(kArmPin, INPUT_PULLUP);
```

Then, next to the existing `lastModeCheckTimestampMs = millis() - 200;` initialization, add (fire the first heartbeat one interval after boot):

```cpp
  lastUsbHeartbeatTimestampMs = millis();
```

- [ ] **Step 4: Send the heartbeat periodically from loop()**

In `MotionGateway/src/MotionGateway.cpp`, in `MotionGateway::loop()`, after the `checkMaxAgeResync();` call and before `handleSerialInput();`, add:

```cpp
  if ((now - lastUsbHeartbeatTimestampMs) >= kUsbHeartbeatIntervalMs)
  {
    lastUsbHeartbeatTimestampMs = now;
    sendUsbHeartbeat();
  }
```

(`now` is already defined at the top of `loop()` as `const unsigned long now = millis();`.)

- [ ] **Step 5: Implement sendUsbHeartbeat()**

In `MotionGateway/src/MotionGateway.cpp`, add the definition (e.g. after `sendStop()`):

```cpp
void MotionGateway::sendUsbHeartbeat()
{
  const uint8_t armed = (digitalRead(kArmPin) == LOW) ? 0x01 : 0x00;
  const uint8_t frame[4] = {'H', 'B', armed, 0x0D};
  Serial.write(frame, sizeof(frame));
}
```

- [ ] **Step 6: Build the firmware**

Run: `cd MotionGateway && ~/.platformio/penv/bin/pio run`
Expected: build succeeds ("SUCCESS"). No new warnings about `kArmPin`/`sendUsbHeartbeat`.

- [ ] **Step 7: Commit**

```bash
git add MotionGateway/src/Configuration.h MotionGateway/src/MotionGateway.h MotionGateway/src/MotionGateway.cpp
git commit -m "feat(MotionGateway): send arm-switch state as USB heartbeat"
```

---

## Task 2: HeartbeatDecoder (plugin RX frame parser)

**Files:**
- Create: `MotionProviderPlugin/src/HeartbeatDecoder.h`
- Create: `MotionProviderPlugin/src/HeartbeatDecoder.cpp`
- Test: `MotionProviderPlugin/tests/test_heartbeat.cpp`
- Modify: `MotionProviderPlugin/tests/CMakeLists.txt`
- Modify: `MotionProviderPlugin/CMakeLists.txt`

**Interfaces:**
- Consumes: the 4-byte wire frame from Task 1.
- Produces: `class HeartbeatDecoder { bool feed(uint8_t b); bool armed() const; void reset(); };` — `feed` returns `true` exactly on the byte that completes a valid frame; `armed()` reads the last decoded payload. Consumed by Task 5.

- [ ] **Step 1: Write the failing test**

Create `MotionProviderPlugin/tests/test_heartbeat.cpp`:

```cpp
#include "HeartbeatDecoder.h"
#include <cstdio>
#include <cstdint>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }

// Feed a byte sequence; return how many complete frames were decoded, and the
// last armed value via out-param.
static int feedAll(HeartbeatDecoder& d, const uint8_t* b, int n, bool& lastArmed) {
    int frames = 0;
    for (int i = 0; i < n; ++i) {
        if (d.feed(b[i])) { ++frames; lastArmed = d.armed(); }
    }
    return frames;
}

int main() {
    // Valid armed frame.
    {
        HeartbeatDecoder d;
        const uint8_t f[] = {'H','B',0x01,0x0D};
        bool armed = false;
        int n = feedAll(d, f, 4, armed);
        check(n == 1, "armed frame decodes exactly one frame");
        check(armed == true, "armed frame -> armed() true");
    }
    // Valid disarmed frame.
    {
        HeartbeatDecoder d;
        const uint8_t f[] = {'H','B',0x00,0x0D};
        bool armed = true;
        int n = feedAll(d, f, 4, armed);
        check(n == 1, "disarmed frame decodes one frame");
        check(armed == false, "disarmed frame -> armed() false");
    }
    // Leading garbage then a valid frame.
    {
        HeartbeatDecoder d;
        const uint8_t f[] = {0x00,0xFF,'X','B','H','B',0x01,0x0D};
        bool armed = false;
        int n = feedAll(d, f, 8, armed);
        check(n == 1, "garbage-prefixed frame still decodes");
        check(armed == true, "garbage-prefixed frame -> armed true");
    }
    // Truncated frame (no CR) followed by a fresh valid frame.
    {
        HeartbeatDecoder d;
        const uint8_t f[] = {'H','B',0x01, /*no CR*/ 'H','B',0x00,0x0D};
        bool armed = true;
        int n = feedAll(d, f, 7, armed);
        check(n == 1, "truncated then fresh frame decodes once");
        check(armed == false, "recovered frame -> armed false");
    }
    // Wrong terminator byte does not complete a frame.
    {
        HeartbeatDecoder d;
        const uint8_t f[] = {'H','B',0x01,0x00};  // 0x00 instead of CR
        bool armed = false;
        int n = feedAll(d, f, 4, armed);
        check(n == 0, "wrong terminator -> no frame");
    }
    // Two back-to-back frames.
    {
        HeartbeatDecoder d;
        const uint8_t f[] = {'H','B',0x01,0x0D,'H','B',0x00,0x0D};
        bool armed = true;
        int n = feedAll(d, f, 8, armed);
        check(n == 2, "two frames decode as two");
        check(armed == false, "last frame armed value wins");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
```

- [ ] **Step 2: Register the test in CMake and confirm it fails to build**

In `MotionProviderPlugin/tests/CMakeLists.txt`, add after the `test_monitor` executable block (after line 26):

```cmake
add_executable(test_heartbeat test_heartbeat.cpp ../src/HeartbeatDecoder.cpp)
target_include_directories(test_heartbeat PRIVATE ../src)
```

And in the `enable_testing()` block, after the `monitor` test line:

```cmake
add_test(NAME heartbeat COMMAND test_heartbeat)
```

Run: `cmake -S MotionProviderPlugin/tests -B MotionProviderPlugin/tests/build && cmake --build MotionProviderPlugin/tests/build --target test_heartbeat`
Expected: FAIL — `HeartbeatDecoder.h` not found / `HeartbeatDecoder.cpp` missing.

- [ ] **Step 3: Write the header**

Create `MotionProviderPlugin/src/HeartbeatDecoder.h`:

```cpp
#pragma once
#include <cstdint>

// Decodes the MotionGateway arm heartbeat: 'H' 'B' <armed 0x00|0x01> CR.
// Pure byte-in/frame-out state machine, no threads or I/O — unit-testable.
class HeartbeatDecoder {
public:
    // Feed one received byte. Returns true exactly on the byte that completes a
    // valid frame; call armed() afterwards to read the payload.
    bool feed(uint8_t b);

    // Last decoded armed value (meaningful only right after feed() returns true).
    bool armed() const { return armed_; }

    // Discard any in-progress frame (e.g. on reconnect).
    void reset();

private:
    enum class S : uint8_t { SyncH, SyncB, Payload, CR };
    S       state_   = S::SyncH;
    uint8_t pending_ = 0;
    bool    armed_   = false;
};
```

- [ ] **Step 4: Write the implementation**

Create `MotionProviderPlugin/src/HeartbeatDecoder.cpp`:

```cpp
#include "HeartbeatDecoder.h"

void HeartbeatDecoder::reset() {
    state_ = S::SyncH;
    pending_ = 0;
}

bool HeartbeatDecoder::feed(uint8_t b) {
    switch (state_) {
        case S::SyncH:
            state_ = (b == 'H') ? S::SyncB : S::SyncH;
            return false;
        case S::SyncB:
            if (b == 'B')      state_ = S::Payload;
            else               state_ = (b == 'H') ? S::SyncB : S::SyncH;
            return false;
        case S::Payload:
            pending_ = b;
            state_ = S::CR;
            return false;
        case S::CR:
            if (b == 0x0D) {
                armed_ = (pending_ != 0);
                state_ = S::SyncH;
                return true;
            }
            state_ = (b == 'H') ? S::SyncB : S::SyncH;
            return false;
    }
    return false;
}
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `cmake --build MotionProviderPlugin/tests/build --target test_heartbeat && ./MotionProviderPlugin/tests/build/test_heartbeat`
Expected: PASS — final line `13 checks, 0 failures`, exit code 0.

- [ ] **Step 6: Add HeartbeatDecoder to the plugin build**

In `MotionProviderPlugin/CMakeLists.txt`, in the `set(SOURCES ...)` list, after `src/BffEncoder.cpp` (line 57), add:

```cmake
    src/HeartbeatDecoder.cpp
```

And in the `HEADERS` list, after `src/BffEncoder.h`, add:

```cmake
    src/HeartbeatDecoder.h
```

- [ ] **Step 7: Commit**

```bash
git add MotionProviderPlugin/src/HeartbeatDecoder.h MotionProviderPlugin/src/HeartbeatDecoder.cpp MotionProviderPlugin/tests/test_heartbeat.cpp MotionProviderPlugin/tests/CMakeLists.txt MotionProviderPlugin/CMakeLists.txt
git commit -m "feat(MotionProvider): add HeartbeatDecoder for arm heartbeat RX"
```

---

## Task 3: ArmRamp::requestArm()

**Files:**
- Modify: `MotionProviderPlugin/src/ArmRamp.h`
- Test: `MotionProviderPlugin/tests/test_armramp.cpp`
- Modify: `MotionProviderPlugin/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: existing `ArmRamp` (`ArmState`, `toggle()`, `requestDisarm()`, `update()`, `state()`, `blend()`).
- Produces: `void ArmRamp::requestArm();` — moves `Disarmed`/`Disarming` to `Arming`, leaves `Arming`/`Armed` unchanged. Consumed by Task 6.

- [ ] **Step 1: Write the failing test**

Create `MotionProviderPlugin/tests/test_armramp.cpp`:

```cpp
#include "ArmRamp.h"
#include <cstdio>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }

int main() {
    // From Disarmed -> Arming.
    {
        ArmRamp r;   // starts Disarmed
        r.requestArm();
        check(r.state() == ArmState::Arming, "requestArm from Disarmed -> Arming");
    }
    // From Disarming -> Arming (reverses the ramp).
    {
        ArmRamp r;
        r.requestArm();                      // Arming
        r.update(1.0, 0.5, 0.5);             // ramps to Armed (blend 1)
        check(r.state() == ArmState::Armed, "reaches Armed");
        r.requestDisarm();                   // Disarming
        check(r.state() == ArmState::Disarming, "requestDisarm -> Disarming");
        r.requestArm();                      // back to Arming mid-ramp
        check(r.state() == ArmState::Arming, "requestArm from Disarming -> Arming");
    }
    // From Armed -> unchanged (no toggle).
    {
        ArmRamp r;
        r.requestArm();
        r.update(1.0, 0.5, 0.5);             // Armed
        r.requestArm();                      // idempotent
        check(r.state() == ArmState::Armed, "requestArm while Armed -> stays Armed");
    }
    // From Arming -> unchanged.
    {
        ArmRamp r;
        r.requestArm();                      // Arming
        r.requestArm();                      // idempotent
        check(r.state() == ArmState::Arming, "requestArm while Arming -> stays Arming");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
```

- [ ] **Step 2: Register the test and confirm it fails**

In `MotionProviderPlugin/tests/CMakeLists.txt`, after the `test_heartbeat` block, add:

```cmake
add_executable(test_armramp test_armramp.cpp)
target_include_directories(test_armramp PRIVATE ../src)
```

And in the test registrations:

```cmake
add_test(NAME armramp COMMAND test_armramp)
```

Run: `cmake -S MotionProviderPlugin/tests -B MotionProviderPlugin/tests/build && cmake --build MotionProviderPlugin/tests/build --target test_armramp && ./MotionProviderPlugin/tests/build/test_armramp`
Expected: FAIL — `requestArm` is not a member of `ArmRamp`.

- [ ] **Step 3: Add requestArm()**

In `MotionProviderPlugin/src/ArmRamp.h`, immediately after the existing `requestDisarm()` method (line 22), add:

```cpp
    // Force a ramp-up to the live pose (symmetric to requestDisarm).
    void requestArm() {
        if (state_ == ArmState::Disarmed || state_ == ArmState::Disarming)
            state_ = ArmState::Arming;
    }
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cmake --build MotionProviderPlugin/tests/build --target test_armramp && ./MotionProviderPlugin/tests/build/test_armramp`
Expected: PASS — `6 checks, 0 failures`, exit code 0.

- [ ] **Step 5: Commit**

```bash
git add MotionProviderPlugin/src/ArmRamp.h MotionProviderPlugin/tests/test_armramp.cpp MotionProviderPlugin/tests/CMakeLists.txt
git commit -m "feat(MotionProvider): add ArmRamp::requestArm"
```

---

## Task 4: ArmGate (fault / e-stop latch)

**Files:**
- Create: `MotionProviderPlugin/src/ArmGate.h`
- Test: `MotionProviderPlugin/tests/test_armgate.cpp`
- Modify: `MotionProviderPlugin/tests/CMakeLists.txt`
- Modify: `MotionProviderPlugin/CMakeLists.txt`

**Interfaces:**
- Consumes: nothing (pure logic).
- Produces: header-only `class ArmGate { bool update(bool hwArmed); void latchDisarm(); bool latched() const; };`. `update` returns `true` on the tick the switch goes armed→disarmed (the reset point) and clears the latch; `latchDisarm()` sets the latch; final arm intent is computed by the caller as `hwArmed && !latched()`. Consumed by Task 6.

- [ ] **Step 1: Write the failing test**

Create `MotionProviderPlugin/tests/test_armgate.cpp`:

```cpp
#include "ArmGate.h"
#include <cstdio>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }

// Arm intent as the caller computes it.
static bool intent(ArmGate& g, bool hwArmed) {
    g.update(hwArmed);
    return hwArmed && !g.latched();
}

int main() {
    // Plain follow: switch on -> armed, switch off -> disarmed.
    {
        ArmGate g;
        check(intent(g, false) == false, "off -> no intent");
        check(intent(g, true)  == true,  "on -> intent");
        check(intent(g, true)  == true,  "still on -> intent");
        check(intent(g, false) == false, "off -> no intent");
    }
    // Latch (fault/e-stop) while switch stays on: stays disarmed until cycled.
    {
        ArmGate g;
        (void)intent(g, true);           // armed
        g.latchDisarm();                 // fault or e-stop
        check((true && !g.latched()) == false, "latched -> no intent while switch on");
        check(intent(g, true) == false,  "still on + latched -> no intent");
        // Cycle the switch off: update returns true (reset point) and clears latch.
        bool cleared = g.update(false);
        check(cleared == true, "switch off -> reset point reported");
        check(g.latched() == false, "switch off clears the latch");
        // Switch back on: arms again.
        check(intent(g, true) == true, "re-arm after cycle");
    }
    // Reset point only fires on the on->off edge, not while steady off.
    {
        ArmGate g;
        (void)g.update(true);
        check(g.update(false) == true, "on->off is a reset point");
        check(g.update(false) == false, "steady off is not a reset point");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
```

- [ ] **Step 2: Register the test and confirm it fails**

In `MotionProviderPlugin/tests/CMakeLists.txt`, after the `test_armramp` block, add:

```cmake
add_executable(test_armgate test_armgate.cpp)
target_include_directories(test_armgate PRIVATE ../src)
```

And in the test registrations:

```cmake
add_test(NAME armgate COMMAND test_armgate)
```

Run: `cmake -S MotionProviderPlugin/tests -B MotionProviderPlugin/tests/build && cmake --build MotionProviderPlugin/tests/build --target test_armgate`
Expected: FAIL — `ArmGate.h` not found.

- [ ] **Step 3: Write the header**

Create `MotionProviderPlugin/src/ArmGate.h`:

```cpp
#pragma once

// Latch that gates the hardware arm switch. A fault or manual e-stop latches
// disarm; the latch clears only when the switch is cycled off (armed->disarmed),
// so a fault can't be instantly re-armed by a switch that is still held on.
// Pure, header-only, no deps.
class ArmGate {
public:
    // Call once per tick with the hardware-armed signal (switch closed AND
    // heartbeat fresh). Returns true on the armed->disarmed edge (the reset
    // point), and clears the latch on that edge.
    bool update(bool hwArmed) {
        const bool cleared = prev_ && !hwArmed;
        if (cleared) latched_ = false;
        prev_ = hwArmed;
        return cleared;
    }

    // Latch disarm until the switch is cycled off (fault or manual e-stop).
    void latchDisarm() { latched_ = true; }

    bool latched() const { return latched_; }

private:
    bool prev_    = false;
    bool latched_ = false;
};
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cmake --build MotionProviderPlugin/tests/build --target test_armgate && ./MotionProviderPlugin/tests/build/test_armgate`
Expected: PASS — `11 checks, 0 failures`, exit code 0.

- [ ] **Step 5: Add ArmGate.h to the plugin build**

In `MotionProviderPlugin/CMakeLists.txt`, in the `HEADERS` list, after `src/ArmRamp.h` (line 85), add:

```cmake
    src/ArmGate.h
```

(No `.cpp` — header-only.)

- [ ] **Step 6: Commit**

```bash
git add MotionProviderPlugin/src/ArmGate.h MotionProviderPlugin/tests/test_armgate.cpp MotionProviderPlugin/tests/CMakeLists.txt MotionProviderPlugin/CMakeLists.txt
git commit -m "feat(MotionProvider): add ArmGate fault/e-stop latch"
```

---

## Task 5: SerialLink receive path

**Files:**
- Modify: `MotionProviderPlugin/src/SerialLink.h`
- Modify: `MotionProviderPlugin/src/SerialLink.cpp`

**Interfaces:**
- Consumes: `HeartbeatDecoder` (Task 2); `SerialPort::readBlocking(void*, size_t)` (existing, 20ms internal timeout).
- Produces: `bool SerialLink::heartbeatArmed() const;` and `bool SerialLink::heartbeatFresh(double maxAgeSec) const;`. Consumed by Task 6.

No automated test: the RX path is a thread over a real serial handle. Coverage comes from `HeartbeatDecoder` (Task 2); this task is build-verified and bench-verified (Task 7).

- [ ] **Step 1: Add RX members and accessors to the header**

In `MotionProviderPlugin/src/SerialLink.h`, add the include near the top (after `#include <cstdint>`):

```cpp
#include "HeartbeatDecoder.h"
```

Add the public accessors (after `std::string port() const { return port_; }`, line 32):

```cpp
    bool heartbeatArmed() const { return hbArmed_.load(); }
    bool heartbeatFresh(double maxAgeSec) const;
```

Add the private RX members (after the existing `std::thread ioThread_;` / atomics block, near line 47):

```cpp
    std::thread rxThread_;
    HeartbeatDecoder rxDecoder_;                 // touched only by the RX thread
    std::atomic<bool> hbArmed_{false};
    std::atomic<long long> hbLastMicros_{0};     // steady_clock micros of last valid frame; 0 = never
```

Add the private method declarations (near `void ioThreadLoop();`):

```cpp
    void rxThreadLoop();
```

- [ ] **Step 2: Start/stop the RX thread alongside the TX thread**

In `MotionProviderPlugin/src/SerialLink.cpp`, in `startIoThread()`, after the existing TX thread is created (`ioThread_ = std::thread(&SerialLink::ioThreadLoop, this);`), add:

```cpp
    if (rxThread_.joinable()) rxThread_.join();  // reap a self-exited RX thread
    rxDecoder_.reset();
    rxThread_ = std::thread(&SerialLink::rxThreadLoop, this);
```

In `stopIoThread()`, after the existing TX-thread join block, add:

```cpp
    if (rxThread_.joinable()) {
        running_.store(false);
        rxThread_.join();
    }
```

(Setting `running_` false is idempotent; the TX block above already set it.)

- [ ] **Step 3: Implement the RX loop**

In `MotionProviderPlugin/src/SerialLink.cpp`, add the include at the top if not present:

```cpp
#include <chrono>
```

(already present) and add the RX loop definition (e.g. after `ioThreadLoop()`):

```cpp
void SerialLink::rxThreadLoop() {
    uint8_t buf[64];
    while (running_.load(std::memory_order_relaxed) && serial_.isOpen()) {
        // Blocks up to SerialPort's internal read timeout (~20ms), then returns
        // whatever arrived (0 on timeout). Low-CPU wait, not a busy-poll.
        const std::size_t n = serial_.readBlocking(buf, sizeof(buf));
        for (std::size_t i = 0; i < n; ++i) {
            if (rxDecoder_.feed(buf[i])) {
                hbArmed_.store(rxDecoder_.armed(), std::memory_order_relaxed);
                const auto now = std::chrono::steady_clock::now().time_since_epoch();
                const long long us =
                    std::chrono::duration_cast<std::chrono::microseconds>(now).count();
                hbLastMicros_.store(us, std::memory_order_relaxed);
            }
        }
    }
}
```

- [ ] **Step 4: Implement heartbeatFresh()**

In `MotionProviderPlugin/src/SerialLink.cpp`, add:

```cpp
bool SerialLink::heartbeatFresh(double maxAgeSec) const {
    const long long last = hbLastMicros_.load(std::memory_order_relaxed);
    if (last == 0) return false;   // never received one
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const long long us =
        std::chrono::duration_cast<std::chrono::microseconds>(now).count();
    return (us - last) <= static_cast<long long>(maxAgeSec * 1e6);
}
```

- [ ] **Step 5: Build the plugin**

Run: `cd MotionProviderPlugin && ./build-macos.sh debug`
Expected: build succeeds; no warnings about `rxThreadLoop`, `heartbeatFresh`, `hbArmed_`.

- [ ] **Step 6: Commit**

```bash
git add MotionProviderPlugin/src/SerialLink.h MotionProviderPlugin/src/SerialLink.cpp
git commit -m "feat(MotionProvider): add SerialLink RX thread for arm heartbeat"
```

---

## Task 6: Drive arm state from the hardware switch

**Files:**
- Modify: `MotionProviderPlugin/src/MotionProvider.h`
- Modify: `MotionProviderPlugin/src/MotionProvider.cpp`

**Interfaces:**
- Consumes: `SerialLink::heartbeatFresh`/`heartbeatArmed` (Task 5); `ArmGate` (Task 4); `ArmRamp::requestArm` (Task 3).
- Produces: hardware-driven arm behavior in `onFlightLoopTick`; `UI_DISARM` now latches. `onUiAction` mode toggle handling and enum changes are Task 7.

No automated test (MotionProvider pulls in X-Plane deps via DataRefManager/StatusWindow); the arm logic itself is covered by the `ArmGate`/`ArmRamp` unit tests. Build- and bench-verified.

- [ ] **Step 1: Add the ArmGate member and heartbeat constant**

In `MotionProviderPlugin/src/MotionProvider.h`, add the include (after `#include "ArmRamp.h"`):

```cpp
#include "ArmGate.h"
```

Add the member alongside `ArmRamp armRamp_;` (near line 71):

```cpp
    ArmGate armGate_;
```

In `MotionProviderPlugin/src/MotionProvider.cpp`, in the anonymous namespace at the top (near `constexpr double kTwoPi ...`), add:

```cpp
constexpr double kHeartbeatMaxAgeSec = 1.5;   // 3x the gateway's 500ms send period
```

- [ ] **Step 2: Replace fault-only disarm with the hardware-driven arm block**

In `MotionProviderPlugin/src/MotionProvider.cpp`, in `onFlightLoopTick`, find the existing line:

```cpp
    if (monitor_.fault() != FaultCode::None) armRamp_.requestDisarm();  // home-on-fault (latched)
```

Replace it with:

```cpp
    // Hardware arm switch: fresh heartbeat AND armed. A fault or e-stop latches
    // disarm (ArmGate) until the switch is cycled off.
    const bool hwArmed = serial_ &&
                         serial_->heartbeatFresh(kHeartbeatMaxAgeSec) &&
                         serial_->heartbeatArmed();
    if (armGate_.update(hwArmed)) monitor_.clear();   // switch cycled off -> reset latch + fault
    if (monitor_.fault() != FaultCode::None) armGate_.latchDisarm();  // home-on-fault (latched)

    const bool armIntent = hwArmed && !armGate_.latched();
    if (armIntent && armRamp_.state() == ArmState::Disarmed) {
        // Rising edge into an auto (SIM) arm: reset the stateful filters so the
        // ramp starts from a clean pose. Manual mode has no filters to reset.
        if (!manualMode_ && washout_ && effects_) { washout_->reset(); effects_->reset(); }
    }
    if (armIntent) armRamp_.requestArm();
    else           armRamp_.requestDisarm();
```

(This sits after the `monitor_.update(...)` call and before `armRamp_.update(...)`, exactly where the old fault line was.)

- [ ] **Step 3: Make the DISARM button latch**

In `MotionProviderPlugin/src/MotionProvider.cpp`, in `onUiAction`, change the `UI_DISARM` case from:

```cpp
        case UI_DISARM:      armRamp_.requestDisarm(); break;
```

to:

```cpp
        case UI_DISARM:      armRamp_.requestDisarm(); armGate_.latchDisarm(); break;
```

- [ ] **Step 4: Neutralize the now-obsolete UI_ARM / UI_MANUAL handlers**

In `MotionProviderPlugin/src/MotionProvider.cpp`, in `onUiAction`, delete the entire `case UI_ARM:` block (lines handling "ARM in AUTO") and the entire `case UI_MANUAL:` block (lines handling "ARM in MANUAL"). Arming is now hardware-driven; these UI actions no longer exist after Task 7. Leave the rest of the switch intact.

- [ ] **Step 5: Build the plugin**

Run: `cd MotionProviderPlugin && ./build-macos.sh debug`
Expected: build succeeds. It is expected that `UI_ARM`/`UI_MANUAL` are still referenced by `StatusWindow.cpp` at this point — they remain defined in the enum until Task 7, so this still compiles.

- [ ] **Step 6: Run the full test suite (no regressions)**

Run: `cmake -S MotionProviderPlugin/tests -B MotionProviderPlugin/tests/build && cmake --build MotionProviderPlugin/tests/build && ctest --test-dir MotionProviderPlugin/tests/build --output-on-failure`
Expected: all tests PASS, including `heartbeat`, `armramp`, `armgate`.

- [ ] **Step 7: Commit**

```bash
git add MotionProviderPlugin/src/MotionProvider.h MotionProviderPlugin/src/MotionProvider.cpp
git commit -m "feat(MotionProvider): drive arm state from hardware switch heartbeat"
```

---

## Task 7: Status window — SIM/MANUAL toggle, DISARM e-stop, state display

**Files:**
- Modify: `MotionProviderPlugin/src/StatusData.h`
- Modify: `MotionProviderPlugin/src/StatusWindow.cpp`
- Modify: `MotionProviderPlugin/src/MotionProvider.cpp`

**Interfaces:**
- Consumes: `StatusData` (arm state fields already present); `MotionProvider::onUiAction`.
- Produces: `UI_TOGGLE_MODE` action; UI with no ARM/MANUAL buttons. Terminal task — no downstream consumers.

- [ ] **Step 1: Update the UiAction enum**

In `MotionProviderPlugin/src/StatusData.h`, replace the `UiAction` enum (lines 9-19) with:

```cpp
enum UiAction {
    UI_RELOAD = 0,
    UI_NEXT_AXIS,
    UI_NUDGE_MINUS,
    UI_NUDGE_PLUS,
    UI_RESET,
    UI_RESCAN_PORTS,
    UI_TOGGLE_MODE,   // toggle SIM <-> MANUAL (only while disarmed)
    UI_DISARM         // manual e-stop
};
```

(Removes `UI_ARM` and `UI_MANUAL`.)

- [ ] **Step 2: Handle the mode toggle in MotionProvider**

In `MotionProviderPlugin/src/MotionProvider.cpp`, in `onUiAction`, add a case (e.g. after `UI_RELOAD`):

```cpp
        case UI_TOGGLE_MODE:
            // Switch SIM <-> MANUAL only while fully disarmed (avoids a pose jump).
            if (armRamp_.state() == ArmState::Disarmed) manualMode_ = !manualMode_;
            break;
```

- [ ] **Step 3: Replace the ARM/MANUAL/DISARM button row in the status window**

In `MotionProviderPlugin/src/StatusWindow.cpp`, in `draw()`, replace the entire block from the comment `// ---- ARM / MANUAL / DISARM: single 3-button control ----` through its closing `y -= 18;` (currently lines 185-209) with:

```cpp
    // ---- Mode toggle (SIM/MANUAL, disarmed only) + DISARM e-stop ----
    // Arming is driven by the hardware switch; the UI only chooses the mode to
    // arm into and offers a software e-stop. Active/static = yellow; clickable =
    // cyan; disabled = grey.
    const int  st       = data_.armState;   // 0 Disarmed,1 Arming,2 Armed,3 Disarming
    const bool disarmed = (st == 0);
    const bool armedish = (st == 1 || st == 2);
    const bool manActive = armedish && data_.manualMode;   // used by manual DOF UI below
    {
        int bx = x;
        // Mode toggle: clickable only while disarmed.
        const char* modeLabel = data_.manualMode ? "[ MANUAL ]" : "[ SIM ]";
        if (disarmed) {
            bx += button(bx, y, modeLabel, UI_TOGGLE_MODE, 0.7f, 0.9f, 1.0f) + gap;
        } else {
            drawString(bx, y, modeLabel, 1.0f, 0.85f, 0.2f);   // yellow = locked-in mode
            bx += static_cast<int>(std::string(modeLabel).size()) * cw + gap;
        }
        // DISARM e-stop: clickable whenever not fully disarmed.
        if (!disarmed) {
            button(bx, y, "[ DISARM ]", UI_DISARM, 0.7f, 0.9f, 1.0f);
        } else {
            drawString(bx, y, "[ DISARM ]", 0.45f, 0.45f, 0.5f);  // grey = disabled
        }
    }
    y -= 18;
```

- [ ] **Step 4: Build the plugin**

Run: `cd MotionProviderPlugin && ./build-macos.sh debug`
Expected: build succeeds with no references to `UI_ARM`/`UI_MANUAL` remaining (they are gone from the enum and no longer referenced).

- [ ] **Step 5: Run the full test suite**

Run: `cmake -S MotionProviderPlugin/tests -B MotionProviderPlugin/tests/build && cmake --build MotionProviderPlugin/tests/build && ctest --test-dir MotionProviderPlugin/tests/build --output-on-failure`
Expected: all tests PASS.

- [ ] **Step 6: Commit**

```bash
git add MotionProviderPlugin/src/StatusData.h MotionProviderPlugin/src/StatusWindow.cpp MotionProviderPlugin/src/MotionProvider.cpp
git commit -m "feat(MotionProvider): SIM/MANUAL toggle + DISARM e-stop, arm state as display"
```

---

## Task 8: End-to-end bench verification

**Files:** none (verification only).

No code changes. Confirms the hardware-in-the-loop behavior the unit tests can't cover.

- [ ] **Step 1: Flash the gateway and wire the switch**

Run: `cd MotionGateway && ~/.platformio/penv/bin/pio run -t upload`
Wire a switch between pin 26 and GND on the MotionGateway board.

- [ ] **Step 2: Verify the heartbeat on the wire (optional sanity check)**

With the plugin disconnected, observe the gateway USB output; a 4-byte `H B <00|01> 0D` frame should appear every ~500ms, the third byte tracking the switch (0x01 closed/armed, 0x00 open).

- [ ] **Step 3: Verify arm tracking in X-Plane**

Load the MotionProvider plugin in X-Plane, select the gateway's serial port. Confirm:
- Switch closed → status window shows `ARMING` then `ARMED` within ~1.5s.
- Switch open → shows `DISARMING` then `disarmed (park)`.
- Unplug USB while armed → disarms within ~1.5s (heartbeat stale).

- [ ] **Step 4: Verify the e-stop latch**

With the switch closed (armed), click `[ DISARM ]`. Confirm the platform disarms and stays disarmed even though the switch is still closed. Cycle the switch off then on; confirm it re-arms.

- [ ] **Step 5: Verify the mode toggle gate**

While disarmed, confirm `[ SIM ]`/`[ MANUAL ]` toggles and is cyan (clickable). While armed, confirm the mode label is yellow and not clickable. Confirm arming in MANUAL shows the manual DOF controls.

---

## Self-Review

**Spec coverage:**
- Gateway armPin INPUT_PULLUP + `armed==(LOW)` → Task 1. ✓
- Heartbeat every 500ms, period in Configuration.h → Task 1 (`kUsbHeartbeatIntervalMs`). ✓
- `bool armed` payload → Task 1 (byte 3). ✓
- Plugin receives heartbeat → Tasks 2, 5. ✓
- Missing/false armed → DISARM → Task 6 (`heartbeatFresh`/`armIntent`). ✓
- Present + armed → ARM → Task 6. ✓
- Idempotent state guards ("if not X yet") → `ArmRamp::requestArm`/`requestDisarm` are state-guarded (Tasks 3, existing). ✓
- Mode toggle [SIM]/[MANUAL] instead of 3 buttons → Task 7. ✓
- Arm state display only → Task 7 (state text retained, no button). ✓
- Keep manual DISARM e-stop → Tasks 6, 7. ✓
- Mode toggle only while disarmed → Tasks 6/7 (`UI_TOGGLE_MODE` guard). ✓
- 1500ms freshness → Task 6 (`kHeartbeatMaxAgeSec = 1.5`). ✓
- Not added to SerialMessageId.h → honored (new `HeartbeatDecoder`). ✓
- BenchDebug out of scope → not touched. ✓

**Type consistency:** `HeartbeatDecoder::feed/armed/reset`, `SerialLink::heartbeatFresh(double)/heartbeatArmed()`, `ArmRamp::requestArm()`, `ArmGate::update(bool)/latchDisarm()/latched()`, `UI_TOGGLE_MODE`, `kHeartbeatMaxAgeSec` — names used consistently across Tasks 2–7.

**Placeholder scan:** none — all steps carry concrete code/commands.
