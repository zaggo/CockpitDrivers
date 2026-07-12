# RPM Gauge (RPMGaugeCAN) DCU + Plugin Support Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire live RPM and tach-hours (odometer) data from X-Plane, through `DCUProviderPlugin` and the DCU gateway, onto the CAN bus so `RPMGaugeCAN` (node `0x05`) receives `rpm` (CAN `0x106`) and `odometer` (CAN `0x1F0`) frames, plus give DCU's bench-test mode commands to simulate both without X-Plane attached.

**Architecture:** Two new serial message types (`SerialMessageRPM`, `SerialMessageOdometer`) carry plugin→DCU data over the existing `0xAA 0x55 TYPE LEN PAYLOAD` framing. The plugin polls new datarefs on a per-message rate-limited accumulator (mirroring the existing fuel/lights/transponder pattern) and sends plain floats. `DCUReceiver` decodes, caches, change-detects, and forwards onto CAN using the existing `packBE16`/`CAN::sendMessage` primitives, with 5-second max-age resync like every other message type. `BenchDebug` gets independent `rp`/`oh` console commands that build and send the same CAN frames directly, for testing without a plugin/serial connection.

**Tech Stack:** PlatformIO/Arduino C++ (DCU, `megaatmega2560`), C++17 X-Plane 12 plugin (CMake, `DCUProviderPlugin`), shared `CANBase` library.

## Global Constraints

- No unit test infrastructure exists for CAN/DCUReceiver/DataRefManager/DCUProvider logic in this
  repo — `test/` directories are empty PlatformIO scaffolds (per `DCU/CLAUDE.md`). This plan does
  not introduce new test infrastructure; verification is build success + manual BenchDebug console
  testing + (optionally) live X-Plane flight-loop testing, matching the approved spec's Testing
  section and existing repo convention.
- Use the existing `packBE16`/`unpackBE16` helpers (`DCU/include/WireEncoding.h`) for all CAN-side
  big-endian 16-bit packing — do not hand-roll shift/mask code where these apply.
- The CAN-wire-side odometer payload (5×`uint8` digits + 1×`uint16`) must be built as an explicit
  byte array (`byte data[7]`), never as a mixed-width `struct` — avoids compiler padding entirely
  rather than fighting it with `__attribute__((packed))`.
- The plugin-side serial downlink payloads (all-`float` structs) are fine as plain structs — no
  padding risk since every field is the same width, matching the existing `FuelData`/`LightsData`
  local-struct convention in `DCUProvider.cpp`.
- Follow existing per-board file layout / naming conventions (`../CLAUDE.md`, `DCU/CLAUDE.md`) —
  no new files needed for this feature; all changes are additions to existing files.
- Git CLI is currently broken in the working sandbox (`xcode-select` misconfiguration outside this
  task's scope) — commit steps are written as normal `git commit` calls; if git is still broken
  when a task runs, skip the commit step and note it rather than working around it destructively.

---

### Task 1: Shared serial message type enum

**Files:**
- Modify: `shared/CANBase/include/SerialMessageId.h`

**Interfaces:**
- Produces: `MessageType::SerialMessageRPM` (`0x05`), `MessageType::SerialMessageOdometer` (`0x06`)
  — consumed by Task 3 (DCUReceiver) and Task 6 (DCUProvider).

- [ ] **Step 1: Add the two new enumerators**

Edit `shared/CANBase/include/SerialMessageId.h`:

```cpp
enum class MessageType : uint8_t {
    SerialMessageFuel   = 0x01,
    SerialMessageLights = 0x02,
    SerialMessageTransponder = 0x03,
    SerialMessageHandbrake = 0x04,
    SerialMessageRPM = 0x05,
    SerialMessageOdometer = 0x06,
};
```

- [ ] **Step 2: Build DCU to verify the header still compiles**

Run: `pio run` (from `DCU/`)
Expected: `SUCCESS`, no errors referencing `SerialMessageId.h`.

- [ ] **Step 3: Build the plugin to verify the header still compiles there too**

Run: `./build-macos.sh` (from `DCUProviderPlugin/`)
Expected: build completes without errors referencing `SerialMessageId.h`.

- [ ] **Step 4: Commit**

```bash
git add shared/CANBase/include/SerialMessageId.h
git commit -m "feat: add RPM and odometer serial message types"
```

---

### Task 2: DCU CAN error-tracking capacity bump

**Files:**
- Modify: `DCU/src/CAN.h:35`

**Interfaces:**
- No new interfaces — pure constant change.

- [ ] **Step 1: Bump `kMaxCanIdErrors`**

Edit `DCU/src/CAN.h`, change:

```cpp
        static constexpr uint8_t kMaxCanIdErrors = 8;
```

to:

```cpp
        static constexpr uint8_t kMaxCanIdErrors = 12;
```

Reason: adding `rpm` (`0x106`) and `odometer` (`0x1F0`) as new outbound-trackable CAN IDs brings
steady-state distinct IDs to 9 (4 existing outbound + 3 existing inbound + 2 new), leaving headroom
for per-node heartbeat-timeout entries (`instrumentHeartbeatId + nodeId`) without silently dropping
error/alarm tracking once the array fills (`CAN.cpp:304`, `setCanIdError`, adds nothing past the cap
without warning).

- [ ] **Step 2: Build to verify**

Run: `pio run` (from `DCU/`)
Expected: `SUCCESS`.

- [ ] **Step 3: Commit**

```bash
git add DCU/src/CAN.h
git commit -m "fix: raise kMaxCanIdErrors to fit new RPM/odometer CAN IDs"
```

---

### Task 3: DCU DCUReceiver — decode, cache, forward RPM/odometer to CAN

**Files:**
- Modify: `DCU/src/DCUReceiver.h`
- Modify: `DCU/src/DCUReceiver.cpp`

**Interfaces:**
- Consumes: `MessageType::SerialMessageRPM`/`SerialMessageOdometer` (Task 1),
  `packBE16(uint8_t* dst, uint16_t value)` (`DCU/include/WireEncoding.h`),
  `CanMessageId::rpm`/`CanMessageId::odometer` (existing, `shared/CANBase/include/CanMessageId.h`),
  `CAN::sendMessage(CanMessageId id, uint8_t len, byte* data)` (existing, `DCU/src/CAN.h:22`),
  `isStale(uint32_t lastSendMs, uint32_t nowMs, uint32_t maxAgeMs)` (`DCU/include/Heartbeat.h`).
- Produces: nothing consumed by other tasks — this is a leaf.

- [ ] **Step 1: Add cached state + MessageMeta to `DCUReceiver.h`**

Edit `DCU/src/DCUReceiver.h`, add after the existing `Transponder` member block (after line 44):

```cpp
        // RPM Gauge
        uint16_t rpmValue = 0;

        // Odometer (RPM Gauge)
        uint8_t tachHrs1000 = 0;
        uint8_t tachHrs100 = 0;
        uint8_t tachHrs10 = 0;
        uint8_t tachHrs1 = 0;
        uint8_t tachHrsTenths = 0;
        uint16_t tachHrsHundredths100 = 0;
```

and add these method declarations after `void sendTransponder();` (line 29):

```cpp
        void sendRpm();
        void sendOdometer();
```

and add these `MessageMeta` members after `MessageMeta transponderMeta;` (line 49):

```cpp
        MessageMeta rpmMeta;
        MessageMeta odometerMeta;
```

- [ ] **Step 2: Initialize the new `MessageMeta` in the constructor**

Edit `DCU/src/DCUReceiver.cpp`, in the constructor (after line 17 `transponderMeta = {0, 5000};`):

```cpp
  rpmMeta = {0, 5000};
  odometerMeta = {0, 5000};
```

- [ ] **Step 3: Add `SerialMessageRPM`/`SerialMessageOdometer` cases to `handleFrame`**

Edit `DCU/src/DCUReceiver.cpp`, add these two `case` blocks inside the `switch (type)` in
`handleFrame`, right before the `default:` line (line 124):

```cpp
  case MessageType::SerialMessageRPM:
  {
    // Payload: float rpm (4 bytes)
    if (len != 4)
      return;

    float rpm;
    memcpy(&rpm, payload + 0, 4);

    uint16_t rpmRounded = static_cast<uint16_t>(rpm);
    if (rpmRounded != rpmValue)
    {
      rpmValue = rpmRounded;
      DEBUGLOG_PRINTLN(String(F("Received MSG_RPM Datagram rpm: ")) + String(rpmValue));
      sendRpm();
    }
    break;
  }

  case MessageType::SerialMessageOdometer:
  {
    // Payload: 6 floats - hrs1000, hrs100, hrs10, hrs1, hrsTenths, hrsHundredths (24 bytes)
    if (len != 24)
      return;

    float hrs1000, hrs100, hrs10, hrs1, hrsTenths, hrsHundredths;
    memcpy(&hrs1000, payload + 0, 4);
    memcpy(&hrs100, payload + 4, 4);
    memcpy(&hrs10, payload + 8, 4);
    memcpy(&hrs1, payload + 12, 4);
    memcpy(&hrsTenths, payload + 16, 4);
    memcpy(&hrsHundredths, payload + 20, 4);

    uint8_t d1000 = static_cast<uint8_t>(hrs1000);
    uint8_t d100 = static_cast<uint8_t>(hrs100);
    uint8_t d10 = static_cast<uint8_t>(hrs10);
    uint8_t d1 = static_cast<uint8_t>(hrs1);
    uint8_t dTenths = static_cast<uint8_t>(hrsTenths);
    uint16_t dHundredths100 = static_cast<uint16_t>(hrsHundredths * 1000.);

    if (d1000 != tachHrs1000 || d100 != tachHrs100 || d10 != tachHrs10 ||
        d1 != tachHrs1 || dTenths != tachHrsTenths || dHundredths100 != tachHrsHundredths100)
    {
      tachHrs1000 = d1000;
      tachHrs100 = d100;
      tachHrs10 = d10;
      tachHrs1 = d1;
      tachHrsTenths = dTenths;
      tachHrsHundredths100 = dHundredths100;
      DEBUGLOG_PRINTLN(String(F("Received MSG_ODOMETER Datagram")));
      sendOdometer();
    }
    break;
  }

```

- [ ] **Step 4: Add `sendRpm()`/`sendOdometer()` implementations**

Edit `DCU/src/DCUReceiver.cpp`, add these two methods after `sendTransponder()` (after line 176):

```cpp
void DCUReceiver::sendRpm()
{
  byte data[2] = {0};

  packBE16(data + 0, rpmValue);

  canBus->sendMessage(CanMessageId::rpm, 2, data);

  // Update last send timestamp for maxAge resync
  rpmMeta.lastSendTimestamp = millis();
}

void DCUReceiver::sendOdometer()
{
  byte data[7] = {0};

  data[0] = tachHrs1000;
  data[1] = tachHrs100;
  data[2] = tachHrs10;
  data[3] = tachHrs1;
  data[4] = tachHrsTenths;
  packBE16(data + 5, tachHrsHundredths100);

  canBus->sendMessage(CanMessageId::odometer, 7, data);

  // Update last send timestamp for maxAge resync
  odometerMeta.lastSendTimestamp = millis();
}
```

- [ ] **Step 5: Add maxAge resync checks**

Edit `DCU/src/DCUReceiver.cpp`, in `checkMaxAgeResync()`, add before the closing `}` (after the
transponder block ending at line 201):

```cpp

  // Check RPM message
  if (isStale(rpmMeta.lastSendTimestamp, now, rpmMeta.maxAgeMs))
  {
    DEBUGLOG_PRINTLN(String(F("MaxAge resync for rpm")));
    sendRpm();
  }

  // Check odometer message
  if (isStale(odometerMeta.lastSendTimestamp, now, odometerMeta.maxAgeMs))
  {
    DEBUGLOG_PRINTLN(String(F("MaxAge resync for odometer")));
    sendOdometer();
  }
```

- [ ] **Step 6: Build to verify**

Run: `pio run` (from `DCU/`)
Expected: `SUCCESS`.

- [ ] **Step 7: Commit**

```bash
git add DCU/src/DCUReceiver.h DCU/src/DCUReceiver.cpp
git commit -m "feat: forward RPM and odometer serial messages onto CAN"
```

---

### Task 4: DCU BenchDebug — `rp`/`oh` bench-test commands

**Files:**
- Modify: `DCU/src/BenchDebug.h`
- Modify: `DCU/src/BenchDebug.cpp`

**Interfaces:**
- Consumes: `packBE16` (`DCU/include/WireEncoding.h`), `CanMessageId::rpm`/`CanMessageId::odometer`
  (existing), `CAN::sendMessage` (existing, `DCU/src/CAN.h:22`).
- Produces: nothing consumed by other tasks — this is a leaf (independent of Task 3; `BenchDebug`
  and `DCUReceiver` are mutually exclusive at compile time via the `BENCHDEBUG` flag in
  `Configuration.h`/`main.cpp`, and each already duplicates its own send logic rather than sharing
  it — matching the existing `sendFuelLevel`/`sendCockpitLightLevel` duplication between the two
  classes).

- [ ] **Step 1: Add state + method declarations to `BenchDebug.h`**

Edit `DCU/src/BenchDebug.h`, add after `void sendCockpitLightLevel();` (line 22):

```cpp
        void sendRpm();
        void sendOdometer();
```

and add after `uint8_t cockpitLightLevel = 0;` (line 29):

```cpp

        uint16_t rpmValue = 0;
        float odometerHours = 0.;
```

- [ ] **Step 2: Add `sendRpm()`/`sendOdometer()` implementations**

Edit `DCU/src/BenchDebug.cpp`, add after `sendCockpitLightLevel()` (after line 46):

```cpp

void BenchDebug::sendRpm() {
    byte data[2] = {0};
    packBE16(data + 0, rpmValue);

    Serial.println(String(F("Send RPM: ")) + rpmValue);

    canBus->sendMessage(CanMessageId::rpm, 2, data);
}

void BenchDebug::sendOdometer() {
    // Round to the nearest ten-thousandth-of-an-hour once (preserves up to 4
    // typed decimal digits, e.g. 123.4595), then extract every digit via
    // integer arithmetic - doing it digit-by-digit in floating point truncates
    // instead of rounds (823.3 -> tenths computed as 2.999... -> 2).
    //
    // The last two decimal digits (thousandths/ten-thousandths-of-an-hour)
    // don't map to a displayed digit - the device only shows whole
    // hundredths - but they set the fractional part of the CAN "hundredths"
    // field, which the device uses to roll the hundredths digit smoothly
    // between two values instead of snapping (see RPMGaugeCAN's
    // Odometer::displayNumber, which animates once that fraction is >= 0.9).
    uint32_t totalTenThousandths = static_cast<uint32_t>(odometerHours * 10000. + 0.5);

    uint32_t wholeHours = totalTenThousandths / 10000;
    uint16_t fracTenThousandths = static_cast<uint16_t>(totalTenThousandths % 10000);

    uint8_t d1000 = static_cast<uint8_t>((wholeHours / 1000) % 10);
    uint8_t d100 = static_cast<uint8_t>((wholeHours / 100) % 10);
    uint8_t d10 = static_cast<uint8_t>((wholeHours / 10) % 10);
    uint8_t d1 = static_cast<uint8_t>(wholeHours % 10);
    uint8_t dTenths = static_cast<uint8_t>(fracTenThousandths / 1000);
    uint16_t dHundredths100 = static_cast<uint16_t>(fracTenThousandths % 1000) * 10;

    byte data[7] = {0};
    data[0] = d1000;
    data[1] = d100;
    data[2] = d10;
    data[3] = d1;
    data[4] = dTenths;
    packBE16(data + 5, dHundredths100);

    Serial.println(String(F("Send Odometer hours: ")) + odometerHours);

    canBus->sendMessage(CanMessageId::odometer, 7, data);
}
```

- [ ] **Step 3: Add `rp`/`oh` command handling**

Edit `DCU/src/BenchDebug.cpp`, in `handleAltimeterInput`, add before the `else if (command.startsWith("?"))` block (before line 71):

```cpp
    } else if (command.startsWith("rp")) {
        String rString = command.substring(2);
        rString.trim();
        rpmValue = static_cast<uint16_t>(rString.toInt());
        Serial.println(String(F("RPM set to "))+rpmValue);
        sendRpm();
        return true;
    } else if (command.startsWith("oh")) {
        String rString = command.substring(2);
        rString.trim();
        odometerHours = rString.toFloat();
        Serial.println(String(F("Odometer hours set to "))+odometerHours);
        sendOdometer();
        return true;
```

- [ ] **Step 4: Add help text**

Edit `DCU/src/BenchDebug.cpp`, in the `"?"` help block, add after `Serial.println(F("cl<0..255>: set light brightness"));` (line 75):

```cpp
        Serial.println(F("rp<rpm>: set RPM gauge value"));
        Serial.println(F("oh<hours>: set odometer total hours"));
```

- [ ] **Step 5: Build with BENCHDEBUG enabled to verify**

Temporarily set `#define BENCHDEBUG 1` in `DCU/src/Configuration.h`, then:

Run: `pio run` (from `DCU/`)
Expected: `SUCCESS`.

Then revert `Configuration.h` back to `#define BENCHDEBUG 0` (its committed default) before
continuing — do not leave bench mode enabled in the committed tree.

- [ ] **Step 6: Commit**

```bash
git add DCU/src/BenchDebug.h DCU/src/BenchDebug.cpp
git commit -m "feat: add rp/oh BenchDebug commands to simulate RPM gauge"
```

---

### Task 5: Plugin DataRefManager — RPM + tach-hours datarefs

**Files:**
- Modify: `DCUProviderPlugin/src/DataRefManager.h`
- Modify: `DCUProviderPlugin/src/DataRefManager.cpp`

**Interfaces:**
- Produces: `float getRpm() const`, `float getTachHours1000() const`, `float getTachHours100()
  const`, `float getTachHours10() const`, `float getTachHours1() const`, `float
  getTachHoursTenths() const`, `float getTachHoursHundredths() const` — consumed by Task 6
  (`DCUProvider::updateDownlink`).

- [ ] **Step 1: Add dataref members**

Edit `DCUProviderPlugin/src/DataRefManager.h`, add after `XPLMDataRef dr_ParkingBrake = nullptr;`
(line 79):

```cpp
    XPLMDataRef dr_rpm = nullptr;
    XPLMDataRef dr_tachHrs1000 = nullptr;
    XPLMDataRef dr_tachHrs100 = nullptr;
    XPLMDataRef dr_tachHrs10 = nullptr;
    XPLMDataRef dr_tachHrs1 = nullptr;
    XPLMDataRef dr_tachHrsTenths = nullptr;
    XPLMDataRef dr_tachHrsHundredths = nullptr;
```

- [ ] **Step 2: Add getter declarations**

Edit `DCUProviderPlugin/src/DataRefManager.h`, add in the `DOWNLINK` public section, after
`float getDomeLightBrightness() const;  // 0.0 - 1.0` (line 34):

```cpp

    // RPM / Tach
    float getRpm() const;  // engine_speed_rpm[0], direct RPM value
    float getTachHours1000() const;
    float getTachHours100() const;
    float getTachHours10() const;
    float getTachHours1() const;
    float getTachHoursTenths() const;
    float getTachHoursHundredths() const;  // 0.0-1.0 fraction, *1000 on the wire
```

- [ ] **Step 3: Look up the new datarefs in `onAircraftLoaded()`**

Edit `DCUProviderPlugin/src/DataRefManager.cpp`, add after
`dr_ParkingBrake = XPLMFindDataRef("sim/cockpit2/controls/parking_brake_ratio");` (line 33):

```cpp

    // RPM / Tach (VFLYTEAIR is an aircraft-specific third-party dataref namespace;
    // absent on other aircraft, getters fall back to 0 via readFloat's default)
    dr_rpm = XPLMFindDataRef("sim/cockpit2/engine/indicators/engine_speed_rpm");
    dr_tachHrs1000 = XPLMFindDataRef("VFLYTEAIR/tach/TachTimeHrs1000");
    dr_tachHrs100 = XPLMFindDataRef("VFLYTEAIR/tach/TachTimeHrs100");
    dr_tachHrs10 = XPLMFindDataRef("VFLYTEAIR/tach/TachTimeHrs10");
    dr_tachHrs1 = XPLMFindDataRef("VFLYTEAIR/tach/TachTimeHrs1");
    dr_tachHrsTenths = XPLMFindDataRef("VFLYTEAIR/tach/TachTimeTenths");
    dr_tachHrsHundredths = XPLMFindDataRef("VFLYTEAIR/tach/TachTimeHundredths");
```

Note: `TachTimeHrs1000`/`Hrs100`/`Hrs10`/`Hrs1`/`Tenths` are `INT`-typed datarefs in X-Plane (confirmed
from the aircraft's dataref reference); only `TachTimeHundredths` is `FLOAT`. Reading an `INT`-only
dataref with the float accessor (`XPLMGetDataf`) returns 0 regardless of its real value — it does
not autoconvert. Their getters below must use an int-reading helper, not `readFloat`.

- [ ] **Step 4: Implement the getters**

Edit `DCUProviderPlugin/src/DataRefManager.cpp`, add after `getDomeLightBrightness()` (after line 57):

```cpp

// RPM / Tach
float DataRefManager::getRpm() const
{
    auto values = readFloatArray(dr_rpm, 0, 1);
    return values[0];
}

float DataRefManager::getTachHours1000() const
{
    return readInt(dr_tachHrs1000, 0);
}

float DataRefManager::getTachHours100() const
{
    return readInt(dr_tachHrs100, 0);
}

float DataRefManager::getTachHours10() const
{
    return readInt(dr_tachHrs10, 0);
}

float DataRefManager::getTachHours1() const
{
    return readInt(dr_tachHrs1, 0);
}

float DataRefManager::getTachHoursTenths() const
{
    return readInt(dr_tachHrsTenths, 0);
}

float DataRefManager::getTachHoursHundredths() const
{
    return readFloat(dr_tachHrsHundredths, 0.0f);
}
```

Also add the `readInt` helper next to `readFloat` (`DCUProviderPlugin/src/DataRefManager.h`, private
section, and `.cpp`):

```cpp
// .h, after: static float readFloat(XPLMDataRef dr, float def = 0.0f);
static float readInt(XPLMDataRef dr, int def = 0);
```

```cpp
// .cpp, after readFloat()'s definition:
float DataRefManager::readInt(XPLMDataRef dr, int def)
{
    if (!dr)
    {
        return static_cast<float>(def);
    }
    return static_cast<float>(XPLMGetDatai(dr));
}
```

- [ ] **Step 5: Build to verify**

Run: `./build-macos.sh` (from `DCUProviderPlugin/`)
Expected: build completes without errors.

- [ ] **Step 6: Commit**

```bash
git add DCUProviderPlugin/src/DataRefManager.h DCUProviderPlugin/src/DataRefManager.cpp
git commit -m "feat: add RPM and tach-hours dataref getters"
```

---

### Task 6: Plugin DCUProvider — RPM/odometer downlink

**Files:**
- Modify: `DCUProviderPlugin/src/DCUProvider.h`
- Modify: `DCUProviderPlugin/src/DCUProvider.cpp`

**Interfaces:**
- Consumes: `MessageType::SerialMessageRPM`/`SerialMessageOdometer` (Task 1),
  `DataRefManager::getRpm()`/`getTachHours1000()`...`getTachHoursHundredths()` (Task 5),
  `MessageQueue::enqueueTx(MessageType type, const void* payload, uint8_t len)` (existing,
  `DCUProviderPlugin/src/MessageQueue.h:26`).
- Produces: nothing consumed by other tasks — this is the final leaf; after this task the feature
  is end-to-end wired.

- [ ] **Step 1: Add rate-limiting accumulators/constants**

Edit `DCUProviderPlugin/src/DCUProvider.h`, add after `float transponderAccumulator_ = 0.0f;`
(line 79):

```cpp
    float rpmAccumulator_ = 0.0f;
    float odometerAccumulator_ = 0.0f;
```

and after `static constexpr float TRANSPONDER_RATE = 10.0f; // Hz` (line 83):

```cpp
    static constexpr float RPM_RATE = 50.0f;      // Hz
    static constexpr float ODOMETER_RATE = 10.0f; // Hz
```

- [ ] **Step 2: Add the RPM downlink block**

Edit `DCUProviderPlugin/src/DCUProvider.cpp`, in `updateDownlink()`, add after the Transponder Data
block (after line 235, before the `// TODO: Add more downlink data...` comment):

```cpp

    // ============ RPM Data (50 Hz) ============
    rpmAccumulator_ += dt;
    float rpmRate = 1.0f / RPM_RATE;

    if (rpmAccumulator_ >= rpmRate)
    {
        struct RpmData
        {
            float rpm;
        };

        RpmData rpm;
        rpm.rpm = dataRefMgr_->getRpm();

        msgQueue_->enqueueTx(MessageType::SerialMessageRPM, &rpm, sizeof(rpm));

        rpmAccumulator_ = 0.0f;
    }

    // ============ Odometer Data (10 Hz) ============
    odometerAccumulator_ += dt;
    float odometerRate = 1.0f / ODOMETER_RATE;

    if (odometerAccumulator_ >= odometerRate)
    {
        struct OdometerData
        {
            float hrs1000;
            float hrs100;
            float hrs10;
            float hrs1;
            float hrsTenths;
            float hrsHundredths;
        };

        OdometerData odometer;
        odometer.hrs1000 = dataRefMgr_->getTachHours1000();
        odometer.hrs100 = dataRefMgr_->getTachHours100();
        odometer.hrs10 = dataRefMgr_->getTachHours10();
        odometer.hrs1 = dataRefMgr_->getTachHours1();
        odometer.hrsTenths = dataRefMgr_->getTachHoursTenths();
        odometer.hrsHundredths = dataRefMgr_->getTachHoursHundredths();

        msgQueue_->enqueueTx(MessageType::SerialMessageOdometer, &odometer, sizeof(odometer));

        odometerAccumulator_ = 0.0f;
    }
```

- [ ] **Step 3: Build to verify**

Run: `./build-macos.sh` (from `DCUProviderPlugin/`)
Expected: build completes without errors.

- [ ] **Step 4: Commit**

```bash
git add DCUProviderPlugin/src/DCUProvider.h DCUProviderPlugin/src/DCUProvider.cpp
git commit -m "feat: send RPM and odometer downlink data to the DCU gateway"
```

---

## End-to-end verification (after all tasks)

- [ ] **Bench test (no X-Plane needed):** flash DCU with `BENCHDEBUG` temporarily enabled, connect
  RPMGaugeCAN to the CAN bus, run `rp2500` then `oh1234.56` via the DCU serial console — confirm
  the gauge needle moves and the odometer digits update to match.
- [ ] **Live test:** with `BENCHDEBUG` back to `0`, flash DCU, connect X-Plane with the
  `DCUProviderPlugin` loaded and a VFLYTEAIR-based aircraft, confirm RPM gauge tracks engine RPM
  and odometer increments plausibly during a flight session.
