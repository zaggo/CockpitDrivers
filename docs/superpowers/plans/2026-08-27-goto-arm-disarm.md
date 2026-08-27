# Goto-Based Arm/Disarm Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the streamed 60 Hz arm/disarm blend with a single profiled Kangaroo move per actuator (one "goto" command carrying targets + duration), eliminating the move-dwell ripple during arm/disarm.

**Architecture:** New serial frame "BG" (plugin→gateway) and new CAN message `actorPairGoto = 0x382` (gateway→actors). The actor computes per-channel speed from a shared duration so all six actuators arrive together. The plugin holds its demand stream (and keepalive) during the move and resumes from the arrived pose.

**Tech Stack:** PlatformIO/Arduino (AVR) for MotionActor + MotionGateway, C++/CMake for MotionProviderPlugin, shared headers in `shared/CANBase/include`.

**Spec:** `docs/superpowers/specs/2026-08-27-goto-arm-disarm-design.md`

## Global Constraints

- `actorPairGoto` MUST be `0x382` — inside the 0x380–0x38F block passed by the actors' RXB1 range filter (mask 0x7F0, filter 0x380).
- BG serial frame: `'B' 'G'  6×target-MSB  6×target-LSB  duration_ms-MSB  duration_ms-LSB  0x0D` = 17 bytes. BFF actuator order, 0..65280 scale, only valid in gateway Mode 1.
- CAN goto payload (8 bytes): `[0]=nodeId [1..2]=act1 target BE [3..4]=act2 target BE [5..6]=duration_ms BE [7]=0`.
- Race rule in the plugin: `holdStream(true)` BEFORE `sendOneShot(BG)`. Never the other way around.
- Gateway MUST update `actorDemand[]` + `actorDemandMeta[].lastSendTimestamp` when forwarding a goto (maxAge-resync consistency).
- Kangaroo/DeScribe settings are off-limits. No changes to `MotionActor::setDemands` speed math.
- Rollout is lockstep: 3 actor nodes + gateway + plugin together (Task 6).
- No native unit-test envs exist for these projects (`test/` dirs are empty scaffolds; the plugin has no test suite). Each task's test cycle is therefore: compile all envs cleanly, then the hardware/rig verification protocol in Task 6. Do not invent a test harness.
- Windows: fresh unsigned `.xpl` may be blocked by Smart App Control (error 4551 in X-Plane's Log.txt) — check Log.txt after deploying.

---

### Task 1: CANBase — new message id

**Files:**
- Modify: `shared/CANBase/include/MotionMessageId.h`

**Interfaces:**
- Produces: `MotionMessageId::actorPairGoto` (= 0x382), used by Tasks 2 and 3.

- [ ] **Step 1: Add the enum value**

In `shared/CANBase/include/MotionMessageId.h`, after `actorPairStop = 0x381,`:

```cpp
    // Profiled arm/disarm move (gateway -> actor): run one internal Kangaroo
    // profile per channel to a target, speed derived from a shared duration.
    // Payload: [0]=nodeId [1..2]=act1 target BE [3..4]=act2 target BE
    //          [5..6]=duration_ms BE [7]=reserved.
    // Must stay in 0x380-0x38F (actors' RXB1 range filter).
    actorPairGoto = 0x382,
```

- [ ] **Step 2: Verify both firmware projects still compile**

```bash
cd MotionActor && pio run
cd ../MotionGateway && pio run
```
Expected: SUCCESS for all envs (`nanoatmega328new`, `nanoatmega328new_testbench`, `megaatmega1280`; `megaatmega2560`, `megaatmega2560_bench`).

- [ ] **Step 3: Commit**

```bash
git add shared/CANBase/include/MotionMessageId.h
git commit -m "feat(CANBase): actorPairGoto message id for profiled arm/disarm moves"
```

---

### Task 2: MotionActor — gotoDemands + CAN plumbing

**Files:**
- Modify: `MotionActor/src/MotionActor.h`, `MotionActor/src/MotionActor.cpp`
- Modify: `MotionActor/src/CAN.h`, `MotionActor/src/CAN.cpp`

**Interfaces:**
- Consumes: `MotionMessageId::actorPairGoto` (Task 1).
- Produces: `void MotionActor::gotoDemands(uint16_t demand1, uint16_t demand2, uint16_t durationMs)`.

- [ ] **Step 1: Declare gotoDemands**

In `MotionActor/src/MotionActor.h`, after `void setDemands(uint16_t demand1, uint16_t demand2);`:

```cpp
        // Profiled move: one fire-and-forget p(target, speed) per channel, speed
        // chosen so BOTH channels arrive after durationMs (coordinated arrival).
        // Used for arm/disarm; runs in streaming mode like setDemands.
        void gotoDemands(uint16_t demand1, uint16_t demand2, uint16_t durationMs);
```

- [ ] **Step 2: Implement gotoDemands**

In `MotionActor/src/MotionActor.cpp`, after `setDemands` (before `calibrationMove`):

```cpp
void MotionActor::gotoDemands(uint16_t demand1, uint16_t demand2, uint16_t durationMs)
{
    if (state != MotionActorState::active)
    {
        DEBUGLOG_PRINTLN(F("Cannot goto: not active"));
        return;
    }
    if (durationMs < 100)   durationMs = 100;    // no snap moves
    if (durationMs > 30000) durationMs = 30000;

    const uint16_t demands[kActorCount] = {demand1, demand2};

    for (uint8_t i = 0; i < kActorCount; ++i)
    {
        const int32_t pos = map(demands[i], 0, 0xffff, logicalMinPosition[i], logicalMaxPosition[i]);
        const int32_t range = logicalMaxPosition[i] - logicalMinPosition[i];

        int32_t current = 0;
        bool haveCurrent = false;
        if (haveLastCommanded)
        {
            current = lastCommandedPosition[i];
            haveCurrent = true;
        }
        else
        {
            haveCurrent = readCurrentPosition(i, current);
        }

        int32_t speed;
        if (haveCurrent)
        {
            int32_t delta = pos - current;
            if (delta < 0) delta = -delta;
            // Cover delta in exactly durationMs -> speed in Kangaroo units/s.
            // delta <= ~65000, *1000 fits int32.
            speed = (delta * 1000L) / static_cast<int32_t>(durationMs);
        }
        else
        {
            // Unknown position (getP failed): same gentle glide as the first
            // post-homing demand (5 s over the full logical range).
            speed = range / 5;
        }
        if (speed < 1) speed = 1;

        actors[i]->p(pos, speed);
        lastCommandedPosition[i] = pos;
    }
    lastDemandTimestampMs = millis();
    haveLastCommanded = true;   // next streamed demand computes delta from the goto target
}
```

- [ ] **Step 3: Add pendingGoto plumbing to CAN.h**

In `MotionActor/src/CAN.h`, after the `pendingDemand2` member block:

```cpp
    // Latest goto seen while draining RX frames; applied outside the drain loop
    // (same rule as demands: Kangaroo serial writes never inside the drain).
    bool gotoPending = false;
    uint16_t pendingGoto1 = 0;
    uint16_t pendingGoto2 = 0;
    uint16_t pendingGotoDurationMs = 0;
```

- [ ] **Step 4: Handle the frame + apply it in CAN.cpp**

In `CAN::handleFrame`, after the `actorPairStop` case:

```cpp
    case MotionMessageId::actorPairGoto:
        if (len >= 7 && data[0] == static_cast<uint8_t>(kNodeId))
        {
            pendingGoto1 = (static_cast<uint16_t>(data[1]) << 8) | static_cast<uint16_t>(data[2]);
            pendingGoto2 = (static_cast<uint16_t>(data[3]) << 8) | static_cast<uint16_t>(data[4]);
            pendingGotoDurationMs = (static_cast<uint16_t>(data[5]) << 8) | static_cast<uint16_t>(data[6]);
            gotoPending = true;
            demandPending = false;   // a stale demand must not fire after the profiled move
            DEBUGLOG_PRINTLN(String(F("Received goto: ")) + pendingGoto1 + ", " + pendingGoto2 +
                             String(F(" in ")) + pendingGotoDurationMs + F(" ms"));
        }
        break;
```

In `CAN::loop()`, insert BEFORE the `if (demandPending)` block (goto wins over an
older demand parsed in the same drain):

```cpp
    if (gotoPending)
    {
        gotoPending = false;
#if MOTION_TESTBENCH
        const bool benchOwnsMotors =
            testBench != nullptr && testBench->isRunning() && !testBench->isPassthroughActive();
        if (!benchOwnsMotors)
        {
            motionActor->gotoDemands(pendingGoto1, pendingGoto2, pendingGotoDurationMs);
        }
#else
        motionActor->gotoDemands(pendingGoto1, pendingGoto2, pendingGotoDurationMs);
#endif
    }
```

- [ ] **Step 5: Compile all envs**

```bash
cd MotionActor && pio run
```
Expected: SUCCESS for `nanoatmega328new`, `nanoatmega328new_testbench`, `megaatmega1280`.

- [ ] **Step 6: Commit**

```bash
git add MotionActor/src
git commit -m "feat(MotionActor): profiled gotoDemands via actorPairGoto CAN message"
```

---

### Task 3: MotionGateway — BG parser, forwarding, resync consistency, bench command

**Files:**
- Modify: `MotionGateway/src/MotionGateway.h`, `MotionGateway/src/MotionGateway.cpp`
- Modify: `MotionGateway/src/BenchDebug.cpp` (new `gt` command + help text)

**Interfaces:**
- Consumes: `MotionMessageId::actorPairGoto` (Task 1).
- Produces: serial BG frame handling (consumed by Task 4/5's plugin-side encoder).

- [ ] **Step 1: Extend MotionGateway.h**

After `void handleSimToolsFrame(const uint8_t *data);`:

```cpp
        void handleGotoFrame(const uint8_t *data);
        void processGoto();
        void sendActorPairGoto(MotionNodeId nodeId, uint16_t act1Target,
                               uint16_t act2Target, uint16_t durationMs);
```

After the `pendingDemandValid` member:

```cpp
        // Newest complete goto frame, applied once per loop() after the drain
        // (same rule as pendingDemand). A goto supersedes a demand parsed in
        // the same drain pass.
        uint16_t pendingGoto[6] = {0};
        uint16_t pendingGotoDurationMs = 0;
        bool pendingGotoValid = false;
```

- [ ] **Step 2: Extend the parser (MotionGateway.cpp)**

Extend the RX state machine at the top of the file:

```cpp
enum class RxState : uint8_t
{
  SyncB,
  SyncC,
  Reserved,
  Data,
  CR,
  GotoData,
  GotoCR
};

static const size_t kMaxDataSize = 12;
static const size_t kGotoDataSize = 14;   // 12 target bytes + duration_ms (u16 BE)
static RxState state = RxState::SyncB;
static uint8_t data[kGotoDataSize];
static uint8_t idx = 0;
```

In `handleSerialInput()`, **mode 1 branch only**, replace the `SyncC` case and add
the goto states to the switch:

```cpp
      case RxState::SyncC:
        if (b == 'C')      { state = RxState::Reserved; }
        else if (b == 'G') { state = RxState::GotoData; idx = 0; }
        else               { state = RxState::SyncB; }
        break;
```

```cpp
      case RxState::GotoData:
        data[idx++] = b;
        if (idx >= kGotoDataSize)
        {
          state = RxState::GotoCR;
        }
        break;

      case RxState::GotoCR:
        if (b == 0x0D)
        {
          if (idx == kGotoDataSize)
          {
            handleGotoFrame(data);
          }
          state = RxState::SyncB;
        }
        else
        {
          stats.crMissBytes++;   // same wait-for-CR resync as the BFF CR state
        }
        break;
```

- [ ] **Step 3: Frame handler + forwarding (MotionGateway.cpp)**

After `handleSimToolsFrame`:

```cpp
void MotionGateway::handleGotoFrame(const uint8_t *data)
{
  for (uint8_t i = 0; i < 6; ++i)
  {
    pendingGoto[i] = ((uint16_t)data[i] << 8) | data[i + 6];
  }
  pendingGotoDurationMs = ((uint16_t)data[12] << 8) | data[13];
  pendingGotoValid = true;
  pendingDemandValid = false;   // the goto supersedes any demand from this drain
  stats.noteFrame(millis());
}

void MotionGateway::processGoto()
{
  if (mode != MotionMode::mode1)
  {
    return;
  }

  uint16_t pairTargets[kActorNodeCount][2] = {0};
  for (uint8_t actorIdx = 0; actorIdx < 6; ++actorIdx)
  {
    const ActorMapping &map = actorMappingMode1[actorIdx];
    uint8_t pairIdx = static_cast<uint8_t>(map.nodeId) - 1;
    pairTargets[pairIdx][map.motorIndex] = pendingGoto[actorIdx];
  }

  for (uint8_t pairIdx = 0; pairIdx < kActorNodeCount; ++pairIdx)
  {
    if (!canBus->isSystemActive())
    {
      continue;
    }
    // MaxAge-resync consistency: the goto target IS the platform's demand now.
    // A resync during the move resends it as a plain demand -> actor sees
    // delta 0 and skips; the change-dedup swallows identical frames when the
    // plugin resumes streaming.
    actorDemand[pairIdx] = (static_cast<uint32_t>(pairTargets[pairIdx][0]) << 16) |
                           pairTargets[pairIdx][1];
    sendActorPairGoto(static_cast<MotionNodeId>(pairIdx + 1),
                      pairTargets[pairIdx][0], pairTargets[pairIdx][1],
                      pendingGotoDurationMs);
  }
}

void MotionGateway::sendActorPairGoto(MotionNodeId nodeId, uint16_t act1Target,
                                      uint16_t act2Target, uint16_t durationMs)
{
  byte data[8] = {0};

  data[0] = static_cast<uint8_t>(nodeId);
  data[1] = (act1Target >> 8) & 0xFF;
  data[2] = act1Target & 0xFF;
  data[3] = (act2Target >> 8) & 0xFF;
  data[4] = act2Target & 0xFF;
  data[5] = (durationMs >> 8) & 0xFF;
  data[6] = durationMs & 0xFF;

  const uint32_t sendStartUs = micros();
  canBus->sendMessage(MotionMessageId::actorPairGoto, 8, data);
  stats.noteSend(micros() - sendStartUs);

  actorDemandMeta[static_cast<uint8_t>(nodeId) - 1].lastSendTimestamp = millis();
}
```

In `loop()`, insert BEFORE the `if (pendingDemandValid)` block:

```cpp
  if (pendingGotoValid)
  {
    pendingGotoValid = false;
    processGoto();
  }
```

Also reset the goto latch on mode change — in the mode-change block where
`pendingDemandValid = false;` is set, add:

```cpp
      pendingGotoValid = false;
```

- [ ] **Step 4: Bench command `gt` (BenchDebug.cpp)**

In `BenchDebug::handleBenchInput`, BEFORE the `else if (command.startsWith("gs"))`
branch:

```cpp
    else if (command.startsWith("gt"))
    {
        // gt <node 1-3|0=all> <act1 %> <act2 %> <duration ms> - profiled goto move
        int node = 0, p1 = 0, p2 = 0, dur = 0;
        if (sscanf(command.c_str() + 2, "%d %d %d %d", &node, &p1, &p2, &dur) != 4 ||
            node < 0 || node > 3 || p1 < 0 || p1 > 100 || p2 < 0 || p2 > 100 ||
            dur < 100 || dur > 30000)
        {
            Serial.println("Usage: gt <node 1-3|0=all> <act1 0-100> <act2 0-100> <duration 100-30000 ms>");
            return true;
        }
        const uint16_t d1 = static_cast<uint16_t>((p1 / 100.0) * 65280);
        const uint16_t d2 = static_cast<uint16_t>((p2 / 100.0) * 65280);
        byte data[8] = {0};
        data[1] = (d1 >> 8) & 0xFF;
        data[2] = d1 & 0xFF;
        data[3] = (d2 >> 8) & 0xFF;
        data[4] = d2 & 0xFF;
        data[5] = (static_cast<uint16_t>(dur) >> 8) & 0xFF;
        data[6] = static_cast<uint16_t>(dur) & 0xFF;
        for (uint8_t n = 1; n <= 3; ++n)
        {
            if (node != 0 && node != n) continue;
            data[0] = n;
            canBus->sendMessage(MotionMessageId::actorPairGoto, 8, data);
            Serial.println("Goto sent to node " + String(n) + ": " + String(p1) + "%/" +
                           String(p2) + "% in " + String(dur) + " ms");
        }
        return true;
    }
```

Add one line to the `?` help output (match the surrounding format), e.g.:

```cpp
    Serial.println("gt <node|0> <a1%> <a2%> <ms> - profiled goto move (arm/disarm path)");
```

- [ ] **Step 5: Compile all envs**

```bash
cd MotionGateway && pio run
```
Expected: SUCCESS for `megaatmega2560` and `megaatmega2560_bench`.

- [ ] **Step 6: Commit**

```bash
git add MotionGateway/src
git commit -m "feat(MotionGateway): BG goto frame parsing, CAN forwarding, bench gt command"
```

---

### Task 4: Plugin — BG encoder + SerialLink one-shot/hold

**Files:**
- Modify: `MotionProviderPlugin/src/BffEncoder.h`, `MotionProviderPlugin/src/BffEncoder.cpp`
- Modify: `MotionProviderPlugin/src/SerialLink.h`, `MotionProviderPlugin/src/SerialLink.cpp`

**Interfaces:**
- Produces (for Task 5):
  - `BffEncoder::kGotoFrameSize` (= 17) and `void BffEncoder::encodeGoto(const uint16_t targets[6], uint16_t durationMs, uint8_t out[kGotoFrameSize])`
  - `void SerialLink::sendOneShot(const uint8_t* data, std::size_t len)`
  - `void SerialLink::holdStream(bool hold)`

- [ ] **Step 1: BG encoder**

`BffEncoder.h`:

```cpp
namespace BffEncoder {
    constexpr std::size_t kFrameSize = 16;
    void encode(const uint16_t setpoints[6], uint8_t out[kFrameSize]);

    // Goto command frame (profiled arm/disarm move): "BG" MSB[6] LSB[6]
    // duration_ms(BE u16) CR. No reserved byte (unlike BFF).
    constexpr std::size_t kGotoFrameSize = 17;
    void encodeGoto(const uint16_t targets[6], uint16_t durationMs,
                    uint8_t out[kGotoFrameSize]);
}
```

`BffEncoder.cpp`:

```cpp
void BffEncoder::encodeGoto(const uint16_t t[6], uint16_t durationMs,
                            uint8_t out[kGotoFrameSize]) {
    out[0] = 'B';
    out[1] = 'G';
    for (int i = 0; i < 6; ++i) {
        out[2 + i]     = static_cast<uint8_t>((t[i] >> 8) & 0xFF);  // MSB[i]
        out[2 + 6 + i] = static_cast<uint8_t>(t[i] & 0xFF);         // LSB[i]
    }
    out[14] = static_cast<uint8_t>((durationMs >> 8) & 0xFF);
    out[15] = static_cast<uint8_t>(durationMs & 0xFF);
    out[16] = 0x0D;
}
```

- [ ] **Step 2: SerialLink API + state (SerialLink.h)**

After `void setFrame(...)`:

```cpp
    // One-shot command frame (e.g. a BG goto). Written once by the I/O thread,
    // with priority over demand frames. Thread-safe.
    void sendOneShot(const uint8_t* data, std::size_t len);

    // While held, the I/O thread writes neither demand frames nor the keepalive
    // (a keepalive would resend the pre-goto pose and fight the profiled move).
    // One-shots still go out. Releasing marks the current frame dirty so
    // streaming resumes immediately.
    void holdStream(bool hold);
```

After the `frameDirty_` member:

```cpp
    uint8_t oneShot_[32];
    std::size_t oneShotLen_ = 0;
    bool oneShotPending_ = false;
    bool holdStream_ = false;
```

- [ ] **Step 3: SerialLink implementation (SerialLink.cpp)**

New methods:

```cpp
void SerialLink::sendOneShot(const uint8_t* data, std::size_t len) {
    if (len == 0 || len > sizeof(oneShot_)) return;
    {
        std::lock_guard<std::mutex> lk(frameMutex_);
        std::memcpy(oneShot_, data, len);
        oneShotLen_ = len;
        oneShotPending_ = true;
    }
    frameCv_.notify_one();
}

void SerialLink::holdStream(bool hold) {
    {
        std::lock_guard<std::mutex> lk(frameMutex_);
        holdStream_ = hold;
        if (!hold) frameDirty_ = haveFrame_;   // resume immediately with the current frame
    }
    frameCv_.notify_one();
}
```

In `startIoThread()`, before spawning the thread, clear stale transfer state (a
reconnect must never come up with a held stream or a pending command from a dead
session):

```cpp
    {
        std::lock_guard<std::mutex> lk(frameMutex_);
        holdStream_ = false;
        oneShotPending_ = false;
    }
```

In `ioThreadLoop()`, replace the wait predicate and add the one-shot/hold logic
at the top of the loop body:

```cpp
        frameCv_.wait_until(lk, lastSend + keepalive, [&] {
            return oneShotPending_ || frameDirty_ || !running_.load(std::memory_order_relaxed);
        });
        if (!running_.load(std::memory_order_relaxed)) break;

        if (oneShotPending_) {
            uint8_t cmd[sizeof(oneShot_)];
            const std::size_t n = oneShotLen_;
            std::memcpy(cmd, oneShot_, n);
            oneShotPending_ = false;
            lk.unlock();
            if (serial_.writeBestEffort(cmd, n)) {
                frames_.fetch_add(1, std::memory_order_relaxed);
            }
            lk.lock();
            continue;   // deliberately does NOT touch lastSend (rate cap is for demand frames)
        }

        if (holdStream_) {
            lastSend = std::chrono::steady_clock::now();  // suppress keepalive while held
            continue;
        }

        if (!haveFrame_) { lastSend = std::chrono::steady_clock::now(); continue; }
```

(The rest of the loop — rate cap, frame copy, write — stays unchanged.)

- [ ] **Step 4: Build**

```bash
cd MotionProviderPlugin && ./build-windows.sh
```
Expected: BUILD SUCCESSFUL (auto-installs `win.xpl`).

- [ ] **Step 5: Commit**

```bash
git add MotionProviderPlugin/src/BffEncoder.h MotionProviderPlugin/src/BffEncoder.cpp MotionProviderPlugin/src/SerialLink.h MotionProviderPlugin/src/SerialLink.cpp
git commit -m "feat(MotionProviderPlugin): BG goto frame encoder, SerialLink one-shot + stream hold"
```

---

### Task 5: Plugin — goto transition state machine in MotionProvider

**Files:**
- Modify: `MotionProviderPlugin/src/MotionProvider.h`, `MotionProviderPlugin/src/MotionProvider.cpp`

**Interfaces:**
- Consumes: `BffEncoder::encodeGoto`, `SerialLink::sendOneShot`, `SerialLink::holdStream` (Task 4); `SafetyLimiter::reset(const uint16_t[6])` (exists); `ArmRamp` (exists, unchanged).

- [ ] **Step 1: State + helper declaration (MotionProvider.h)**

After the `serialWasConnected_` member:

```cpp
    // Profiled arm/disarm transition (goto-based; see
    // docs/superpowers/specs/2026-08-27-goto-arm-disarm-design.md). While a
    // goto move runs, the demand stream is held; on expiry the SafetyLimiter
    // is snapped to the arrived targets and streaming resumes.
    bool gotoActive_ = false;
    double gotoRemainingSec_ = 0.0;
    uint16_t gotoTargets_[6] = {32640,32640,32640,32640,32640,32640};
    ArmState prevArmState_ = ArmState::Disarmed;
    static constexpr double kGotoMarginSec = 0.3;
    void startGotoTransition(bool arming, const Pose& rawLive);
```

- [ ] **Step 2: Implement startGotoTransition (MotionProvider.cpp)**

After `blendedCommand`:

```cpp
void MotionProvider::startGotoTransition(bool arming, const Pose& rawLive) {
    if (!kin_ || !serial_) return;
    const Pose target = arming ? kin_->clampToReachable(rawLive) : parkPose_;
    const SolveResult s = kin_->solve(target);
    if (!s.allReachable) return;   // fall back to the streamed blend (stream not held)

    for (int i = 0; i < 6; ++i) gotoTargets_[i] = s.setpoints[i];

    double durSec = arming ? safetyCfg_.armRampSec : safetyCfg_.disarmRampSec;
    if (durSec < 0.1)  durSec = 0.1;
    if (durSec > 30.0) durSec = 30.0;
    const uint16_t durMs = static_cast<uint16_t>(std::lround(durSec * 1000.0));

    uint8_t frame[BffEncoder::kGotoFrameSize];
    BffEncoder::encodeGoto(gotoTargets_, durMs, frame);
    serial_->holdStream(true);                    // ALWAYS before the one-shot (race rule)
    serial_->sendOneShot(frame, sizeof(frame));
    gotoActive_ = true;
    gotoRemainingSec_ = durSec + kGotoMarginSec;
}
```

Add `#include <cmath>` at the top of MotionProvider.cpp if not already present
(it is — `std::isfinite` is used).

- [ ] **Step 3: Wire the transition detection into onFlightLoopTick**

Directly AFTER `armRamp_.update(dt, safetyCfg_.armRampSec, safetyCfg_.disarmRampSec);`
insert:

```cpp
    // Entering Arming/Disarming (from any source: switch, e-stop button, port
    // change) starts one profiled goto move instead of streaming the blend.
    // A mid-move reversal simply starts a new goto with the full duration -
    // the Kangaroo re-profiles from its current position.
    const ArmState armStateNow = armRamp_.state();
    if (armStateNow != prevArmState_) {
        if (armStateNow == ArmState::Arming)         startGotoTransition(true, rawLive);
        else if (armStateNow == ArmState::Disarming) startGotoTransition(false, rawLive);
        prevArmState_ = armStateNow;
    }

    if (gotoActive_) {
        gotoRemainingSec_ -= dt;   // dt is maxDtSec-clamped: a sim stall can't skip the move
        if (gotoRemainingSec_ <= 0.0) {
            gotoActive_ = false;
            if (safety_) safety_->reset(gotoTargets_);   // continue from the arrived pose
            if (serial_) serial_->holdStream(false);
        }
    }
```

- [ ] **Step 4: Gate the demand streaming**

Replace the existing serial block at the end of `onFlightLoopTick`:

```cpp
    if (serial_) {
        if (!gotoActive_) {
            uint8_t frame[BffEncoder::kFrameSize];
            BffEncoder::encode(sentSetpoints_, frame);
            serial_->setFrame(frame, sizeof(frame));
        }
        serial_->update(elapsedSec);   // real dt for reconnect timing
    }
```

(While a goto runs, SafetyLimiter keeps integrating toward the blended pose —
harmless, its state is snapped by `reset()` on completion. The status window
keeps showing the ArmRamp blend, which integrates over the same duration.)

- [ ] **Step 5: Build + deploy**

```bash
cd MotionProviderPlugin && ./build-windows.sh
```
Expected: BUILD SUCCESSFUL, auto-install to `X:\X-Plane 12\Resources\plugins\MotionProvider\64\win.xpl`.

- [ ] **Step 6: Commit**

```bash
git add MotionProviderPlugin/src/MotionProvider.h MotionProviderPlugin/src/MotionProvider.cpp
git commit -m "feat(MotionProviderPlugin): goto-based arm/disarm transitions"
```

---

### Task 6: Lockstep flash + verification

**Files:** none (flash + manual verification; user at the rig required for most steps)

- [ ] **Step 1: Flash the gateway (production FW)**

```bash
cd MotionGateway && pio run -e megaatmega2560 -t upload
```

- [ ] **Step 2: Flash all 3 actor nodes**

`MotionActor/src/Configuration.h` line 17 selects the node
(`kNodeId = MotionNodeId::actorNodeId1`). For EACH of the three boards: set
`actorNodeId1` / `actorNodeId2` / `actorNodeId3`, connect that board, then:

```bash
cd MotionActor && pio run -e nanoatmega328new -t upload
```

Leave the file at `actorNodeId1` afterwards (its checked-in state).

- [ ] **Step 3: Bench verification of the CAN path (optional but recommended)**

Flash the bench gateway (`pio run -e megaatmega2560_bench -t upload`), open the
serial console (115200), home with `ho 0`, then:

```
gt 0 50 50 5000
gt 0 20 20 3000
```

Expected: one continuous glide per move, both channels of every pair arriving
together after the commanded duration; no move-dwell ripple. Re-flash the
production gateway FW afterwards (Step 1).

- [ ] **Step 4: Full-chain rig test**

X-Plane up, plugin connected, then verify each:

1. Arm via hardware switch: single smooth 6 s profiled move park→live, then
   normal streaming (check Status window: ARMED, frames counting).
2. Disarm: smooth 4 s move to park.
3. Mid-move reversal: flip the switch during an arm move → platform re-profiles
   toward park without a snap.
4. E-stop button ([ DISARM ]) during an arm move → same smooth park move.
5. After arm completes: normal flight cues work; no snap at the streaming
   handover (SafetyLimiter reset covers residual drift).
6. Gateway `GS` stats on Serial1 stay clean (no resync/crMiss bursts).
7. X-Plane Log.txt: no plugin load errors (SAC check for the fresh win.xpl).

- [ ] **Step 5: Final commit (plan checkboxes / any fixups)**

```bash
git add -A && git commit -m "chore: goto arm/disarm verification fixups"
```
(Skip if nothing changed.)
