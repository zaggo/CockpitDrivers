# Profiled Arm/Disarm Moves ("Goto") — Design Spec

**Date:** 2026-08-27
**Status:** Design approved in chat; implementation not started.
**Scope:** MotionProviderPlugin, MotionGateway, MotionActor, shared/CANBase.

## Problem

The arm/disarm sequence is generated in the plugin as a linear pose blend streamed as
discrete 60 Hz demand frames (`ArmRamp` in `MotionProviderPlugin/src/ArmRamp.h`, blended in
`MotionProvider::blendedCommand`). On the actor, every demand frame becomes an individual
Kangaroo `p(pos, speed)` micro-move with 10% speed headroom
(`MotionActor::setDemands`, `MotionActor/src/MotionActor.cpp`). The actuator arrives ~10%
early each frame and dwells — a built-in 60 Hz velocity ripple. Flight cues mask this;
a 6-second mathematically constant-velocity glide exposes it as grinding/roughness.
The homing sequence feels silky because it is a single `p(target, speed)` command that the
Kangaroo executes as one continuous internal profile.

Goal: run arm/disarm the same way — one profiled move per actuator, commanded once.

## Design

### 1. Wire protocol

**Serial plugin→gateway — new frame type "BG"** (only valid in gateway Mode 1 / BFF mode):

```
'B' 'G'  t1..t6 MSB (6 bytes)  t1..t6 LSB (6 bytes)  duration_ms MSB LSB  0x0D
```

17 bytes total. Target layout identical to the BFF demand frame (BFF actuator order,
0..65280 scale). `duration_ms` is uint16 big-endian. Parser change in
`MotionGateway::handleSerialInput`: in the `SyncC` state, `'C'` → existing BFF path,
`'G'` → new goto path (collect 14 bytes, then CR). Any other byte → resync as today.

**CAN gateway→actor — new message `actorPairGoto = 0x382`** (in `MotionMessageId`,
`shared/CANBase/include/MotionMessageId.h`). MUST stay inside the 0x380–0x38F command
block: the actors' RXB1 range filter (mask 0x7F0, filter 0x380 — `MotionActor/src/CAN.cpp`
`begin()`) passes only that block. 0x382 is free. Payload (8 bytes):

```
[0] nodeId   [1..2] act1 target (BE)   [3..4] act2 target (BE)
[5..6] duration_ms (BE)                [7] reserved = 0
```

**Why duration, not speed:** one shared speed makes actuators with different travel arrive
at different times (legs trickle in, like the homing move to logical min). With a shared
duration each actor computes `speed = |target - current| / duration` per channel itself —
all six arrive together and the platform moves as one rigid pose. The actor knows its
current position (`lastCommandedPosition`; fallback: confirmed `getP()` when
`haveLastCommanded` is false, e.g. right after homing — a one-shot ~20 ms/channel
round-trip is acceptable for a rare command).

### 2. MotionActor

New method `MotionActor::gotoDemands(uint16_t d1, uint16_t d2, uint16_t durationMs)`:

- Only in `active` state (like `setDemands`).
- Clamp `durationMs` to 100..30000.
- Per channel: map demand → Kangaroo units (same `map()` as `setDemands`), compute
  `speed = |target - current| * 1000 / durationMs` (Kangaroo units/s, min 1), issue a
  single fire-and-forget `p(pos, speed)` — the `calibrationMove` pattern. **No** streaming
  toggle, no monitors, no `waitAll`.
- Afterwards set `lastCommandedPosition[i] = pos`, `lastDemandTimestampMs = now`,
  `haveLastCommanded = true` — so the first streamed demand after the move computes
  delta ≈ 0 instead of snapping.

CAN handler (`MotionActor/src/CAN.cpp`): new `case actorPairGoto` — validate
`len >= 7 && data[0] == kNodeId`, coalesce into `pendingGoto*` fields, apply **outside**
the RX drain loop (same pattern as `pendingDemand`; Kangaroo serial writes must not run
inside the drain). A goto clears any `demandPending` (stale demand must not fire after it).
Apply order in `loop()`: goto before demand.

### 3. MotionGateway

- Parse BG frames into `pendingGoto[6]` + `pendingGotoDurationMs` ("latest wins"),
  forward after the serial drain (same rule as `pendingDemand`): map targets through
  `actorMappingMode1`, send 3× `actorPairGoto`.
- **MaxAge-resync consistency (critical):** `checkMaxAgeResync()`
  (`MotionGateway/src/MotionGateway.cpp`) resends the last demand after 5 s of silence.
  During a 6 s goto with the plugin's stream held, it would resend the STALE pre-goto pose
  and yank the platform back. Fix: when forwarding a goto, also write the goto targets
  into `actorDemand[pairIdx]` and refresh `actorDemandMeta[pairIdx].lastSendTimestamp`.
  A resync during the move then resends the goto target as a plain demand — the actor sees
  delta = 0 and skips it. The change-dedup likewise swallows the first identical frames
  when streaming resumes.
- A goto is forwarded regardless of the demand change-dedup (it is a command, not a demand).

### 4. MotionProviderPlugin

**SerialLink** (`src/SerialLink.h/.cpp`), two additions:

- `sendOneShot(const uint8_t* data, size_t len)` — thread-safe one-shot command buffer;
  the I/O thread writes it with priority over demand frames, exactly once.
- `holdStream(bool)` — while held, the I/O thread writes neither new demand frames nor the
  ~10 Hz keepalive (the keepalive would resend the pre-goto pose and fight the move).
  One-shots still go out while held.

**Race rule:** always set hold **before** sending the BG one-shot. Serial is FIFO, so no
stale demand can arrive at the gateway after the BG frame.

**MotionProvider** (`src/MotionProvider.cpp`): the Arming/Disarming phases switch from
"stream the blend" to "goto + timer":

- Arm edge (Disarmed → Arming): targets = IK(clampToReachable(current live pose)),
  duration = `safetyCfg_.armRampSec`. Sequence: `holdStream(true)` → encode BG →
  `sendOneShot` → start timer `duration + 0.3 s margin`.
- Disarm edge: targets = IK(parkPose_), duration = `safetyCfg_.disarmRampSec`, same flow.
- Timer expiry: `safety_->reset(gotoTargets)` (SafetyLimiter continues from the arrived
  pose), force ArmRamp to Armed/Disarmed (blend 1/0), `holdStream(false)`, resume normal
  streaming.
- Switch flip mid-move: send a new BG to the new target with the full configured duration
  and restart the timer. The Kangaroo re-profiles from its current position — inherently
  smooth. No partial-duration math.
- `arm_ramp_sec` / `disarm_ramp_sec` in configuration.toml keep their meaning (move
  duration). Status window shows progress from the plugin timer (it no longer reflects
  actual actuator position — cosmetic, acceptable).
- Manual mode & Identify: unchanged (they run while Armed, streaming normally).

### 5. Failure cases & compatibility

- Serial loss mid-move: the actor completes the profiled move (slow, safe); existing
  heartbeat/watchdog semantics unchanged. Plugin-side reconnect logic unchanged.
- Fault / e-stop during an arm move: DISARM path sends a goto-park immediately, which
  overrides the running profile cleanly.
- Sim pause during a move: timer runs on wall clock (flight-loop dt), margin covers jitter.
- **Rollout is lockstep:** plugin + gateway + all 3 actor nodes must be updated together.
  Old actor FW ignores the unknown CAN id → after the timer the plugin would resume
  streaming from a pose the platform never moved to; the SafetyLimiter rate-caps the catch-up
  (~1 s at 30000 counts/s) — not dangerous, but harsh. Flash everything.
- Kangaroo/Sabretooth (DeScribe) settings stay untouched.
- The actor-side streaming speed cap (range/2 per s) and the plugin `max_velocity_cps`
  constraint (< ~32000) are unaffected by this feature.

### 6. Verification plan

1. **Actor alone:** gateway bench FW (`MotionGateway` env `megaatmega2560_bench`) gets a
   new command that emits `actorPairGoto` directly on CAN (pattern: existing `tr`/`ta`/`td`
   commands in `MotionGateway/src/BenchDebug.cpp`). Observe: one continuous glide, both
   channels arrive together at the commanded duration.
2. **Plugin headless:** socat pty + a small check (pattern:
   `MotionProviderPlugin/tools/measure_bff_timing.py`): on arm edge exactly one BG frame,
   demand stream pauses during the move window and resumes after; keepalive silent while held.
3. **Full chain at the rig:** arm/disarm feel test (the original showcase for the
   roughness), switch-flip mid-move, e-stop mid-move.

## Implementation order

1. `shared/CANBase`: `actorPairGoto = 0x382` in `MotionMessageId.h`.
2. `MotionActor`: `gotoDemands()` + CAN handler + pendingGoto plumbing.
3. `MotionGateway`: BG parser branch + forward + actorDemand/resync consistency
   (+ bench command for verification).
4. `MotionProviderPlugin`: SerialLink `sendOneShot`/`holdStream`, MotionProvider
   transition state machine, BG encoder (extend `BffEncoder` or sibling).
5. Flash actors + gateway, build/deploy plugin, run verification plan.
