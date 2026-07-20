# Motion arm switch: hardware-driven ARM/DISARM

## Problem

Today the motion platform is armed/disarmed purely from software (`[ARM]`/`[MANUAL]`/`[DISARM]`
buttons in the Motion Provider status window). There is no physical switch in the loop — a pilot
sitting on the platform has no hardware kill switch, only a mouse click in an X-Plane window.

This feature adds a physical arm switch on the MotionGateway board. The gateway reports the
switch state to the MotionProviderPlugin over the existing USB link every 500ms. The plugin arms
the platform if and only if the switch is closed (armed) and its signal is fresh; it disarms the
instant the switch opens or the signal goes stale. The plugin's status window stops offering an
ARM button — arming is no longer something you click, only something you flip a switch for. A
software `[DISARM]` e-stop button remains as an independent safety net.

## Scope

In scope:
- MotionGateway: read a GPIO arm switch, send its state as a periodic USB heartbeat.
- MotionProviderPlugin: receive that heartbeat, drive `ArmRamp` from it instead of from UI buttons.
- MotionProviderPlugin: status window UI changes (remove ARM/MANUAL buttons, add a SIM/MANUAL mode
  toggle, keep DISARM).

Out of scope:
- BenchDebug (bench-console simulation mode on MotionGateway) — not updated to simulate the arm
  switch. Can be a follow-up if bench testing without the physical switch turns out to be needed.
- Any change to the CAN-side heartbeat protocol (`gatewayHeartbeat`/`actorHeartbeat`, CAN IDs
  0x300/0x301) — unrelated, unaffected.
- Any change to the DCU/`SerialMessageId.h` protocol — the Motion USB link has always been a
  separate protocol family (see "Wire format" below) and stays that way.

## MotionGateway changes

`Configuration.h`:
```cpp
const uint8_t kArmPin = 26;                    // switch to GND, INPUT_PULLUP
const uint32_t kUsbHeartbeatIntervalMs = 500;  // heartbeat period to MotionProviderPlugin
```

`MotionGateway` constructor: `pinMode(kArmPin, INPUT_PULLUP);` alongside the existing
`kMode1Pin`/`kMode2Pin` setup.

`MotionGateway::loop()`: a new periodic block, timed the same way as the existing 200ms mode-check
(`lastXxxCheckTimestampMs` pattern), firing every `kUsbHeartbeatIntervalMs`:
- Read `armPin`. `armed = (digitalRead(kArmPin) == LOW)`.
- Build and send the 4-byte heartbeat frame (see Wire format).

This runs unconditionally — independent of the current `MotionMode` (mode0/mode1/mode2) and
independent of the BFF/SimTools downlink parser state machine in `handleSerialInput()`. It is a
second, unrelated stream sharing the same `Serial` (USB CDC is full duplex; the downlink parser
already reads while other code writes, e.g. debug logging).

No debounce: the switch is sampled once per 500ms interval, same tolerance already accepted for
the 200ms-sampled mode pins.

## Wire format

New 4-byte frame from MotionGateway to MotionProviderPlugin:

```
'H' 'B' <armed: 0x00|0x01> 0x0D
```

Sync bytes `'H','B'`, one payload byte, CR terminator — mirrors the existing "BC"-sync convention
already used by `MotionGateway::handleSerialInput()` for the (opposite-direction) BFF/SimTools
demand frames.

This is deliberately **not** added to `shared/CANBase/include/SerialMessageId.h`. That header is
the DCU↔DCUProviderPlugin USB protocol; CLAUDE.md documents that changes there ripple into
`DCUReceiver`/`DCUSender`/`DataRefManager`. The MotionGateway↔MotionProviderPlugin USB link has
always been its own protocol (`BffEncoder` on the plugin side, the sync-byte parser in
`MotionGateway::handleSerialInput` on the gateway side) — the heartbeat is a small addition to that
existing family, not a reason to graft an unrelated message type onto the DCU protocol.

## MotionProviderPlugin: receiving the heartbeat

`SerialLink` today is TX-only: a single I/O thread writes the latest setpoint frame at a fixed
rate (`rateHz_`, typically 60Hz) and never reads. This adds a receive path:

- **`HeartbeatDecoder`** (new, small, standalone — mirrors `BffEncoder`): a pure byte-in/frame-out
  state machine (sync `'H'` → sync `'B'` → payload → CR), no thread or I/O dependencies, easy to
  unit-test in isolation.
- **New RX thread in `SerialLink`**, separate from the existing TX thread. Uses
  `SerialPort::readBlocking()` — already implemented, already has a 20ms internal timeout, low-CPU
  blocking wait — but currently unused by anything. Started/stopped alongside the existing TX
  thread in `connect()`/`stop()`.
  - Rationale for a *separate* thread rather than interleaving reads into the existing TX loop:
    the TX loop's timing (`period` computed from `rateHz_`) drives the setpoint stream's cadence;
    splicing a 20ms blocking read into that loop would perturb it. A dedicated thread reading in
    its own loop leaves TX timing untouched. Concurrent read/write on the same serial handle from
    two threads is the standard pattern for full-duplex serial (matches how `DCUProviderPlugin`'s
    `ConnectionManager` is structured per its own docs).
  - Feeds each byte read to `HeartbeatDecoder`. On a complete valid frame: stores `armed` (bool)
    and a `std::chrono::steady_clock` timestamp, both behind atomics (no mutex needed — a single
    writer thread, multiple reader calls from the flight-loop thread).
- **New `SerialLink` accessors**:
  - `bool heartbeatFresh(double maxAgeSec) const` — true if a valid frame arrived within
    `maxAgeSec`.
  - `bool heartbeatArmed() const` — last received armed value (meaningless if not fresh; callers
    must check `heartbeatFresh` first).
- **Freshness threshold: 1500ms** (3× the 500ms send period) — matches the existing
  `InstrumentCAN` convention elsewhere in this repo (heartbeat every 500ms, timeout after 1500ms /
  3 missed beats).

## MotionProviderPlugin: arm state becomes hardware-driven

Today, `onUiAction` handles `UI_ARM`/`UI_MANUAL` by calling `armRamp_.toggle()`, gated to only fire
from `ArmState::Disarmed`. This is replaced by a per-tick computation in `onFlightLoopTick`:

```cpp
bool hwArmed = serial_ && serial_->heartbeatFresh(1.5) && serial_->heartbeatArmed();
```

`ArmRamp` gains a new method, symmetric to the existing `requestDisarm()`:

```cpp
void requestArm() {
    if (state_ == ArmState::Disarmed || state_ == ArmState::Disarming)
        state_ = ArmState::Arming;
}
```

### The gate/reset latch

A naive `if (hwArmed) requestArm(); else requestDisarm();` breaks as soon as a fault or a manual
e-stop click needs to force a disarm: the switch is still physically ON, so `hwArmed` is still
`true` on the very next tick, and `requestArm()` would immediately undo the forced disarm —
thrashing between `Arming`/`Disarming` every frame instead of ever reaching `Disarmed`.

To prevent that, `MotionProvider` tracks one new bool, `gateClosed_` (default `false`):

- Set `true` whenever:
  - a fault newly triggers a forced disarm (existing `if (monitor_.fault() != FaultCode::None)
    armRamp_.requestDisarm();` path), or
  - the `[DISARM]` e-stop button is clicked (`UI_DISARM`).
- Cleared (`false`), along with `monitor_.clear()`, at the reset point: when `hwArmed` transitions
  **true → false** (i.e. the pilot cycles the switch off). This is the only way to clear a fault or
  an e-stop latch going forward — matches how a physical E-stop normally resets (flip off, then
  back on).

Per-tick arm intent: `armIntent = hwArmed && !gateClosed_`. If `armIntent` and
`armRamp_.state()` is `Disarmed`/`Disarming` → `requestArm()`. If `!armIntent` and state is
`Arming`/`Armed` → `requestDisarm()`.

## MotionProviderPlugin: status window UI

- Remove `UI_ARM` and the `[ARM]`/`[MANUAL]`/`[DISARM]` three-button row.
- Add a single toggle button, `UI_TOGGLE_MODE`, rendered as `[ SIM ]` or `[ MANUAL ]` depending on
  `manualMode_`. Clicking it flips `manualMode_` — **only while `ArmState::Disarmed`** (greyed out
  /non-clickable otherwise), to avoid a pose jump switching between washout-driven and hand-set
  poses while the platform is live.
- Keep `[ DISARM ]` exactly as today (visible/clickable whenever not fully disarmed) — the one
  remaining manual control, functioning as a software e-stop independent of the hardware switch.
- ARM/ARMING/ARMED/DISARMING remains a text-only display (existing `stateStr` logic in
  `StatusWindow::draw()`), now purely reflecting state driven by the hardware switch — no button
  behind it.

## Error handling / edge cases

- **Heartbeat never arrives** (gateway not connected, wrong port, powered off): `heartbeatFresh()`
  is false forever → `hwArmed` false → platform stays/goes `Disarmed`. Same as today's "no serial
  link" behavior, just via a different signal path.
- **Heartbeat was arriving, then stops** (USB unplugged, gateway resets): within 1500ms,
  `heartbeatFresh()` goes false → forced disarm, same ramp-to-park behavior as any other disarm.
  This is a second, independent path from the existing `SafetyMonitor` `SerialLost` fault (which
  reacts to `serial_->isConnected()`); both can fire together and both just call
  `armRamp_.requestDisarm()`, which is idempotent.
- **Switch flipped mid-ramp** (e.g. armed while `Disarming`): `requestArm()`/`requestDisarm()` are
  both defined in terms of current state, so this smoothly reverses the ramp direction from
  wherever the blend currently is — no special-casing needed, same as the existing
  `requestDisarm()` behavior today.
- **Fault while hardware-armed**: forced disarm + `gateClosed_ = true`, platform stays disarmed
  even though the switch is still on, until the pilot cycles the switch.
- **Garbled heartbeat bytes** (partial frame, noise): `HeartbeatDecoder` simply fails to complete a
  frame; `heartbeatFresh()` ages out normally, no special error handling needed.

## Testing

- `HeartbeatDecoder`: unit-testable in isolation (pure function, no I/O) — feed byte sequences
  (valid frame, garbage, partial frame followed by a fresh sync, back-to-back frames) and assert
  decoded `armed` values / no false positives.
- `ArmRamp::requestArm()`: extend existing `ArmRamp` unit coverage (if any) with the new method's
  four state transitions.
- Manual/bench verification (no automated test for the physical switch or full-duplex serial
  timing): wire a switch to `armPin` on a bench MotionGateway, confirm the status window's
  ARM/ARMING/ARMED/DISARMING display tracks the switch within ~1.5s, confirm `[DISARM]` e-stop
  holds disarmed until the switch is cycled, confirm SIM/MANUAL toggle is greyed out except when
  disarmed.
