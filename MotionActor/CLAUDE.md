# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

See also the monorepo-level `../CLAUDE.md` for repo-wide conventions, and in particular its
**Motion chain** section for the protocol shared with `MotionGateway` and `MotionProviderPlugin`.
This file covers what's specific to MotionActor.

## Commands

Run from this directory:

```bash
pio run -e nanoatmega328new              # production build
pio run -e nanoatmega328new_testbench    # smoothness test bench (-D MOTION_TESTBENCH=1)
pio run -e megaatmega1280                # alternative board
pio run -t upload -e nanoatmega328new
pio device monitor -b 115200
```

Debug logging is off by default (`DEBUGLOG_ENABLE 0` in `Configuration.h`) and goes to a
SoftwareSerial on pins 8/9 at 9600 baud, not to USB — the Nano has no second hardware UART and
`Serial` would collide with the Kangaroo link.

## What MotionActor is

One node driving **two** actuators of the 6DOF platform through a Dimension Engineering **Kangaroo
x2** motion controller (`HardwareSerial` at 19200, `Kangaroo.h`). Three identical boards make up the
six legs.

**`kNodeId` is a compile-time constant.** `Configuration.h` sets `kNodeId` to
`actorNodeId1`/`2`/`3`, and `kActorAddress` derives from it (`127 + nodeId`). Each of the three
boards therefore needs its **own build with that line edited** before flashing — there is no runtime
node-ID assignment. Flashing all three from one build is a silent misconfiguration: two nodes answer
to the same address.

## Kangaroo command path (`MotionActor.cpp`)

This is where the platform's smoothness is won or lost. Three entry points, all requiring
`state == active`:

- **`setDemands(d1, d2)`** — the streamed 60 Hz path. A bare `p(pos)` would run to each target at
  maximum configured speed and stop, turning a smooth stream into a staircase of micro-snaps.
  Instead each move carries a speed limit sized to arrive as the *next* demand is due:
  `speed = delta * 1100 / dtMs` (10 % headroom), with `dtMs` clamped to 10..100 ms. Unchanged
  targets are skipped rather than resent. Speed is capped at `range/2` per second — a large delta
  after a demand gap would otherwise command a violent snap (measured up to 12× range/s). That cap
  still sits above the plugin's `SafetyLimiter` ceiling (30000 counts/s ≈ 0.46 range/s), so no
  legitimate motion cue is clipped.
- **`gotoDemands(d1, d2, durationMs)`** — one profiled move per channel for arm/disarm. Speed is
  derived from a **shared duration** (`delta * 1000 / durationMs`) rather than a shared speed, so
  both channels — and across the three nodes, all six legs — arrive together and the platform moves
  as one rigid pose. Duration is clamped to 100..30000 ms; the same `range/2` ceiling applies.
  Current position comes from `lastCommandedPosition`, falling back to a `getP()` round-trip when
  `haveLastCommanded` is false (e.g. straight after homing).
- **`home()` / `powerDown()` / `calibrationMove()` / `saveLogicalMin|Max()`** — setup and shutdown.

`setStreaming(true)` is used in the `active` state and off for home/start/waitAll/powerDown. This
matters: a confirmed (non-streaming) Kangaroo `p()` costs a **~20 ms blocking round-trip per
channel**, which throttled the production demand path to ~25 Hz and was the original source of the
felt jerkiness.

Logical min/max limits live in EEPROM (`StoredLogicalLimits`, magic `0x4D41`) and are validated
against the hardware limits on load.

## CAN side (`CAN.cpp/h`)

Subclasses `BaseCAN` directly (not `InstrumentCAN` — the motion chain has its own heartbeat IDs).

- Sends `actorHeartbeat` (`0x301`) every 500 ms; treats the gateway as dead after 1500 ms without
  `gatewayHeartbeat` (`0x300`).
- RXB1 uses a **range filter** (mask `0x7F0`, filter `0x380`), so every gateway→actor command must
  live in `0x380`–`0x38F`. `actorPairDemand` (`0x110`) is matched separately.
- **Kangaroo serial writes never run inside the RX drain loop.** Demands and gotos are latched into
  `pendingDemand*` / `pendingGoto*` during the drain and applied once per `loop()` pass afterwards.
  Blocking inside the drain overflows the RX buffer and drops gateway heartbeats.
- Status LEDs: red on CAN error/uninitialised, green when running (pins 4/3).

## Test bench (`TestBench.cpp/h`, `MOTION_TESTBENCH=1`)

CAN-triggered command-strategy experiments against the Kangaroo, sampled into a static RAM buffer
(160 samples — 240 collided with the stack and crashed at boot) and dumped back over CAN on
`0x3A0`–`0x3A2`. Triggered by `0x386`–`0x388`. Strategy 9 is a passthrough used to measure the
production path end to end. Analysed by `../MotionGateway/tools/analyze_smoothness.py`; the gate
metric is `residual/step ≤ 0.25` from a per-leg quadratic fit.

Watch RAM headroom on the Nano — PlatformIO's reported RAM percentage does not include the heap, and
stack/heap collisions here present as boot crashes rather than compile errors.
