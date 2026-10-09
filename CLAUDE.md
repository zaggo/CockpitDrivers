# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Firmware + tooling for a home-cockpit flight-sim rig: each top-level directory is an Arduino-based
instrument/actuator board (altimeter, HSI, transponder, handbrake, servo/stepper actuators, fuel gauge,
master clock, etc.) that talks to a central DCU (Data Communication Unit) over CAN bus, and/or to
X-Plane via AirManager. `DCUProviderPlugin` is the X-Plane-side counterpart.

## Repo layout

- Board projects (`AirspeedCAN`, `AltimeterCAN`, `AltimeterDriver`, `AttitudeCAN`, `CANDebugNode`,
  `DCU`, `FuelGaugeCAN`, `HSICAN`, `HSIDriver`, `HandbrakeCAN`, `I2CBoard`,
  `MasterClock`, `MotionActor`, `MotionGateway`, `RPMGaugeCAN`,
  `RudderCAN`, `ServoBoard`, `StepperBoard`, `TransponderBoard`, `TurnCoordinatorCAN`,
  `VerticalSpeedCAN`, `WhiskeyCompassCAN`): independent PlatformIO/Arduino projects, each with its own
  `platformio.ini`, `include/`, `lib/`, `src/`, `test/`. Some (`DCU`, `MotionActor`, `MotionGateway`)
  have their own `CLAUDE.md` with board-specific detail — read it too when working in that directory.
  `AltimeterDriver` is the pre-CAN altimeter firmware, driven over USB by AirManager. It is superseded by
  `AltimeterCAN` and kept only as a fallback until the CAN board is confirmed on the rig.
  `HSIDriver` is likewise the pre-CAN HSI firmware, superseded by `HSICAN` and kept only as a fallback
  until the CAN board is confirmed on the rig — except the fallback is nominal only: the board's
  rewiring for `HSICAN` repurposed the pins `HSIDriver`'s `Configuration.h` expects (servos on
  D10–D12, hall sensors on D2/D8/A0), so `HSIDriver` can no longer actually be flashed.
  `StepperBoard` still carries the pre-CAN attitude-indicator driver (`GyroDrive`) next to its
  X25 motors; `AttitudeCAN` supersedes that part, and `TurnCoordinatorCAN` supersedes the two
  X25 channels for turn rate and slip. `StepperBoard` stays as the nominal fallback for both
  until the CAN boards are confirmed on the rig and keeps driving the remaining X25 gauges
  regardless.
- `shared/CANBase`: shared PlatformIO library with the CAN wire protocol (see Architecture below).
  Board projects pull it in via `lib_extra_dirs = ../shared` in `platformio.ini`.
- `DCUProviderPlugin`: X-Plane 12 plugin (C++/CMake, not PlatformIO) bridging DCU serial data into
  X-Plane. Has its own build/debug flow — see its README.
- `MotionProviderPlugin`: the second X-Plane 12 plugin (C++/CMake). Motion cueing for the 6DOF
  Stewart platform — reads flight datarefs, runs a washout filter, solves inverse kinematics and
  streams actuator setpoints to `MotionGateway` over USB serial. Has its own `CLAUDE.md`.
- `XPlaneSDK/`: the X-Plane SDK, vendored once and shared by every X-Plane plugin project
  (`DCUProviderPlugin`, `MotionProviderPlugin`). Their CMake/build scripts resolve it as
  `../XPlaneSDK`; don't re-vendor a per-plugin `SDK/` copy.
- `etc/`: reference material only (Fusion360 CAD, AirManager backups/cleartext LUA) — not source to edit.

## Commands

All board projects use PlatformIO CLI (`pio`), run from inside the project directory:

```bash
pio run                  # build default env
pio run -e <env>         # build a specific env (see platformio.ini for env names)
pio run -t upload        # flash to connected board
pio run -t upload -e <env>
pio device monitor -b 115200   # serial monitor (matches monitor_speed in platformio.ini)
pio test                 # PlatformIO unit tests — most board test/ dirs are still empty scaffolds;
                          # AirspeedCAN, AltimeterCAN, AttitudeCAN, DCU, HSICAN, RudderCAN,
                          # TurnCoordinatorCAN, VerticalSpeedCAN and WhiskeyCompassCAN have real
                          # Unity tests, run natively (no device needed):
pio test -e native       # runs test/test_* against that board's include headers
```

RudderCAN's and AirspeedCAN's `platformio.ini` factor the AVR-common settings (`platform`, `framework`,
`lib_deps`, ...) into an `[avr]` section that `env:nano` / `env:diecimilaatmega328` pick up via
`extends = avr`, rather than repeating them per env as DCU's does — copy this shape for any future board
that adds a native test env. The natively tested logic lives in Arduino-free headers under that board's
`include/` (`AdaptiveFilter.h`, `AxisMapping.h`, `AirspeedCalibration.h`, `VerticalSpeedCalibration.h`,
`AltimeterCalibration.h`, `CompassGeometry.h`, `HSIGeometry.h`, `Backlash.h`, `HSICalibration.h`, `AttitudeGeometry.h`,
`TurnCoordinatorCalibration.h`);
`src/` stays Arduino-coupled.

`DCUProviderPlugin` uses its own scripts instead of PlatformIO:

```bash
./build-macos.sh          # release build
./build-macos.sh debug    # debug build with symbols
./build-windows.sh        # native build on Windows (MSVC)
./build-xc-windows.sh     # cross-compile for Windows from macOS (MinGW)
./build-all.sh            # all platforms
```
Debug via VS Code + CodeLLDB (`.vscode/launch.json`: "Debug X-Plane Plugin" / "Attach to X-Plane").
Plugin logs go to X-Plane's `Log.txt` (`XPLMDebugString`).

## Architecture

### CAN protocol (`shared/CANBase`)

- `CanNodeId.h` / `CanMessageId.h`: the single source of truth for node IDs and message IDs on the bus.
  Any change here affects every board — keep in sync across projects.
- `BaseCAN`: low-level MCP2515 (SPI) wrapper. Interrupt-driven RX (`/INT` active-low); the ISR only sets
  a flag (no SPI/Serial from ISR).
- `InstrumentCAN` (extends `BaseCAN`): adds the gateway/instrument heartbeat protocol
  (`gatewayHeartbeat` / `instrumentHeartbeat`, timeout/discovery callbacks). Board-specific CAN classes
  subclass this and implement `instrumentBegin()` and `handleFrame()`. Fixed timing: instrument sends a
  heartbeat every 500ms; gateway is considered dead after 1500ms of silence.
- CAN std ID filtering: `CAN_STD_ID(id)` left-shifts the 11-bit message ID into the MCP2515's expected
  position; `MASK_EXACT = 0x07FF0000` matches all 11 ID bits exactly. Both must stay bit-compatible with
  whatever mask/filter setup `BaseCAN`/`InstrumentCAN` apply — mismatches silently drop or mis-route frames.
- `SerialMessageId.h`: the `MessageType` enum + payload structs for the DCU↔`DCUProviderPlugin` USB
  serial link (separate protocol from CAN, framed `0xAA 0x55 TYPE LEN PAYLOAD...` — see `DCU/CLAUDE.md`
  for the parser/gateway side). Adding a message type here means also updating `DCUReceiver`/`DCUSender`
  in `DCU` and the matching dataref plumbing in `DCUProviderPlugin/src/DataRefManager`.
- The whiskey compass is the simplest full-chain example to copy for a new read-only
  instrument: dataref in `DCUProviderPlugin/src/DataRefManager`, serial `0x0B` in
  `SerialMessageId.h`, `DCUReceiver::sendCompass()` in the DCU, CAN `0x107` consumed by
  `WhiskeyCompassCAN`. See `docs/superpowers/specs/2026-09-14-whiskey-compass-can-design.md`.
  The HSI is the bidirectional counterpart worth copying instead when a new instrument both
  consumes and drives datarefs: gateway→instrument `0x104`/`0x105` set heading/OBS/bug and CDI/GS
  deviation, while instrument→gateway `0x341` reports the OBS/HDG knob turns back. `0x341` sits
  inside the `0x340`–`0x34F` cluster-input block, which is otherwise documented (in `DCU/CLAUDE.md`)
  as resending every 5s — `0x341` is the exception: it carries encoder deltas, not absolute state,
  so resending it would double-apply a turn that already landed. See
  `docs/superpowers/specs/2026-09-30-hsi-can-design.md`.
  `AttitudeCAN` is the nested-axis variant: CAN `0x101` carries signed pitch/roll, and the
  board's `AttitudeGeometry.h` turns a display attitude into two coupled motor targets (the
  pitch motor rides on the roll frame). Its homing order — roll first, every roll move
  counter-moved on pitch, pitch last — is a mechanical constraint, not a style choice.
  See `docs/superpowers/specs/2026-10-09-attitude-can-design.md`.
  `TurnCoordinatorCAN` is the two-needle X27 variant: CAN `0x103` carries signed turn rate and
  slip, one AX1201728SG drives both motors through the vendored `vid6608` fork, and each needle
  has its own taught calibration table (VSI-style, 0 is a taught mark, not the home stop).
  See `docs/superpowers/specs/2026-10-09-turn-coordinator-can-design.md`.

### Motion chain (`MotionProviderPlugin` → `MotionGateway` → `MotionActor`)

The 6DOF Stewart motion platform is a **second, parallel chain** with its own protocol namespace.
It does not use `CanMessageId.h` / `CanNodeId.h` at all:

- `shared/CANBase/include/MotionMessageId.h` — motion message IDs
- `shared/CANBase/include/MotionNodeId.h` — `MotionNodeId` (gateway `0x00`, actors `0x01`–`0x03`)
  and `MotionActorState`

Mixing the two namespaces up is the easiest mistake to make here. The instrument bus and the motion
bus are separate physical buses; an ID that is free in one says nothing about the other.

Topology:

```
X-Plane ─ MotionProviderPlugin ──USB serial──▶ MotionGateway ──CAN──▶ 3× MotionActor
          (washout + IK, 60 Hz)               (Mega 2560)             (Nano, 2 Kangaroo
                                                                       channels each = 6 legs)
```

**Serial link (plugin → gateway).** Framed, CR-terminated, parsed in
`MotionGateway::handleSerialInput`. Two frame types share the `'B'` sync byte:

- `'B' 'C'` + reserved + 6 MSB + 6 LSB + `0x0D` (15 bytes) — the BFF Motion Driver "BIN2B" demand
  frame. Per actuator `demand = MSB*256 + LSB`, range 0..65280, centre 32640.
- `'B' 'G'` + 6 MSB + 6 LSB + duration_ms (BE, 2 bytes) + `0x0D` (17 bytes) — profiled goto, used
  for arm/disarm. Encoded by `BffEncoder::encodeGoto` in the plugin.

Gateway mode comes from pins 24/25 (`kMode1Pin`/`kMode2Pin`): `mode0` off (bytes drained and
discarded so the RX buffer can't overflow across a mode switch), `mode1` BFF, `mode2` SimTools.
Goto frames are valid in `mode1` only.

**CAN link (gateway → actor).** Command IDs must stay inside `0x380`–`0x38F`: the actors' RXB1 uses
a range filter (mask `0x7F0`, filter `0x380`), so anything outside is silently dropped.
`actorPairDemand` (`0x110`) is the exception, matched separately.

| ID | Message |
|---|---|
| `0x110` | `actorPairDemand` — streamed 60 Hz demand pair |
| `0x300` / `0x301` | gateway / actor heartbeat (own IDs; same 500 ms send, 1500 ms timeout as `InstrumentCAN`) |
| `0x380` / `0x381` | `actorPairHome` / `actorPairStop` |
| `0x382` | `actorPairGoto` — profiled arm/disarm move, `[0]=nodeId [1..2]=act1 [3..4]=act2 [5..6]=duration_ms [7]=reserved`, all BE |
| `0x385`, `0x38a`, `0x38b` | calibration move, save logical min/max |
| `0x386`–`0x388` | test bench start / abort / dump request |
| `0x3A0`–`0x3A2` | test telemetry actor → gateway (gateway RXB1: mask `0x7F0`, filter `0x3A0`) |

**Why goto exists.** Streaming a 60 Hz blend through `setDemands` turns every frame into its own
Kangaroo `p(pos, speed)` micro-move, which the actuator finishes early and then dwells on — a built-in
60 Hz velocity ripple that flight cues mask but a slow arm/disarm glide exposes. `actorPairGoto`
instead issues **one** profiled `p()` per channel, with the speed derived from a shared duration so
all six legs arrive together. Design and rollout: `docs/superpowers/specs/2026-08-27-goto-arm-disarm-design.md`.

**Arm switch.** A physical switch on the gateway (`kArmPin`) is reported back to the plugin in a
USB heartbeat every 500 ms; the plugin arms only while that heartbeat is fresh.

**Adding a motion message ID** touches: `MotionMessageId.h`, the gateway's send path plus filter setup,
and `MotionActor`'s `CAN::handleFrame`. If it is also visible on the serial link, add the encoder in
`MotionProviderPlugin/src/BffEncoder.*` and the send path in `SerialLink` as well. Flashing is lockstep —
gateway and all three actors must run matching firmware.

### Per-board file layout

Each board's `src/` follows the same convention:

- `main.cpp` — `setup()`/`loop()`; switches between real CAN mode and `BenchDebug` simulation mode via
  the `BENCHDEBUG` flag in `Configuration.h`.
- `CAN.cpp/h` — board-specific subclass of `InstrumentCAN`; parses/sends this board's CAN frames.
- `Configuration.h` — pins, this board's `CanNodeId`, `BENCHDEBUG`/`DEBUGLOG_ENABLE` flags.
- `DebugLog.h(.cpp)` — serial debug print macros, gated by `DEBUGLOG_ENABLE`.
- `BenchDebug.cpp/h` — serial-console driven simulation mode for testing the board's logic on a bench
  without the full rig/CAN bus wired up.
- `<Device>.cpp/h` (e.g. `Handbrake`, `FuelGauge`, `Transponder`) — the actual hardware/business logic,
  independent of CAN/BenchDebug.
- `AirManager.cpp/h` (where present — `I2CBoard`, `MasterClock`, `ServoBoard`, `StepperBoard`,
  `TransponderBoard`) — integration with X-Plane's AirManager gauge-output protocol, used instead of
  or alongside CAN.

When adding a new board, follow this same file layout rather than inventing a new one.
