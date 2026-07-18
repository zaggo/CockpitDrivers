# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Firmware + tooling for a home-cockpit flight-sim rig: each top-level directory is an Arduino-based
instrument/actuator board (altimeter, HSI, transponder, handbrake, servo/stepper actuators, fuel gauge,
master clock, etc.) that talks to a central DCU (Data Communication Unit) over CAN bus, and/or to
X-Plane via AirManager. `DCUProviderPlugin` is the X-Plane-side counterpart.

## Repo layout

- Board projects (`AltimeterDriver`, `CANDebugNode`, `DCU`, `FuelGaugeCAN`, `HSIDriver`, `HandbrakeCAN`,
  `I2CBoard`, `MasterClock`, `MotionActor`, `MotionGateway`, `RPMGaugeCAN`, `ServoBoard`, `StepperBoard`,
  `TransponderBoard`): independent PlatformIO/Arduino projects, each with its own
  `platformio.ini`, `include/`, `lib/`, `src/`, `test/`. Some (e.g. `DCU`) have their own `CLAUDE.md`
  with board-specific detail — read it too when working in that directory.
- `shared/CANBase`: shared PlatformIO library with the CAN wire protocol (see Architecture below).
  Board projects pull it in via `lib_extra_dirs = ../shared` in `platformio.ini`.
- `DCUProviderPlugin`: X-Plane 12 plugin (C++/CMake, not PlatformIO) bridging DCU serial data into
  X-Plane. Has its own build/debug flow — see its README.
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
                          # DCU has real Unity tests, run natively (no device needed):
pio test -e native       # (from DCU/) runs DCU/test/test_* against DCU/include headers
```

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
