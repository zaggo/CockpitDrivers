# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

See also the monorepo-level `../CLAUDE.md` for repo-wide conventions (CAN protocol basics, per-board
file layout). This file covers what's specific to DCU.

## Commands

Run from this directory:

```bash
pio run                      # build (env: megaatmega2560)
pio run -t upload            # flash to connected Mega 2560
pio device monitor -b 115200 # serial monitor (debug log is on Serial1, not the USB Serial link)
```

`test/` is an empty PlatformIO scaffold — no tests exist yet.

## What DCU is

DCU (Data Communication Unit) is the **CAN gateway node**, not an instrument. It bridges two links:

- **CAN bus** — talks to instrument boards (fuel gauge, transponder, handbrake, etc.).
- **USB Serial** (`Serial`) — talks to `DCUProviderPlugin` inside X-Plane, using a custom framed
  protocol (not AirManager).

Debug logging goes to `Serial1` (`DebugLog.h`), kept separate from `Serial` since that port carries the
live plugin protocol.

## CAN side (`CAN.cpp/h`)

Unlike instrument boards, `CAN` subclasses `BaseCAN` directly instead of `InstrumentCAN` — DCU *is* the
gateway, so it implements the other half of the heartbeat protocol described in `shared/CANBase`:

- Sends `gatewayHeartbeat` (0x300) every 500ms.
- Tracks `instrumentHeartbeat` (0x301) per node in `lastInstrumentHeartbeatMs[nodeId]`
  (`kMaxInstrumentNodes = 16`), and flags a node dead after 1500ms of silence
  (`checkInstrumentHeartbeats()`).
- Filters/decodes `transponderInput` (0x311) and `handbrakeStatus` (0x330) frames from instruments and
  forwards them to the plugin via `DCUSender`.
- CAN alarm LED (`kCANAlarmPin`) lights if CAN isn't started, or if any tracked CAN ID has an
  outstanding TX/RX/heartbeat-timeout error — tracked in the fixed-size `canIdErrors[]`
  (`kMaxCanIdErrors = 8`, linear scan, no dynamic allocation per Arduino convention).
- `MASK_EXACT`/`CAN_STD_ID` filter setup in `CAN::begin()` must stay in sync with any new message IDs
  added to `CanMessageId.h`.

## Serial side (`DCUReceiver`/`DCUSender`, `SerialMessageId.h`)

Frame format on `Serial`: `0xAA 0x55 TYPE LEN PAYLOAD...` — `TYPE` is `MessageType` from
`shared/CANBase/include/SerialMessageId.h`.

- **`DCUReceiver`**: byte state machine parsing frames from the plugin (`SerialMessageFuel`,
  `SerialMessageLights`, `SerialMessageTransponder`), converts them to CAN messages and sends them
  onward to instruments via `CAN::sendMessage`. Caches last-sent values per message type and resends on
  a 5000ms max-age timer (`checkMaxAgeResync`) even without new plugin input, so instruments recover
  after a dropped frame or a restart.
- **`DCUSender`**: formats outbound frames back to the plugin — used by `CAN` to relay instrument-side
  input (transponder knob commands, handbrake status) that arrived over CAN.
- Adding a new serial message type touches three places: `SerialMessageId.h` (enum + payload struct),
  `DCUReceiver::handleFrame` (plugin → CAN direction) or `CAN::handleFrame`/`updateXxx` +
  `DCUSender::sendFrame` call site (CAN → plugin direction).

## BenchDebug mode

`Configuration.h`'s `BENCHDEBUG` flag swaps `DCUReceiver` out for `BenchDebug` in `main.cpp`: a
serial-console simulator that drives fuel/light CAN messages directly, for testing instruments on the
CAN bus without the plugin/X-Plane attached.
