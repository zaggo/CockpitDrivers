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

```bash
pio test -e native            # run Unity tests against DCU/include headers (no device needed)
```

`test/` has real Unity suites (`test_can_id_error`, `test_command_tokenizer`, `test_heartbeat`,
`test_serial_frame_parser`, `test_wire_encoding`) covering the header-only helpers in `DCU/include/`
(see below) — the `env:native` PlatformIO env builds/runs them on the host.

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
- Filters/decodes `transponderInput` (0x311), `handbrakeStatus` (0x330) and `rudder` (0x303) frames from
  instruments and forwards them to the plugin via `DCUSender`. Note `rudder` arrives big-endian on CAN
  but the serial `RudderToDcuMessage` struct is host order, so `updateRudder()` converts via
  `unpackBE16` instead of reinterpret_cast'ing the CAN buffer (unlike `updateTransponder`).
- CAN alarm LED (`kCANAlarmPin`) lights if CAN isn't started, or if any tracked CAN ID has an
  outstanding TX/RX/heartbeat-timeout error — tracked in the fixed-size `canIdErrors[]`
  (`kMaxCanIdErrors = 12`, linear scan, no dynamic allocation per Arduino convention).
- `MASK_EXACT`/`CAN_STD_ID` filter setup in `CAN::begin()` must stay in sync with any new message IDs
  added to `CanMessageId.h`.

## Serial side (`DCUReceiver`/`DCUSender`, `SerialMessageId.h`)

Frame format on `Serial`: `0xAA 0x55 TYPE LEN PAYLOAD...` — `TYPE` is `MessageType` from
`shared/CANBase/include/SerialMessageId.h`.

- **`DCUReceiver`**: byte state machine (via `SerialFrameParser`) parsing frames from the plugin
  (`SerialMessageFuel`, `SerialMessageLights`, `SerialMessageTransponder`, `SerialMessageHandbrake`,
  `SerialMessageRPM`, `SerialMessageOdometer`, `SerialMessageAirspeed`, `SerialMessageAltimeterVsi`),
  converts them to CAN messages and sends them
  onward to instruments via `CAN::sendMessage`. Caches last-sent values per message type and resends on
  a 5000ms max-age timer (`checkMaxAgeResync`) even without new plugin input, so instruments recover
  after a dropped frame or a restart.
- **`DCUSender`**: formats outbound frames back to the plugin — used by `CAN` to relay instrument-side
  input (transponder knob commands, handbrake status) that arrived over CAN.
- Adding a new serial message type touches three places: `SerialMessageId.h` (enum + payload struct),
  `DCUReceiver::handleFrame` (plugin → CAN direction) or `CAN::handleFrame`/`updateXxx` +
  `DCUSender::sendFrame` call site (CAN → plugin direction).

## Header-only helpers (`DCU/include/`)

Unit-tested independent of Arduino/hardware (see `env:native` above):

- `WireEncoding.h` — `packBE16`/`unpackBE16` and `packBE32`/`unpackBE32` big-endian pack/unpack for
  CAN/serial payloads.
- `CanIdError.h` — `CanIdError`/`CanErrorType` fixed-size error record + `anyCanIdHasError()`; backs
  `CAN`'s `canIdErrors[]` alarm-LED logic above.
- `Heartbeat.h` — `heartbeatAlive()`/`isStale()`; rollover-safe `millis()` comparisons shared by the
  CAN heartbeat timeout and serial max-age resync logic.
- `CommandTokenizer.h` — `tokenizeCommands()`; in-place space-splitting for `BenchDebug`'s serial
  console commands.
- `SerialFrameParser.h` — byte-in/frame-out state machine for the `0xAA 0x55 TYPE LEN PAYLOAD...`
  framing, used by `DCUReceiver`.

## BenchDebug mode

`Configuration.h`'s `BENCHDEBUG` flag swaps `DCUReceiver` out for `BenchDebug` in `main.cpp`: a
serial-console simulator that drives fuel/light/RPM/odometer/airspeed CAN messages directly, for
testing instruments on the CAN bus without the plugin/X-Plane attached. `?` lists the commands
(`lt`/`rt`/`cl`/`rp`/`oh`/`as`/`al`/`vs`/`rw`); `as<knots>` sends an `airspeed` (0x100) frame to
AirspeedCAN. `al<feet>` and `vs<fpm>` both resend the same `altimeterVsi` (0x102) frame — altitude and
climb rate share one message, so each command updates its half and ships both.

Because no `DCUSender` exists in bench builds, instrument→plugin frames decoded by `CAN` have no sink.
For rudder input the `rw` console command works around that: `CAN` keeps the last decoded
`RudderToDcuMessage` in a `#if BENCHDEBUG` slot that `BenchDebug` drains via `takeRudderSample()` and
prints (changed values only) until any key is pressed. Other instrument→plugin messages
(transponder, handbrake) are still dropped in bench mode.
