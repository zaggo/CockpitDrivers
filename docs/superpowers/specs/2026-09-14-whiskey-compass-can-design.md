# WhiskeyCompassCAN — Design

Date: 2026-09-14
Status: approved, ready for implementation plan

## Purpose

Add a new instrument board, `WhiskeyCompassCAN`, driving the standby ("whiskey")
magnetic compass of the cockpit. The compass card is a full 360° rotating disc on a
28BYJ-48 stepper; the instrument also carries up to three optional panel-light LEDs.

The board is a pure consumer: it receives a magnetic heading over CAN and has no
inputs of its own beyond its homing sensor.

Nothing of this chain exists yet — neither the node ID, the CAN message, the serial
message, nor the dataref plumbing — so the work spans the whole path from X-Plane to
the stepper.

## Topology

```
X-Plane ── DCUProviderPlugin ──USB serial 0x0B──▶ DCU ──CAN 0x107──▶ WhiskeyCompassCAN
           compass_heading_deg_mag               (Mega 2560)        (Nano, 28BYJ-48 + 1..3 LEDs)
```

## Protocol additions (`shared/CANBase`)

These three headers are the single source of truth for the whole rig, so they change
first and every other part of the work reads from them.

### `CanNodeId.h`

```cpp
compassNodeId = 0x0A
```

Follows `altimeterNodeId = 0x09`. Matches the device table on the rig documentation
(Compass = 10).

### `CanMessageId.h`

```cpp
// 0x107: Magnetic compass (Gateway -> Instrument, 50Hz)
// [0..1] heading uint16, degrees * 100, 0..35999
//        (sim/cockpit2/gauges/indicators/compass_heading_deg_mag)
// [2..7] reserved
compass = 0x107
```

Big-endian on the wire like every other multi-byte CAN field in this repo.

### `SerialMessageId.h`

```cpp
// Plugin -> DCU. Payload: float headingDegMag (4 bytes, host order), degrees magnetic.
SerialMessageCompass = 0x0B
```

Shaped after `SerialMessageAirspeed = 0x08`: a bare float, host byte order, no packed
struct needed for a single scalar.

## Hardware

Arduino Nano (ATmega328), MCP2515 CAN transceiver on hardware SPI, ULN2003 driver
board for the stepper, up to three LEDs.

| Pin | Function | Rationale |
|---|---|---|
| D11 / D12 / D13 | SPI MOSI / MISO / SCK | hardware SPI, fixed |
| D10 | MCP2515 `/CS` | D10 is the AVR `/SS` pin and must be an output in master mode; using it as chip select means `SPI.begin()` drives it. Left floating it can drop SPI out of master mode. |
| D2 | MCP2515 `/INT` | `BaseCAN` hangs its RX handler off `attachInterrupt()`, so `/INT` must be D2 or D3 (INT0/INT1). D2 is free here — unlike on AltimeterCAN, no sensor needs it. |
| D4, D7, D8, D9 | ULN2003 IN1..IN4 | purely digital; deliberately kept off the PWM pins so all three PWM outputs stay available for the LEDs |
| D3, D5, D6 | LED 1..3 | the three PWM pins left over (Timer2, Timer0, Timer0). D9/D10/D11 PWM is unusable: D10/D11 are SPI, D9 is a stepper pin. |
| A0 | Hall sensor, compass card zero | `INPUT_PULLUP`, active LOW, polled (no interrupt needed) |

No Servo library and no MCP23017 on this board, so Timer1 stays untouched and
`lib_deps` needs only `coryjfowler/mcp_can`.

LED count is a soldering decision, fixed at compile time:

```cpp
const uint8_t kLightCount = 3;                 // 1..3
const uint8_t kLightPins[kLightCount] = {3, 5, 6};
```

`setBrightness()` writes the same PWM value to all configured pins. All three LEDs
share one brightness; per-LED control is explicitly out of scope.

## Project layout

Follows the shape of the newer `xxxCAN` boards (`AltimeterCAN`, `VerticalSpeedCAN`):

```
WhiskeyCompassCAN/
  platformio.ini                  [avr] section + env:nano + env:native
  lib/CheapStepper/               vendored stepper driver (see below)
    library.json
    src/CheapStepper.cpp
    src/CheapStepper.h
  include/CompassGeometry.h       Arduino-free maths, natively tested
  test/test_geometry/
    test_geometry.cpp             Unity tests
  src/main.cpp                    setup()/loop(), BENCHDEBUG switch
  src/Configuration.h             pins, node ID, flags
  src/DebugLog.h                  serial debug macros
  src/WhiskeyCompass.cpp/.h       stepper + homing state machine + EEPROM + LEDs
  src/CAN.cpp/.h                  InstrumentCAN subclass
  src/BenchDebug.cpp/.h           serial console simulation mode
```

### `platformio.ini`

```ini
[avr]
framework = arduino
platform = atmelavr
monitor_speed = 115200
lib_deps =
    coryjfowler/mcp_can@^1.5.1
lib_extra_dirs =
  ../shared

[env:nano]
extends = avr
board = nanoatmega328new

[env:native]
platform = native
test_framework = unity
build_src_filter = -<*>
```

The `[avr]` + `extends` shape is the one CLAUDE.md points at for new boards.

### Vendored `CheapStepper`

`CheapStepper` currently exists as three hand-copied versions in `AltimeterDriver/src`,
`HSIDriver/src` and `AltimeterCAN/src`. All three `#include <MCP23017.h>` because of a
`patternOut` constructor that writes the step pattern into an I/O-expander port buffer.

This board drives four pins directly and has no expander. The copy therefore goes into
`lib/CheapStepper/` — the way `VerticalSpeedCAN` vendors its `vid6608` fork — with the
`MCP23017` include and the `patternOut` constructor and its two branches removed. That
keeps `blemasle/MCP23017` out of `lib_deps` entirely.

Consolidating all four copies into `shared/` was considered and rejected: it drags a
refactor of three unrelated, currently-working boards into this project's scope. The
duplication is noted as existing debt, not fixed here.

## `CompassGeometry.h` — Arduino-free logic

Deliberately free of Arduino headers so it can be exercised by the native Unity tests,
mirroring `AltimeterCAN/include/AltimeterCalibration.h`.

Contents:

- `normalizeZeroAdjust(int32_t degrees) -> int16_t` — folds a degree value into
  -180..179 so repeated calibration passes cannot accumulate whole turns.
- `accumulateZeroAdjust(int16_t stored, int32_t jogDegrees) -> int16_t` — adds a jog
  onto the stored offset (adds, never replaces — replacing discards every previous
  pass and makes the card wander further each time).
- `jogDegreesFromPosition(uint32_t position, uint32_t totalSteps) -> int32_t` — the
  stepper reports its position normalised into `0..totalSteps-1`, so a small
  counter-clockwise jog comes back as nearly a full turn; this folds it onto the short
  way round.
- `degreesToSteps(double degrees, uint32_t totalSteps) -> uint32_t` — heading to
  normalised step position.
- `shortestPathSteps(uint32_t current, uint32_t target, uint32_t totalSteps) -> int32_t`
  — signed step delta taking the short way round, so 359° → 1° is +2° of travel rather
  than -358°. Positive is clockwise. At exactly half a turn the tie is broken
  **clockwise**, fixed by test so it cannot drift with a refactor.

The first three are the same functions `AltimeterCalibration.h` already defines. They
are re-declared here rather than shared: the two boards' `include/` trees are
independent by repo convention, and pulling one board's header into another would
couple two otherwise unrelated projects.

## `WhiskeyCompass` — device logic

Owns everything hardware-facing except CAN.

```cpp
class WhiskeyCompass {
public:
    enum HomingPhase { unknown, leaveZero, searchZero, searchZeroEnd,
                       returnToZeroEnd, searchZeroStart, moveToTrueZero,
                       moveToAdjustedZero, homed, timeout };

    WhiskeyCompass();

    bool isHomed = false;

    void loop();
    void beginHoming();
    bool isHoming() const;

    void moveToHeading(double degMag);
    void setBrightness(uint8_t pwm);

    bool calibrateZero();
    void wipeCalibration();
    int16_t zeroAdjustDegree() const;

    void stop();
    void off();
};
```

### Motion

One `CheapStepper` on the four ULN2003 pins, `totalSteps = 4096` (`kTotalSteps` in
`Configuration.h` — change it there if a gear reduction sits between motor and card;
the maths is ratio-agnostic). RPM window 10..14 as on `AltimeterCAN`: below ~6 rpm the
28BYJ-48 overheats, above ~23 rpm it skips steps.

That RPM ceiling *is* the rate limiter. No additional low-pass filter: the motor's own
inertia and step rate already produce the lag a wet compass has. A damping filter stays
a possible follow-up if the card looks twitchy on the rig.

`moveToHeading(deg)`:

1. target position = `degreesToSteps(deg + zeroAdjustDegree(), kTotalSteps)`
2. delta = `shortestPathSteps(stepper.getPosition(), target, kTotalSteps)`
3. issue a non-blocking `newMove` for that delta

A new target arriving mid-move replaces the old one, recomputed from the current
position. No queueing — at 50 Hz a queue would only accumulate stale headings.

### Homing

A non-blocking state machine driven by `loop()`, with the same phases `Altimeter` uses:
`leaveZero → searchZero → searchZeroEnd → returnToZeroEnd → searchZeroStart →
moveToTrueZero → moveToAdjustedZero → homed`, plus a `timeout` phase.

It walks the full width of the Hall window and takes its **midpoint** as mechanical
zero, rather than the first edge it sees — the edge position depends on approach
direction and makes homing non-repeatable.

Non-blocking is load-bearing, not a style choice: `HSIDriver` homes synchronously and a
run takes several seconds, well past the 1500 ms both ends of the heartbeat protocol use
to declare each other dead. A blocking run here would have the DCU drop the instrument
mid-startup.

### Zero calibration (EEPROM)

The Hall sensor's mechanical zero is not the card's painted N mark. The offset between
them lives in EEPROM so it can be taught on the bench without a reflash:

```cpp
struct CompassConfig {
    uint16_t magic;              // identifies a valid, matching layout
    uint8_t  version;
    int16_t  zeroAdjustDegree;   // -180..179
};
```

A wrong or missing magic/version falls back to `kDefaultZeroAdjustDegree` from
`Configuration.h` and rewrites the block, so a freshly flashed board comes up sane.

- `calibrateZero()` — requires `isHomed`. Takes the card's current position as the jog
  off zero (`jogDegreesFromPosition`), adds it onto the stored offset
  (`accumulateZeroAdjust`), persists, and re-zeroes the card.
- `wipeCalibration()` — factory reset to the compiled-in default.

## `CAN` — transport layer

`InstrumentCAN` subclass, `kNodeId = CanNodeId::compassNodeId`.

### Receive

Three exact IDs, all masks `MASK_EXACT` (`0x07FF0000`, all 11 ID bits significant).
The MCP2515 offers two masks and six filters; the filter layout mirrors
`AltimeterCAN`'s, doubling up IDs on RXB1 rather than leaving slots open:

- RXB0 (mask 0): filter 0 = `0x107`, filter 1 = `0x300`
- RXB1 (mask 1): filter 2 = `0x203`, filter 3 = `0x300`, filter 4 = `0x203`,
  filter 5 = `0x300`

| ID | Handling |
|---|---|
| `0x107` `compass` | `len >= 2`; `deg100 = BE16(data[0..1])`; `moveToHeading(deg100 / 100.0)` |
| `0x203` `lights` | `len >= 8`; `panelDim1000 = BE16(data[0..1])`; `setBrightness(constrain(panelDim1000/1000.0, 0, 1) * 255)` |
| `0x300` `gatewayHeartbeat` | handled by `InstrumentCAN` |

### Transmit

`instrumentHeartbeat` (`0x301`) with node ID `0x0A` every 500 ms — emitted by
`InstrumentCAN`, no board-specific TX path. The whiskey compass has no inputs to
report.

### Heartbeat hooks

- `onGatewayHeartbeatDiscovered()` — LEDs on, and start homing if not homed and not
  already homing. First contact with the gateway is what starts the card: homing before
  that would drive the instrument with no sim running.
- `onGatewayHeartbeatTimeout()` — brightness to 0.

Same behaviour as `AltimeterCAN::CAN`.

## `BenchDebug` — bench console

Serial-console simulation, selected by `BENCHDEBUG` in `Configuration.h`, for testing
without a CAN bus. Commands:

| Command | Effect |
|---|---|
| `ho` | start homing |
| `hd<deg>` | move card to heading |
| `li<0..255>` | set LED brightness |
| `cz` | calibrate zero at current position |
| `cw` | wipe calibration to defaults |
| `st` | stop / de-energise the stepper |
| `?` | help |

## DCU side

### `DCU/src/DCUReceiver`

- New `MessageMeta compassMeta = {0, 5000}`, alongside `airspeedMeta`.
- New `case MessageType::SerialMessageCompass` in the parser: read the float, wrap and
  clamp to `0..35999` in degrees * 100, store, call `sendCompass()`.
- `sendCompass()` — pack the big-endian `uint16` into `data[0..1]`, zero `data[2..7]`,
  `canBus->sendMessage(CanMessageId::compass, 8, data)`, stamp
  `compassMeta.lastSendTimestamp`.
- Stale check in the periodic sweep: resend after 5 s of silence, like airspeed. This
  is how the instrument recovers its heading after a plugin restart.

Wrapping matters: `compass_heading_deg_mag` can read slightly negative or above 360
depending on X-Plane's internal state, and an unwrapped negative float cast to
`uint16` would send the card on a full spurious turn.

### `DCU/src/BenchDebug`

New `co<deg>` command plus its help line, matching the existing `as<knots>` shape.

## Plugin side

`DCUProviderPlugin/src/DataRefManager`: read
`sim/cockpit2/gauges/indicators/compass_heading_deg_mag` and send
`SerialMessageCompass` on the plugin's existing send cadence, alongside the airspeed
and altimeter/VSI messages.

## Testing

**Native unit tests** (`pio test -e native`, `test/test_geometry/`) against
`include/CompassGeometry.h`:

- shortest path: 359 → 1 is +2°, 1 → 359 is -2°, 10 → 200 goes the short way, exactly
  180° resolves clockwise
- `degreesToSteps` at 0, 90, 180, 359.99, and for a non-4096 `totalSteps`
- zero-adjust accumulation over repeated passes stays in -180..179 and does not drift
  by whole turns
- `jogDegreesFromPosition` folds a near-full-turn position onto a small negative jog

**Hand verification on the bench** (`BENCHDEBUG=1`), not automated — consistent with
the rest of the repo, which has no hardware-in-the-loop harness:

- homing repeatability: home ten times, card lands on the same mark
- homing does not block: heartbeat keeps flowing through a homing run (verify with the
  real gateway, not just the console)
- wrap-around: step the heading across 0° in both directions, card never takes the long
  way
- LED dimming across the full `0x203` range
- zero calibration survives a power cycle

## Open items, to settle on the bench

- **Card rotation direction.** Whether the card turns with or against the heading
  depends on the mechanical build. Handled by an `inversed` flag in `Configuration.h`,
  default `false`, flipped once observed.
- **Gear ratio.** Assumed 1:1, i.e. 4096 steps per card revolution. If there is a
  reduction, only `kTotalSteps` changes; none of the maths does.

## Out of scope

- Per-LED brightness control (all LEDs share one value by requirement).
- A liquid-damping low-pass filter on the heading (deferred until the rig shows it is
  needed).
- Consolidating the four `CheapStepper` copies into `shared/` (separate refactor
  touching three working boards).
