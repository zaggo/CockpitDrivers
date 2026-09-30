# HSICAN — Design

Date: 2026-09-30
Status: approved, ready for implementation plan

## Purpose

Convert the HSI (Horizontal Situation Indicator) from the AirManager-driven
`HSIDriver` firmware to a CAN instrument, as `AltimeterCAN` did for the altimeter.
A new project `HSICAN` is created alongside `HSIDriver`; AirManager dependencies are
removed, CAN is added, and the driver is modernised on the way.

Unlike the whiskey compass, the HSI is **bidirectional**: it consumes six datarefs
and drives two of them back from its knobs. Nothing of the chain exists yet — no node
ID, no CAN messages, no serial messages, no dataref plumbing — so the work spans the
whole path from X-Plane to the needles.

The board also needs a **pin remap**: the Nano currently uses the hardware SPI pins
for servos, so the MCP2515 has nowhere to sit.

`HSIDriver` stays in the tree, but see "Rollback" below — after the rewiring it is no
longer flashable as a fallback.

## Topology

```
X-Plane ── DCUProviderPlugin ──USB serial 0x0C/0x0D──▶ DCU ──CAN 0x104/0x105──▶ HSICAN
           6 datarefs read                             (Mega 2560)              (Nano)
                        ◀── USB serial 0x0E ────────────── ◀── CAN 0x341 ──────
           2 datarefs written                                (knobs)
```

## Instrument mechanics

Three 28BYJ-48 stepper axes on geared drives, four servos, two rotary encoders with
push buttons, three hall sensors for homing, an MCP23017 port extender, and panel
lighting.

| Axis / actuator | Drives | Driven by |
|---|---|---|
| `compass` stepper | the rotating compass card | `heading_electric_deg_mag_pilot` |
| `obs` stepper | the course pointer, riding on the card | `hsi_obs_deg_mag_pilot` |
| `hdg` stepper | the heading bug, riding on the card | `heading_dial_deg_mag_pilot` |
| `cdiServo` | the lateral deviation bar | `nav1_hdef_dot` |
| `fromToServo` | TO/FROM flag, three discrete positions | `nav1_fromto` |
| `gs1Servo`, `gs2Servo` | the glideslope needle, driven from both ends | `nav1_vdef_dot` |

`gs1Servo` and `gs2Servo` are **one logical channel**: `HSIDriver/src/HSI.cpp:285-288`
feeds both with the same offset, only the mechanical linkage differs
(`kServoAdjustRatio` 2.0 against -10/7).

## Renaming

`HSIDriver` carries six identifiers that name the wrong thing, and this port is the
only moment they are free to fix. They are renamed in `HSICAN`; `HSIDriver` is left
alone.

| Old | New | Why |
|---|---|---|
| `ServoId::vsi1Servo`, `vsi2Servo` | `gs1Servo`, `gs2Servo` | "VSI" here is the glideslope, not vertical speed |
| `ServoId::vorServo` | `cdiServo` | it is the CDI deviation bar and works on a localizer too, not only a VOR |
| `HSIAxis::cdi` | `obs` | it is the course pointer, following `hsi_obs_deg_mag_pilot`; the CDI is a servo |
| `kCDIEncoder*` | `kObsEncoder*` | that knob sets the OBS course |
| `kCompEncoder*` | `kHdgEncoder*` | that knob sets the **heading bug**, not the compass card |

The VSI one matters beyond tidiness: `VerticalSpeedCAN`, `CanMessageId::altimeterVsi`,
`SerialMessageAltimeterVsi` and `vsiFpm` are genuine vertical speed elsewhere in this
repo. Two meanings for one abbreviation on one bus is the same class of confusion the
root `CLAUDE.md` already warns about for the two CAN namespaces.

`kCompEncoder*` is the most misleading of the set — that knob has nothing to do with
the compass card. The old LUA confirms it: `SetDegreesCompass` writes
`heading_dial_deg_mag_pilot`, and `0x341` control = 1 is HDG for the same knob.

The renames pull in the ISR state (`obsEncoderState`, `hdgEncoder`,
`currentHdgEncoderButtonState`, `lastHdgDebounceTime`, …), the `axisName()` and
`servoName()` strings, the homing log lines and the comments in `Configuration.h`.
No protocol impact — `0x104`, `0x105` and `0x341` are unchanged.

## Protocol additions (`shared/CANBase`)

These headers are the single source of truth for the whole rig, so they change first.

### `CanNodeId.h`

```cpp
hsiNodeId = 0x0B
```

### `CanMessageId.h`

**`hsiHeading = 0x104`** — Gateway → Instrument, 50 Hz

| Bytes | Field | Type | Source dataref |
|---|---|---|---|
| 0..1 | compass heading, deg × 100, 0..35999 | uint16 BE | `sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot` |
| 2..3 | OBS course, deg × 100 | uint16 BE | `sim/cockpit2/radios/actuators/hsi_obs_deg_mag_pilot` |
| 4..5 | HDG bug, deg × 100 | uint16 BE | `sim/cockpit2/autopilot/heading_dial_deg_mag_pilot` |
| 6 | flags | uint8 | see below |
| 7 | reserved | | |

Flags byte:

| Bit | Meaning | Source |
|---|---|---|
| 0 | TO | `sim/cockpit/radios/nav1_fromto == 1` |
| 1 | FROM | `sim/cockpit/radios/nav1_fromto == 2` |
| 2 | GS valid | `sim/cockpit/radios/nav1_CDI != 0` |
| 3..7 | reserved | |

`nav1_fromto == 0` leaves bits 0 and 1 clear — the "no nav" case.

`nav1_CDI` is confirmed from Laminar's own dataref list
(`X-Plane/XPlane2Blender/io_xplane2blender/resources/DataRefs.txt`):

```
sim/cockpit/radios/nav1_CDI  int  n  bool  Are we receiving an expected glide slope for nav1
```

The name is misleading — it reads like a CDI flag but is documented as glideslope
reception. It is read-only. `sim/cockpit/radios/glideslope_signal_valid`, considered
during design, does **not** exist in any Laminar source and must not be used.

**`hsiNav = 0x105`** — Gateway → Instrument, 10 Hz

| Bytes | Field | Type | Source dataref |
|---|---|---|---|
| 0..1 | CDI deviation, dots × 1000 | int16 BE | `sim/cockpit/radios/nav1_hdef_dot` |
| 2..3 | GS deviation, dots × 1000 | int16 BE | `sim/cockpit/radios/nav1_vdef_dot` |
| 4..7 | reserved | | |

`int16` at dots × 1000 spans ±32.7 dots; `nav1_hdef_dot` reaches ±2.5. The dots →
servo-degree conversion happens on the board and is calibratable.

**`hsiInput = 0x341`** — Instrument → Gateway, onChange

| Bytes | Field | Type |
|---|---|---|
| 0 | control: 0 = OBS, 1 = HDG | uint8 |
| 1 | action: 0 = delta, 1 = push | uint8 |
| 2..3 | delta in encoder clicks, signed (0 when action = push) | int16 BE |
| 4 | pushState 0/1 (0 when action = delta) | uint8 |
| 5..7 | reserved | |

**Exception to the block convention, and it must be stated in the header comment.**
`CanMessageId.h` documents the `0x340..0x34F` cluster-input block as sending an
unconditional resend every 5 s so the sim recovers knob positions after a restart.
`0x341` carries **deltas** and must therefore **never** resend — a resend would apply
the clicks twice. It sends onChange only.

Delta rather than absolute is deliberate: the OBS stepper already follows the sim's
OBS value via `0x104` bytes 2..3, so the loop closes through the sim and the board
holds no state of its own. No seed value after homing is needed (the old AirManager
chain needed `SetInitialDegrees` for exactly that reason), and a GPS or autopilot
changing OBS on the sim side causes no fight.

The DCU's existing range filter (`MASK_RANGE`, `DCU/src/CAN.cpp:70`) already covers
`0x341`. No gateway filter change.

### `SerialMessageId.h`

```cpp
SerialMessageHsiHeading = 0x0C,   // Plugin -> DCU
SerialMessageHsiNav     = 0x0D,   // Plugin -> DCU
SerialMessageHsiInput   = 0x0E,   // DCU -> Plugin
```

```cpp
struct __attribute__((packed)) HsiHeadingMessage {    // 13 bytes, host order
    float   headingDegMag;   // heading_electric_deg_mag_pilot
    float   obsDeg;          // hsi_obs_deg_mag_pilot
    float   hdgBugDeg;       // heading_dial_deg_mag_pilot
    uint8_t flags;           // bit0 TO, bit1 FROM, bit2 GS valid
};

struct __attribute__((packed)) HsiNavMessage {        // 8 bytes, host order
    float cdiDots;
    float gsDots;
};

struct __attribute__((packed)) HsiInputToDcuMessage { // 5 bytes, host order
    uint8_t control;
    uint8_t action;
    int16_t delta;
    uint8_t pushState;
};
```

Both `_MSC_VER` and GCC variants, as the existing structs in that header do.

### Link budget

Serial: `hsiHeading` 50 Hz × 17 bytes = 850 B/s, `hsiNav` 10 Hz × 12 bytes = 120 B/s.
Existing traffic is roughly 2.1 kB/s and 115200 baud carries 11.5 kB/s, so the link
lands near 27 % utilisation.

CAN: five 50 Hz frames (`0x100`, `0x102`, `0x104`, `0x106`, `0x107`) plus one at 10 Hz
(`0x105`) and the heartbeats at 500 kbit/s stays under 8 %.

## Hardware — pin remap

The MCP2515 needs five pins: D11 MOSI, D12 MISO, D13 SCK (hardware SPI, not
relocatable), `/CS` and `/INT`. On the current board D11 and D12 carry servos, D13 is
free, and every remaining pin is taken.

`/INT` is genuinely load-bearing even though the ISR is not. `InstrumentCAN.cpp:80-92`
gates its fast path on `digitalRead(intPin) == HIGH` and drains with
`while (digitalRead(intPin) == LOW)`. The ISR only sets a flag. Proof that the ISR is
optional: `DCU/src/Configuration.h:9` sets `kCanIntPin = 48`, which is not an
interrupt pin on the Mega 2560, so `attachInterrupt` silently does nothing there and
the DCU has always run on the level-read path. Dropping the `/INT` **wire** is
therefore not possible without changing the shared library — the pin would float and
a HIGH reading would make the fast path return forever.

Because the ISR is optional, `/INT` is not actually pinned to D2/D3. The comment in
`AltimeterCAN/src/Configuration.h` is stricter than reality on this point. It is put
on D2 here anyway, which keeps the interrupt working.

Panel lighting is PWM (`CanMessageId::lights` 0x203 carries a 0..1000 dim value, see
`AltimeterCAN/src/CAN.cpp:75-85`). Of the ATmega328P's PWM pins, the Servo library
(Timer1) takes D9/D10, D11 is MOSI, and D3/D5 are encoder inputs. **D6 is the only
candidate** — the same bottleneck that put the altimeter's light on D5.

All three hall sensors move to the MCP23017. That keeps the internal pull-ups (A6/A7
have none and the PCB has no external ones), gives all three sensors an identical
path, and — because they land on one port — lets a single `readPort(B)` return all
three bits in one I2C transaction.

### Arduino Nano

| Pin | New use | Previously |
|---|---|---|
| D0/D1 | USB serial (DebugLog) | unchanged |
| **D2** | MCP2515 `/INT` (INT0) | OBS hall |
| D3/D4/D5 | OBS encoder A/B/push | unchanged |
| **D6** | panel light, PWM (Timer0/OC0A) | free |
| **D7** | `gs1Servo` | free |
| **D8** | `fromToServo` | HDG hall |
| D9 | `cdiServo` (was `vorServo`) | unchanged |
| **D10** | MCP2515 `/CS` (= `/SS`) | `vsi1Servo` |
| **D11/D12/D13** | MOSI / MISO / SCK | `fromToServo` / `vsi2Servo` / free |
| **A0** | `gs2Servo` | compass hall |
| A1/A2/A3 | HDG encoder A/B/push | unchanged |
| A4/A5 | I2C → MCP23017 | unchanged |
| A6/A7 | free (spare) | free |

### MCP23017

| Port | New use | Previously |
|---|---|---|
| GPA0..3 | OBS stepper coils | unchanged |
| GPA4..7 | HDG stepper coils | unchanged |
| GPB0..3 | compass stepper coils | unchanged |
| **GPB4** | OBS hall | — |
| **GPB5** | compass hall | — |
| **GPB6** | HDG hall | — |
| GPB7 | free | — |

### Seven wiring changes

Three servos (D10→D7, D11→D8, D12→A0), three halls (D2→GPB4, A0→GPB5, D8→GPB6), one
new light wire to D6. No components to retrofit.

### What the remap costs in code

1. `portMode(MCP23017Port::B, 0x70, 0x70, 0x70)` — GPB4..6 as inputs, pull-ups on,
   IPOL inverted so a set bit means "over the magnet". `writePort(B, compPattern & 0x0F)`
   stays valid: writes to input bits only land in OLAT.
2. **`Wire.setClock(400000)`.** Missing today — `HSIDriver/src/HSI.cpp:75` calls only
   `Wire.begin()`, so the bus runs at 100 kHz, where a one-byte register read costs
   roughly 400 µs against a 1.22 ms step interval at 12 rpm / 4096 steps. At 400 kHz
   it is about 100 µs.
3. Read port B **once per `loop()`** and cache it, not once per `fetchZeroedState()`
   call. One read serves all three axes.
4. `Servo::attach(A0)` — A0 is a full digital pin, no special case.
5. `analogWrite(6)` is safe: OC0A only sets COM0A1, it does not touch the Timer0
   prescaler, so `millis()`/`micros()` are unaffected.
6. RX filters: four exact IDs (`0x104`, `0x105`, `0x203`, `0x300`) across six slots.
   Put the two high-rate IDs on RXB0 (filters 0/1) and `0x203`/`0x300` plus duplicates
   on RXB1 (filters 2..5), so 50 Hz frames do not queue behind slow ones in the same
   buffer.

## Project layout

Copy of `HSIDriver`, then `lib/SiMessagePort` and `src/AirManager.*` removed and
`src/CAN.*` added.

```
HSICAN/
  platformio.ini
  include/
    HSIGeometry.h        Arduino-free, natively tested
    HSICalibration.h     Arduino-free, natively tested
    Backlash.h           Arduino-free, natively tested
  src/
    main.cpp  CAN.cpp/h  Configuration.h  DebugLog.h
    BenchDebug.cpp/h  HSI.cpp/h  CheapStepper.cpp/h
  test/
    test_geometry/  test_calibration/  test_backlash/
```

### `platformio.ini`

Follow the `[avr]` / `extends` shape of `AltimeterCAN/platformio.ini` rather than
repeating settings per env:

```ini
[avr]
framework = arduino
platform = atmelavr
monitor_speed = 115200
lib_deps =
    blemasle/MCP23017@^2.0.0
    arduino-libraries/Servo@^1.2.2
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

## `HSIGeometry.h` — Arduino-free logic

### Axis targets

`HSIDriver/src/HSI.cpp:273-277` drives the two pointer axes **relative to the compass
card**, because the course pointer and the heading bug ride on the rotating disc:

```cpp
cdi  = fmod(compassDegree - cdiDegree + 360, 360)   // the obs axis, in old naming
comp = compassDegree
hdg  = fmod(compassDegree - hdgDegree + 360, 360)
```

The old AirManager LUA (`etc/AirManagerCleartextLUA/HSI-MSG-H.lua`) pre-mirrored every
value before sending it: `compassDegree = -h`, `cdiDegree = (360-o)%360`,
`hdgDegree = (360-b)%360`. Substituting those, the mirroring **cancels out on the two
pointers**:

```
compass axis = (-h) mod 360        only the card is mirrored
obs     axis = (o - h) mod 360     a plain difference
hdg     axis = (b - h) mod 360     a plain difference
```

HSICAN receives the raw sim values over `0x104` and does this itself:

```cpp
// Inputs are sim values, outputs are motor angles for the three axes.
struct AxisTargets { uint16_t compassAxis100, obsAxis100, hdgAxis100; };
AxisTargets axisTargets(uint16_t heading100, uint16_t obs100, uint16_t hdgBug100);
```

The three `CheapStepper(inversed = true)` constructor flags are unrelated and stay —
they invert the coil sequence, not the target angle. Both effects are active on the
running rig and both must be reproduced.

### Homing window centre — the 180° trap

`HSIDriver/src/HSI.cpp:562-566` computes the hall-window midpoint as:

```cpp
uint32_t zeroStartPosition = axes[axis]->getPosition();
int32_t zeroAdjust = ((zeroEndPosition[axis] - zeroStartPosition) % totalSteps) / 2L;
```

Both operands are `uint32_t`, so the subtraction wraps. In the normal case
`zeroEnd - zeroStart` is the window width and half of it is the centre. If
`zeroEnd < zeroStart` — an overshoot larger than the window, or sensor noise at the
edge — the difference wraps to nearly `totalSteps` and `/2` lands **180° off, with
`isHomed == true`**.

This is the same defect the `WhiskeyCompassCAN` work hit twice; the fix that stuck
there was a plausibility bound. `AltimeterCAN` still carries the unguarded expression
and runs on the rig (see "Follow-ups"). The HSI has **three** axes, so the exposure is
tripled.

```cpp
// A hall window spans a few degrees. Anything beyond totalSteps/8 is a
// misread, not half a window.
bool homingWindowCentre(uint32_t zeroEnd, uint32_t zeroStart,
                        uint32_t totalSteps, int32_t* centreSteps);
```

`false` yields `homingTimeout` instead of a silent mis-home.

## `Backlash.h` — Arduino-free logic

The existing gear-backlash compensation is a **correct model** and is preserved
unchanged in behaviour. How it works today, in the vendored `CheapStepper`:

```cpp
// newMove, on a direction change:
unusedBacklashSteps = stepsBacklash - unusedBacklashSteps;
stepsLeft = numSteps + unusedBacklashSteps;

// incrementPosition / decrementPosition:
if (unusedBacklashSteps > 0) unusedBacklashSteps--;  // slack: motor turns, output does not
else                         position = ...;         // only then does position count
```

`position` is therefore the **output** position, not the motor position, which is what
makes `calculateShortestPath` and the homing captures correct. Partial reversals are
handled correctly too: having travelled 30 of 75 slack steps and reversing yields
`75 - 30 = 45`, not a full 75.

Configured values are 15 (compass), 28 (OBS) and 75 (HDG) steps out of 4096, i.e.
**1.3° / 2.5° / 6.6°** of play.

Four improvements, all agreed:

### a) Deadband against 50 Hz dither

`0x104` arrives at 50 Hz with deg × 100 resolution. At 11.38 steps per degree the
integer target jitters by ±1 step continuously, and every one of those direction
flips appends up to `stepsBacklash` extra motor steps — 75 on the HDG axis. A pointer
that should be standing still would instead be driven back and forth through the
slack: audible, wearing, and the axis never settles. The old chain had the same
weakness (`var_round(deg, 2)` in the LUA) but hid it behind onChange delivery.

Two thresholds per axis, a larger one for a reversal than for continuing:

```cpp
steps    = calculateShortestPath(axis, target);
reversal = (steps > 0) != lastDirectionIsClockwise();
limit    = reversal ? reversalDeadband[axis] : moveDeadband[axis];
if (abs(steps) < limit) return success;   // do nothing
```

Bench starting values: `moveDeadband = 2` steps (~0.18°),
`reversalDeadband = stepsBacklash / 3` (9 / 5 / 25 steps for obs / compass / hdg).
Both live in EEPROM. `reversalDeadband = 0` disables the behaviour.

### b) Timed slack take-up

`HSIDriver/src/HSI.cpp:378-386` steps in a tight loop with no delay, so the step rate
is set by I2C throughput rather than `kRpmLimits`. Over 75 steps that outruns the
motor, it slips, and the slack is not mechanically taken up at all — which is the
starting point of the entire homing sequence.

Becomes a `takeUpBacklash` phase ahead of `unknown` in the state machine, issued
through `newMove()` at min rpm and driven from `loop()`.

### c) `stepsBacklash` in EEPROM

15/28/75 are clearly measured values, and they change whenever the gear train is
re-greased or taken apart. They join the rest of the calibration below, with a
BenchDebug command that measures them against the hall sensor: creep one way until
the sensor switches, reverse, count steps until it switches again.

### d) Bookkeeping moved out of the vendored fork

The logic is pure integer arithmetic sitting between Arduino pins and coil patterns.
It moves to `include/Backlash.h`; `CheapStepper` calls into it.

```cpp
uint32_t backlashOnReversal(uint32_t unused, uint32_t total);
bool     consumeBacklashStep(uint32_t* unused);
```

This matches the repo pattern stated in the root `CLAUDE.md`: natively tested logic in
Arduino-free headers under `include/`, `src/` stays Arduino-coupled.

## `HSICalibration.h` and the EEPROM config

Same shape as `AltimeterCAN`: a struct with a magic word, `EEPROM.get`/`EEPROM.put`,
defaults written when the magic does not match.

All arrays are indexed by the enums, so `[hsiAxisCount]` is `{ obs, compass, hdg }` and
`[servoCount]` is `{ cdiServo, fromToServo, gs1Servo, gs2Servo }`.

```cpp
struct HsiConfig {
    uint16_t magic;
    //                                               obs  comp   hdg
    int16_t  zeroAdjustDegree[hsiAxisCount];      // -90,   85,   11
    uint16_t stepsBacklash[hsiAxisCount];         //  28,   15,   75
    uint16_t moveDeadband[hsiAxisCount];          //   2,    2,    2
    uint16_t reversalDeadband[hsiAxisCount];      //   9,    5,   25
    //                                               cdi  frTo   gs1    gs2
    int16_t  servoAdjustDegree100[servoCount];    // 8000,    0, 7100,  8900
    int16_t  servoAdjustRatio1000[servoCount];    // 2000, 1000, 2000, -1429
    int16_t  cdiDegPerDot100;                     // 1000 = 10.0 deg/dot
    int16_t  gsDegPerDot100;                      // 1000
};
```

Watch the backlash ordering: `HSIDriver/src/HSI.cpp:87-89` constructs the steppers in
the order compass/cdi/hdg with 15/28/75, but the enum order is cdi (now obs) first, so
the array reads **28, 15, 75**.

Compiled-in defaults reproduce the values the AirManager firmware carried, so a
freshly flashed board behaves like the old one.

Helper functions carried over from `AltimeterCalibration.h`: `normalizeZeroAdjust`,
`accumulateZeroAdjust`, `jogDegreesFromPosition`. Plus the servo adjust computation
and the dots → degree conversion, both now reading from EEPROM.

## `HSI` — device logic

### Homing becomes non-blocking

`HSIDriver::homeAllAxis()` is a blocking `while (!checkAllHomed())` loop even though
the state machine behind it is already non-blocking. A three-axis sweep takes several
seconds, during which 50 Hz frames would fill both RX buffers and the gateway
heartbeat would expire. Replaced by `beginHoming()` / `isHoming()` driven from
`loop()`, as in `AltimeterCAN/src/CAN.cpp:105`.

This also closes follow-up ticket 1 from the `WhiskeyCompassCAN` work: `homingState`
gets an accessor and a retry. Without it a failed homing is a dead instrument until
power-cycle, because `onGatewayHeartbeatDiscovered()` only fires on the false→true
edge.

### Park state

`onGatewayHeartbeatTimeout()` reproduces what `bus_volts < 6` did in the LUA: compass
card to 0, `cdiServo` and both GS servos to 0, `fromToServo` to `noNav`. The OBS and
HDG pointers are left where they are — the LUA did not blank those either.

### Flags

```
bit0 TO    -> fromToServo = to    (90)
bit1 FROM  -> fromToServo = from  (182)
neither    -> fromToServo = noNav (0)
bit2 == 0  -> gs1Servo/gs2Servo to mechanical zero
```

This replaces the broken float comparison in `HSIDriver/src/AirManager.cpp:98`, which
tests `fromTo == 2.4` while the LUA sends `nav1_fromto * 1.0`, i.e. `2.0`. It never
matches, falls into the error branch, and resolves to `noNav` — **the FROM flag has
never worked on the rig**.

### Encoders and buttons

Both encoder ISRs are unchanged. The send path replaces the absolute counter with a
delta edge: remember the last counter value, send the difference on change, reset.

| Gesture | Effect |
|---|---|
| OBS knob turn | `0x341` {control 0, action 0, delta} |
| OBS knob push | read and debounced, `pushState` stays 0 — path reserved |
| HDG knob turn | `0x341` {control 1, action 0, delta} |
| HDG knob short press | `0x341` {control 1, action 1, pushState 1} → plugin syncs bug to heading |
| HDG knob long press (> 1.5 s) | local `beginHoming()`, nothing on CAN |

The long-press measurement is repaired on the way. `HSIDriver/src/HSI.cpp:197` tests
`uS - lastCompDebounceTime > longPressDelay`, but `lastCompDebounceTime` is updated on
every debounced transition, so it measures time since the last edge rather than press
duration — after 1.5 s of idle the **first short press** homes. The replacement
records the press start and measures duration on release.

## `CAN` — transport layer

Subclass of `InstrumentCAN`, following `AltimeterCAN/src/CAN.h`.

```cpp
onGatewayHeartbeatDiscovered()  -> light on, beginHoming() if not homed
onGatewayHeartbeatTimeout()     -> light off, park state
onStartupFail()                 -> light off
homing failed                   -> blink the light (WhiskeyCompassCAN pattern), retry
```

Receive: `0x104`, `0x105`, `0x203`, `0x300`, filters as described under the pin remap.
Transmit: `0x341` onChange only, plus the `InstrumentCAN` heartbeat.

Keep `String` out of `handleFrame`, `loop()` and the homing path — see "RAM" below.

## `BenchDebug` — bench console

`hd<deg>` card · `ob<deg>` OBS · `bg<deg>` HDG bug · `cd<dots>` CDI · `gs<dots>` GS ·
`fl<bits>` flags · `hm` re-home · `li<0..255>` light · `cz<axis>` calibrate zero ·
`bl<axis>` measure backlash · `cw` wipe calibration · `st` stop axes.

## DCU side

### `DCUReceiver`

New cases `SerialMessageHsiHeading` → `sendHsiHeading()` (CAN `0x104`) and
`SerialMessageHsiNav` → `sendHsiNav()` (CAN `0x105`). Both join the 5 s
`checkMaxAgeResync` cache — gateway→instrument traffic is idempotent, exactly like
`sendCompass()`.

### `CAN::handleFrame`

New case `hsiInput` → `DCUSender::sendFrame(SerialMessageHsiInput, ...)`. No cache, no
resend. The existing range filter already admits `0x341`.

### `BenchDebug`

`hd<deg>`, `hn<dots>`, `hf<flags>` to drive the instrument, and an `hw` watch for
`hsiInput` following the `bw`/`rw` pattern described in `DCU/CLAUDE.md`.

## Plugin side

New datarefs, read: `heading_electric_deg_mag_pilot`, `hsi_obs_deg_mag_pilot`,
`heading_dial_deg_mag_pilot`, `nav1_fromto`, `nav1_hdef_dot`, `nav1_vdef_dot`,
`nav1_CDI`.

New datarefs, written: `hsi_obs_deg_mag_pilot`, `heading_dial_deg_mag_pilot`.

Transmit follows the `compassAccumulator_` pattern at
`DCUProviderPlugin/src/DCUProvider.cpp:413`, with `HSI_HEADING_RATE` 50 and
`HSI_NAV_RATE` 10.

Receive accumulation for `SerialMessageHsiInput`:

```
control 0, action 0                -> obs    += delta * kObsDegPerClick, wrap 0..360
control 1, action 0                -> hdgBug += delta * kHdgDegPerClick, wrap 0..360
control 1, action 1, pushState 1   -> hdgBug  = headingElectric
control 0, action 1                -> no-op (path reserved)
```

`kObsDegPerClick = kHdgDegPerClick = 1.0`. The old chain fed its encoder counter
straight into the dataref as degrees, so one click is one degree.

**Cleanup:** `dr_HeadingBug` currently points at
`sim/cockpit/autopilot/heading_bug_deg_mag_pil`
(`DCUProviderPlugin/src/DataRefManager.cpp:23`) — a different dataref from the
`cockpit2/heading_dial_deg_mag_pilot` used by the sheet and the LUA — and nothing
calls it. It moves to `cockpit2` and becomes the heading-bug write path.

## Testing

`pio test -e native` in `HSICAN`:

- `test_geometry` — `axisTargets()` across the wrap cases (h=0, o=359, b=1, all three
  equal, the 359→0 transition), and `homingWindowCentre()` for the two edge cases that
  bit the whiskey compass in turn (`zeroEnd == 0`, `zeroStart == 0`), an implausible
  window beyond `totalSteps/8`, and `totalSteps == 0`.
- `test_calibration` — `normalizeZeroAdjust`, `accumulateZeroAdjust`,
  `jogDegreesFromPosition`, the servo adjust computation, dots → degree, and the
  default fallback on an invalid magic word.
- `test_backlash` — full reversal from rest yields `total`; partial reversal of 30 of
  75 yields 45; reversing twice returns 30; same direction leaves the counter
  untouched; `stepsBacklash == 0` always yields 0.

`pio test -e native` in `DCU`: add flags-byte composition (`nav1_fromto` + `nav1_CDI`
→ bits 0..2) to the existing suite.

Builds: `HSICAN` `env:nano` with `BENCHDEBUG` 0 **and** 1; `DCU` mega and native;
`DCUProviderPlugin/build-macos.sh`; and every other CAN board rebuilt against the
changed shared headers, not only the ones obviously touched.

### RAM is the main risk

HSICAN will be the heaviest board in the repo: three `CheapStepper`, four `Servo`,
MCP23017, MCP2515, EEPROM config. `HSIDriver/src/HSI.cpp` is written throughout with
`String(F(...))` — precisely what `AltimeterCAN`'s `handleFrame` forbids itself
("No String here: heap churn + extra stack in the deepest call path",
`AltimeterCAN/src/CAN.cpp:43`).

PlatformIO's RAM percentage does not account for the heap; an RPMGaugeCAN CAN freeze
turned out to be a stack/heap collision behind a harmless-looking figure. Strip
`String` from `handleFrame`, `loop()` and the homing path, and measure free RAM at
runtime rather than trusting the build output.

## Build order

1. **Shared headers** — `CanNodeId.h`, `CanMessageId.h`, `SerialMessageId.h`, then
   rebuild every CAN board.
2. **DCU** — `DCUReceiver`, `CAN::handleFrame`, `BenchDebug`; mega and native.
3. **Plugin** — datarefs, TX accumulators, RX accumulation; `./build-macos.sh`.
4. **HSICAN scaffold** — project, `platformio.ini`, the three Arduino-free headers,
   native tests. Green before any firmware lands.
5. **HSICAN firmware** — `CAN.*`, HSI rework, EEPROM, BenchDebug.
6. **Bench, then rig.**

## Rollback

After the seven wiring changes, **`HSIDriver` is no longer flashable as a fallback** —
its `Configuration.h` expects servos on D10..D12 and halls on D2/D8/A0. The same is
already true of `AltimeterDriver` against `AltimeterCAN`, per the root `CLAUDE.md`.

Mitigation, at no extra cost: flash HSICAN with `BENCHDEBUG = 1` onto the rewired
board first and verify mechanics, homing and servo travel over the console before the
MCP2515 is connected to the bus at all. Document the old wiring before touching it.

## Bench checklist

- Home ten times and watch specifically for a run that lands **~180° off**, not merely
  for scatter.
- Card rotation direction.
- Pointer differences at h=0 / o=90 / b=270.
- Zero points survive a power cycle.
- Backlash: approach a pointer target from both directions and compare final position.
- Deadband: hold a steady heading and confirm the pointers are silent, then confirm a
  real 1° change still moves them.
- TO/FROM flag in all three states, especially FROM, which has never been seen on the
  rig.
- GS needles when `nav1_CDI == 0`.
- Light dimming across the 0..1000 range.
- Encoder deltas in both directions, 100 clicks each way, no drift.
- HDG short press syncs the bug; long press homes; short press does **not** home.

## Follow-ups, deliberately not in this project

- **The same 180° homing bug in `AltimeterCAN`.** The unguarded expression runs on the
  rig there. Fix it when that board is next touched, using `homingWindowCentre()`.
- **Idle coil shutdown.** All three steppers stay energised at rest, roughly
  600–900 mA for an instrument that is almost always still. The `off()` call in
  `CheapStepper::run()`'s `stepsLeft == 0` branch is commented out, inherited from the
  upstream fork, and the only escape is the `st` bench command. `CheapStepper` is
  vendored separately into every board project, so an idle timeout belongs in all of
  them or none — its own round of work, together with follow-up ticket 2 from the
  `WhiskeyCompassCAN` notes.

## Out of scope

- Retiring `HSIDriver` from the tree.
- Any change to the motion chain (`MotionMessageId.h` / `MotionNodeId.h`), which is a
  separate namespace on a separate physical bus.
