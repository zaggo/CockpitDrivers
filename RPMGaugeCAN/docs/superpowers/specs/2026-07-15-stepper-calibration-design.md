# RPM Gauge Stepper Calibration Design

## Problem

The stepper needle (`vid6608`, 12 native motor steps per degree, absolute-position
API) is mapped to RPM values purely via the hardcoded `kMinimumDegree`/`kMaximumDegree`
constants in `Configuration.h`. Two issues follow from this:

- After `motor->zero()` homing, the needle's mechanical zero position does not line
  up with the "0" mark on the physical scale — the needle rests below the scale's
  zero.
- Sending `rp3500` in BenchDebug mode does not point the needle at the scale's max
  mark, because `kMaximumDegree` (320°) doesn't match the motor's actual absolute
  position for that mark on the installed dial.

Both are calibration problems: the motor's absolute coordinate system and the
physical dial's printed scale are not aligned, and there's currently no way to
teach the firmware the alignment.

## Precedent

`HandbrakeCAN/src/Handbrake.cpp` already solves the equivalent problem for its
potentiometer: a `Config` struct (magic + version + payload) persisted via
`EEPROM.get`/`EEPROM.put`, loaded in the constructor with fallback to defaults on
invalid/missing data, and `calibrateMin()`/`calibrateMax()` methods wired to `mi`/`ma`
BenchDebug commands. This design reuses that pattern for `RPMGauge` directly —
same command names, same struct shape, same load/save flow — for consistency
across boards.

## Data model

New `Config` struct owned by `RPMGauge` (declared in `RPMGauge.h`, EEPROM I/O in
`RPMGauge.cpp`):

```cpp
struct Config {
    uint32_t magic;
    uint16_t version;
    uint16_t minStep;  // absolute motor step position representing RPM = 0
    uint16_t maxStep;  // absolute motor step position representing RPM = kMaxRPM
};
```

- EEPROM address `0` (RPMGaugeCAN uses EEPROM nowhere else).
- Magic constant distinct from HandbrakeCAN's (e.g. `'R','G','C','1'`).
- Version `1`.
- Stored in absolute motor steps (native `vid6608` unit, same unit `getPosition()`/
  `moveTo()` use) — avoids the rounding a degree-based representation would
  introduce both when capturing the current position (`getPosition()` is already
  in steps) and when computing the RPM→position mapping.

## `RPMGauge` changes

- `loadConfig()`: called from the constructor, after `motor->zero()`. Reads the
  `Config` via `EEPROM.get`. If `magic`/`version` don't match, resets to defaults
  (`minStep = kMinimumDegree * 12`, `maxStep = kMaximumDegree * 12` — preserves
  today's behavior when no calibration has been saved yet) and writes them back,
  exactly like `Handbrake::loadConfig()`.
- `saveConfig()`: `EEPROM.put` of the current `Config`.
- `calibrateMin()` / `calibrateMax()`: read `motor->getPosition()` into
  `config.minStep` / `config.maxStep` respectively, then `saveConfig()`.
- `moveNeedle(uint16_t rpm, bool calibration = false)`:
  - `calibration == true` (the existing `cl` command path): **unchanged**,
    stays unclamped and independent of `minStep`/`maxStep` — this is required so a
    new physical endpoint can be driven to and captured via `mi`/`ma` even when it
    lies outside the currently-saved logical range.
  - `calibration == false` (normal RPM display path): replaces the old
    degree-based clamp/scale with
    `step = config.minStep + ratio * (config.maxStep - config.minStep)`
    (where `ratio = constrain(rpm / kMaxRPM, 0, 1)`), then `motor->moveTo(step)`
    directly — no intermediate degree value or `* 12` conversion.

`kMinimumDegree`/`kMaximumDegree` remain in `Configuration.h`, now used only as
the EEPROM-invalid fallback and by the existing `cl` degree-based calibration
path (unchanged there).

## BenchDebug changes

Two new commands, matching `HandbrakeCAN`'s naming and feedback style:

- `mi`: calibrate logical minimum to the stepper's current absolute position.
- `ma`: calibrate logical maximum to the stepper's current absolute position.

Both print a confirmation line (à la Handbrake's `"Min position calibrated."`).
Both are added to the `continuousTestActive` blocklist in
`handleRPMGaugeInput` alongside `rp`/`cl`/`br`/`od`/`oh`, since capturing a
position while the continuous test is driving the motor makes no sense. The `?`
help text gains two lines describing `mi`/`ma`.

## Startup (both modes)

No changes needed outside `RPMGauge` itself: `main.cpp` constructs `RPMGauge`
identically in both `BENCHDEBUG` and normal (CAN) modes, and the constructor now
calls `loadConfig()` unconditionally. `CAN.cpp` requires no changes.

## Testing

`RPMGaugeCAN/test/` is currently an empty PlatformIO scaffold. Given EEPROM
access requires either hardware or a native mock, automated coverage is
best-effort:

- If a native EEPROM mock is feasible in the `native` test env, add unit tests
  for `loadConfig` (valid config round-trips; invalid magic/version falls back
  to defaults) and for the `moveNeedle` RPM→step mapping math (pure function,
  no hardware needed if extracted/testable in isolation).
- Otherwise, verify manually via BenchDebug: `mi`/`ma` at two physical
  positions, reset the board, confirm `rp0` and `rp3500` (or `rp<kMaxRPM>`)
  land on those exact positions, and confirm `cl<degree>` still moves
  unclamped outside that range.
