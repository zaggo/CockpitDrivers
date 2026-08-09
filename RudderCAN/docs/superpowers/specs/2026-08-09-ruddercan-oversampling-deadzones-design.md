# RudderCAN: Oversampled Analog Inputs + Percent-Based Deadzones

Date: 2026-08-09
Status: approved, ready for implementation plan

## Goal

Port two ideas from the standalone `Hall49` sketch (`/Users/zaggo/Developer/Hall49/src/main.cpp`)
into `RudderCAN`:

1. **Oversampling** — trade sample rate for effective ADC resolution, so the rudder and brake axes
   resolve far finer than the raw 10 bits of the ATmega328 ADC.
2. **Deadzones as tunable percentages** — snap-to-anchor behaviour around the calibrated endpoints
   and the rudder center, with the travel between anchors rescaled so there is no jump.

`RudderCAN` already implements the second idea structurally (`mapHalf` / `mapUnipolar` in
`src/Rudder.cpp` snap and rescale), but hardcodes the zones as a raw ADC count (center) and a
fixed `span / 20` (ends). The port makes both configurable and consistent.

## Constraints

- Target: ATmega328 (`nanoatmega328new`, `diecimilaatmega328`), 16 MHz, 2 KB SRAM.
- `analogRead()` costs ~112 µs. Three axes.
- The main loop must keep polling CAN RX and emitting the 500 ms instrument heartbeat, so a
  blocking 256-sample burst per axis (~86 ms per `getState()`) is not acceptable at runtime.
- No `float` math on the hot path — integer only.
- No dynamic allocation beyond what already exists; SRAM headroom matters (past CAN freezes on
  other AVR nodes were stack/heap collisions).

## Decisions

| Question | Decision |
|---|---|
| Oversampling structure | Non-blocking fixed-point EMA, one `analogRead` per axis per `loop()` |
| Resolution | EMA shift N = 6 (~64-sample window) → **3 extra bits, 13-bit effective**, τ ≈ 32 ms |
| Deadzone model | Keep existing snap+rescale, integer math; expose zones as percent constants |
| EEPROM | Migrate v1 → v2 by scaling stored calibration points ×8 |
| Testing | Extract pure mapping/EMA math into a header, cover it with native Unity tests |

### Why EMA rather than block decimation

Gaining `E` extra bits requires averaging over `4^E` samples regardless of implementation, so the
~32 ms window at N = 6 is inherent, not an artifact. The difference is the update characteristic:
block decimation emits a new value only once per window (visible steps), while the EMA updates on
every sample and follows the pedal continuously. For an axis driving a flight sim, continuous
tracking wins. N = 6 was chosen over N = 8 (full 14 bits, τ ≈ 128 ms) because pedal responsiveness
matters more than the 14th bit — the wire scale is only 0..1000, which is coarser than 13 bits
anyway.

## Design

### New unit: `src/OversampledInput.h` / `.cpp`

One analog axis, self-contained. Depends only on `Arduino.h` (for `analogRead`/`pinMode`) and
`RudderMapping.h` (for the EMA step).

```cpp
class OversampledInput {
public:
    explicit OversampledInput(uint8_t pin);

    void     begin();          // pinMode + prime the accumulator from one raw read
    void     sample();         // exactly one analogRead; call once per loop() iteration
    uint16_t value() const;    // last smoothed value, 0..kAdcMax
    uint16_t sampleBlocking(); // full 256-sample burst, resets the accumulator; calibration only

private:
    uint8_t _pin;
    int32_t _acc;              // EMA accumulator, value scaled by 2^kAdcEmaShift
    uint16_t _value;
};
```

RAM: 7 bytes per instance, 21 bytes for three axes.

`_acc` is `int32_t` (not unsigned) so the `raw - (_acc >> N)` term is plainly signed and needs no
wraparound reasoning.

### Fixed-point EMA

```
acc += raw - (acc >> kAdcEmaShift);
value = acc >> (kAdcEmaShift - kAdcExtraBits);
```

With `kAdcEmaShift = 6` and `kAdcExtraBits = 3`:

- `acc` converges to `raw << 6`; maximum `1023 << 6 = 65472`.
- `value = acc >> 3`, range `0 .. 8184` (`1023 << 3`).
- Time constant ≈ 64 samples; at ~0.5 ms per loop iteration that is ~32 ms.

`begin()` primes `_acc = analogRead(pin) << kAdcEmaShift` so the axis does not ramp up from zero
after reset.

`sampleBlocking()` sums `kAdcCalibrationSamples = 256` raw reads and returns
`sum >> (kAdcCalibrationShift - kAdcExtraBits)` = `sum >> 5`, landing on exactly the same
`0 .. 8184` scale as the EMA path. It then re-primes `_acc` from that result. Cost ~29 ms, only
ever called from the calibration commands, and it replaces the current
`sampleAverage(pin, 16)` with its `delay(5)` per sample (~80 ms) — so calibration actually gets
faster while sampling 16× more.

### `Configuration.h` additions

```cpp
// ADC oversampling. A fixed-point EMA over ~2^kAdcEmaShift samples buys
// kAdcExtraBits of resolution; the window is inherent to the averaging, not
// a choice of implementation.
const uint8_t  kAdcEmaShift           = 6;   // ~64-sample time constant, ~32ms
const uint8_t  kAdcExtraBits          = 3;   // 10-bit ADC -> 13-bit effective
const uint8_t  kAdcCalibrationShift   = 8;   // 256-sample blocking burst
const uint16_t kAdcCalibrationSamples = 1U << kAdcCalibrationShift;
const uint16_t kAdcMax                = (uint16_t)(1023U << kAdcExtraBits); // 8184

// Deadzones as a percentage of the calibrated travel of each half/axis.
const uint8_t kRudderCenterDeadzonePercent = 2;
const uint8_t kRudderEndDeadzonePercent    = 5;
const uint8_t kBrakeDeadzonePercent        = 5;
```

`kBrakeDeadzonePercent` defaults to 5 to preserve today's `span / 20` behaviour; Hall49 used 2 and
the constant now makes that a one-line change.

### `src/RudderMapping.h` (new, header-only, Arduino-free)

Holds the pure math so it can be unit-tested natively. No `Arduino.h`; provides its own
`mapLinear()` and `clampLong()` instead of the Arduino `map()`/`constrain()` macros.

```cpp
long     clampLong(long v, long lo, long hi);
long     mapLinear(long x, long inMin, long inMax, long outMin, long outMax);
int32_t  emaStep(int32_t acc, uint16_t raw, uint8_t shift);   // returns new accumulator
uint16_t emaValue(int32_t acc, uint8_t shift, uint8_t extraBits);
uint16_t percentOf(long span, uint8_t percent);               // |span| * percent / 100
uint16_t mapHalf(long raw, long center, long end, uint8_t centerPercent, uint8_t endPercent);
uint16_t mapUnipolar(long raw, long lo, long hi, uint8_t deadzonePercent);
int16_t  mapRudderAxis(long raw, long min, long center, long max,
                       uint8_t centerPercent, uint8_t endPercent);
```

Behaviour is unchanged apart from the parameterisation:

- `mapHalf` derives both zones from the signed `span = end - center`, so inverted potentiometer
  wiring keeps working without special casing.
- Values inside the center zone map below 0 and are clamped to 0 — that clamp *is* the snap. The
  explicit early-return currently in `Rudder::mapRudder` therefore disappears; the half is selected
  by the sign test alone.
- `mapUnipolar` applies `deadzonePercent` at both ends.

### `src/Rudder.cpp` / `.h` changes

- Three `OversampledInput` members replace the direct `analogRead` calls.
- New `void poll();` — calls `sample()` on all three inputs. `main.cpp` calls `rudder->poll()` once
  per `loop()`, before dispatching to `benchDebug->loop()` / `canBus->loop()`, so both modes are fed
  identically.
- `getRawRudder()` / `getRawLeftBrake()` / `getRawRightBrake()` return the smoothed 13-bit value
  (BenchDebug's raw display keeps working, numbers just get bigger).
- `sampleAverage()` is deleted; the seven `calibrate*()` methods use `sampleBlocking()`.
- `mapHalf` / `mapUnipolar` / `mapRudder` become thin forwarders into `RudderMapping.h`.
- `kRudderCenterDeadbandRaw` is deleted (superseded by the percent constants).
- `kRudderChangeThreshold` / `kBrakeChangeThreshold`: **8 → 4**. The old comment ("one ADC LSB is
  roughly 2 units of the 0..1000 scale") no longer holds — at 13 bits one LSB is ~0.25 units, so 8
  throws away resolution the oversampling just bought. The endpoint-exactness rule in
  `getStateUpdate()` is unchanged.

### EEPROM migration (v1 → v2)

`kRudderConfigVersion` goes to 2. `loadConfig()`:

- `magic` mismatch → defaults, as today, but on the new scale:
  `rudderMin = 0`, `rudderCenter = kAdcMax / 2` (4092), `rudderMax = kAdcMax` (8184),
  brakes `0 .. kAdcMax`.
- `magic` matches and `version == 1` → shift all seven calibration points left by `kAdcExtraBits`
  (×8), set `version = 2`, `EEPROM.put`, log the migration. Existing bench calibration survives; no
  re-calibration needed.
- `magic` matches and `version == 2` → load as-is.
- Anything else → defaults.

Note the struct layout is unchanged (all `uint16_t`), so v1 records deserialize correctly before
being rescaled.

### Testing

`test/test_mapping/test_mapping.cpp`, Unity, run natively via a new `[env:native]` in
`platformio.ini` (`platform = native`, no Arduino framework). Coverage:

- `emaStep` / `emaValue`: converges to `raw << extraBits`; a primed accumulator holding a constant
  input stays put; monotone approach from below and above; `acc` never exceeds `1023 << shift`.
- `mapHalf`: 0 inside the center zone; 1000 at and past the end zone; monotone in between;
  identical results for inverted wiring (`end < center`); `span == 0` returns 0.
- `mapUnipolar`: 0 at/below `lo + deadzone`, 1000 at/above `hi - deadzone`, monotone between,
  inverted wiring, degenerate span.
- `mapRudderAxis`: correct sign per half, exact 0 at center, ±1000 at the endpoints, asymmetric
  calibration (min/max at different distances from center) handled per-half.

## Out of scope

- CAN wire format and `RudderState` scaling (0..1000) are unchanged — no gateway or
  `DCUProviderPlugin` change is needed.
- The BenchDebug command set is unchanged.
- No changes to any other board project.

## Verification

- `pio run -e nano` and `pio run -e diecimilaatmega328` build clean; check the reported SRAM figure
  against the previous build (expect ~+25 B static).
- `pio test -e native` passes.
- On hardware: existing EEPROM calibration is picked up and migrated (log line), rudder reads 0 at
  rest and reaches ±1000 at the stops, brakes reach 0 and 1000, and CAN traffic stays event-driven
  rather than saturating at the new finer threshold.
