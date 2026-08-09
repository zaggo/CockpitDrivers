# Rudder axis smoothing (RudderCAN)

Date: 2026-08-09
Status: approved, ready for implementation planning

## Problem

During end-to-end testing with X-Plane the rudder value jitters while the pedals are being
moved. At rest the value is stable.

The current signal chain in `RudderCAN` runs once every 20 ms (`CAN::loop()`):

```
1x analogRead()  ->  mapRudder()  ->  change threshold +-8  ->  CAN frame
```

There is no filtering at all — a single 10-bit ADC sample is mapped and sent. Two mechanisms
produce the visible steps:

1. **ADC quantisation.** One LSB maps to `1000 / (0.95 * |halfSpan|)` wire units. With a
   half-span of 490 counts that is ~2 units; with 150 counts it is ~7 units.
2. **The change threshold itself** (`kRudderChangeThreshold = 8`, `Rudder.cpp:17`). The reported
   value stays flat and then jumps by 8 or more at once. Sensor and foot-tremor noise makes the
   mapped value re-latch back and forth across that threshold, which is the jitter that was
   observed.

The threshold is a pure quantiser, not a filter, and is likely the larger contributor.

## Goal

Smooth the reported rudder (and toe brake) values without introducing perceptible lag on real
pedal movement.

Non-goals: changing the CAN wire format, changing the calibration procedure, changing the
DCU or `DCUProviderPlugin` side.

## Approach

Oversample at a fixed rate well above the send rate, run a speed-adaptive exponential moving
average over the raw ADC value, carry the extra fractional resolution all the way into the
mapping, and only then lower the change threshold.

Rejected alternatives:

- **Oversampling only** (no low-pass): guarantees zero lag, but leaves residual jitter whenever
  real sensor noise exceeds the quantisation step.
- **Full One Euro filter** (low-pass on the derivative driving the cutoff of the signal
  low-pass): best behaviour, but roughly 3x the code of the adaptive EMA for little practical
  gain on a single 50 Hz axis.

## Design

### 1. Sampling decoupled from reporting

`getState()` currently performs the `analogRead`. Sampling moves into its own method so it can
run far more often than the 20 ms send cadence:

```
loop(), every pass    ->  rudder->sample()          gated to every 2 ms, 3x analogRead -> filters
loop(), every 20 ms   ->  rudder->getStateUpdate()  reads filter state only, no ADC
```

`sample()` never blocks: three ADC conversions at ~112 us each every 2 ms is about 17 % CPU on
a 16 MHz AVR, leaving ample headroom for the MCP2515 SPI traffic.

Callers gain one line at the top of their `loop()`:

- `CAN::loop()` (`RudderCAN/src/CAN.cpp:16`)
- `BenchDebug::loop()` (`RudderCAN/src/BenchDebug.cpp:158`)

If `getState()` is called before any sample has been taken, it seeds the filters first so it can
never return zeros.

`getRawRudder()` / `getRawLeftBrake()` / `getRawRightBrake()` / `sampleAverage()` and every
`calibrateXxx()` keep reading the ADC directly and unfiltered — calibration points must not be
taken through the filter.

### 2. `AdaptiveFilter` (header-only, no Arduino dependency)

New file `RudderCAN/include/AdaptiveFilter.h`, following the pattern of the header-only helpers
in `DCU/include/` so it can be unit tested natively.

State is the filtered raw value in Q6 fixed point (`raw << 6`). Maximum is `1023 << 6 = 65472`,
so `int32_t` is used throughout.

```
update(raw):
    d_q6  = (raw << 6) - _q6
    d_lsb = |d_q6| >> 6
    alpha = min(a0 + k * d_lsb, 256)        // Q8
    _q6  += (d_q6 * alpha) >> 8
```

`a0` and `k` are constructor arguments so tests can vary them; production values live next to the
existing tuning constants in `Rudder.cpp`.

| Parameter | Value | Effect |
| --- | --- | --- |
| Sample interval | 2 ms (500 Hz) | 10 samples per CAN frame |
| `a0` (alpha min, Q8) | 32 | tau ~16 ms at rest, ~4x noise reduction |
| `k` (slope) | 12 | fully transparent from ~19 LSB deviation |

The steady-state tracking error solves `k*d^2 + a0*d = 256*v`, with `v` in LSB per sample:

| Situation | `v` | `d` | `alpha` | Lag |
| --- | --- | --- | --- | --- |
| Pedals at rest | 0 | noise only | ~32 | — (smoothing) |
| Slow creep (100 LSB/s) | 0.2 | 1.1 | 45 | 11 ms |
| Full sweep in 0.5 s | 4 | 8 | 128 | 4 ms |
| Kick in 0.15 s | 13 | — | ~188 | near instant |

Every lag figure is below the 20 ms frame interval, so it cannot be perceived at the 50 Hz
send rate.

Noise attenuation at rest is `sqrt(a/(2-a)) = sqrt(0.125/1.875) = 0.26`, i.e. roughly 4x.

### 3. Carry the resolution into the mapping

Filtering is wasted if the result is truncated back to whole ADC counts before mapping.

`mapHalf()`, `mapUnipolar()` and `Rudder::mapRudder()` take the Q6 value instead of a
`uint16_t` raw count. Calibration points and `kRudderCenterDeadbandRaw` are scaled by `<< 6` at
the point of use. All arithmetic stays in `long`; the worst case `65472 * 1000 = 6.5e7` is well
inside `INT32_MAX`.

Output granularity is then set by the filter (effectively 12–13 bits) rather than by the 10-bit
ADC, so the wire quantum approaches 1 unit even with a small sensor span.

Only because of this does lowering the threshold make sense:

- `kRudderChangeThreshold`: 8 -> 2
- `kBrakeChangeThreshold`: 8 -> 2

The send rate stays capped at 50 Hz by `kMinSendIntervalMs = 20` (`CAN.h:31`), so bus load is
unchanged. The exact-endpoint reporting rule (`Rudder.cpp:205-211`) is kept as is.

`kRudderCenterDeadbandRaw = 12` stays unchanged — it covers mechanical slop in the pedal
linkage, not electrical noise.

### 4. Build and tests

`RudderCAN/platformio.ini` gains a `native` env mirroring `DCU/platformio.ini:20`. The existing
`[env]` section propagates `platform = atmelavr` and `framework = arduino` to every environment,
so the AVR-common settings move into an `[avr]` section and `env:nano` /
`env:diecimilaatmega328` pick them up via `extends = avr`.

Unity tests for `AdaptiveFilter.h`, run natively with `pio test -e native`:

- `reset()` seeds exactly, with no ramp from zero
- Step response converges to a constant input without overshoot
- Feeding a +-2 LSB square wave leaves an output span below 1 LSB
- On a ramp of 4 LSB per sample the tracking error stays below 10 LSB
- `alpha` saturates at 256 and a 0 <-> 1023 jump does not overflow

`BenchDebug::printState()` additionally prints the filtered Q6 value so the measurement step
below is observable.

## Measurement step (before implementation)

Set `BENCHDEBUG 1` in `RudderCAN/src/Configuration.h:7`, flash, and read `s` at both rudder
endstops to record the actual raw span of the hall sensor.

This does not change the design; it only calibrates expectations. A near-full span (+-450
counts) means section 3 alone may already fix the complaint, while a small span means the
filter carries most of the improvement.

## Hardware note

A 100 nF capacitor from A0 to GND at the Arduino end suppresses pickup on the pedal wiring. It
costs nothing and is independent of everything above.

## Risks

- The 2 ms sample gate uses `millis()`, whose 1 ms resolution makes the effective interval
  2–3 ms. The filter constants are not sensitive at that level.
- Adding ~17 % ADC load to `loop()` must not starve the MCP2515 RX path. RX is interrupt-driven
  with buffering, so this is expected to be safe, but it should be confirmed on the bench.
- RAM cost is 3 filters x (`int32_t` + flag) = well under 20 bytes. Noted because AVR RAM
  headroom has bitten this repo before.
