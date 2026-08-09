# DCU BenchDebug — Rudder Watch

Date: 2026-08-09

## Problem

In `BENCHDEBUG` mode `main.cpp` builds `BenchDebug` instead of `DCUReceiver`, so no `DCUSender` is
ever attached to `CAN`. `CAN::updateRudder()` decodes incoming `rudder` (0x303) frames and then drops
them, because its only sink is the null `dcuSender`. Bench-testing the RudderCAN board against the DCU
therefore gives no feedback at all on the DCU side.

## Goal

A console mode in `BenchDebug` that prints rudder and toe-brake values as they arrive over CAN, and
stops on any keypress.

## Design

### Transport: sample slot polled by BenchDebug

`CAN` keeps the last decoded frame in a one-entry slot instead of calling back into the console:

```cpp
#if BENCHDEBUG
    bool takeRudderSample(RudderToDcuMessage& sample);   // public
    RudderToDcuMessage rudderSample = {0, 0, 0};         // private
    bool rudderSampleValid = false;
#endif
```

`updateRudder()` fills the slot right after `unpackBE16` decoding, before the `dcuSender` null check.
`takeRudderSample()` returns `true` and clears `rudderSampleValid` when a frame arrived since the last
call. `updateRudder()` runs from `CAN::loop()` (the RX path is polled, not ISR-driven), so the slot
needs no interrupt guarding.

Rejected alternatives: an observer interface mirroring `setDCUSender()` (more ceremony for one
consumer), and logging inline in `updateRudder()` under `#if BENCHDEBUG` (puts console output in the
CAN layer). The slot keeps `CAN` output-agnostic and keeps printing where the console lives.

The whole addition is `#if BENCHDEBUG`-guarded, so the production DCU build is byte-identical.

### Console behaviour

- `rw` arms the watch: prints `Rudder watch on - press any key to stop`, discards any stale sample so
  only frames received *while watching* are shown, and forces the first such frame to print.
- While armed, `handleUserInput()` does not parse commands. Any byte on `Serial` drains the input,
  disarms the watch, and prints `Rudder watch off`.
- `loop()` polls `takeRudderSample()` while armed and prints one line per **changed** value triple:
  `Rudder <n> lBrk <n> rBrk <n>`. RudderCAN resends periodically even when idle; suppressing unchanged
  triples keeps the console readable while the pedals are still.
- Printing uses `Serial.print(F(...))` plus raw integers — no `String`. String heap churn in a
  frequently-hit path is what froze the CAN link on other AVR nodes.
- `?` help gains `rw: watch rudder/toe brake input (any key stops)`.

### Out of scope

No CAN filter or message-ID changes — `rudder` (0x303) already passes filter 4 in `CAN::begin()`.
No serial-protocol changes; `SerialMessageId.h` is untouched.
