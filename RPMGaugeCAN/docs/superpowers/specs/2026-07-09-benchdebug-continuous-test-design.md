# BenchDebug Continuous Test Mode (`co`/`cx`) — Design

## Purpose

Add a bench-console command pair, `co` and `cx`, that exercises the RPM gauge motor
and the odometer display continuously and unattended, for burn-in / stress testing
on the bench without needing to issue `rp`/`od`/`cl` commands by hand.

- `co<seconds>`: starts continuous test mode. `<seconds>` is the odometer start
  value, parsed the same way as the existing `od<seconds>` command.
- `cx`: stops continuous test mode.

While running, the odometer display advances by one second, once per real second,
starting from the `co` value. Simultaneously, the RPM needle performs random
moves: after each move finishes, it waits a random 500–2000ms, then moves to a
random position over the full needle range.

## Non-goals

- No change to CAN mode, `RPMGauge`'s normal RPM-driven behavior, or the odometer's
  CAN-driven behavior. This is BenchDebug-only (`#if BENCHDEBUG`), matching every
  other command in that file.
- No persistence of test state across resets/power cycles.

## API additions

`RPMGauge` gains one new public passthrough method:

```cpp
bool isMoving(); // returns motor->isMoving()
```

No other changes to `RPMGauge`. The random needle move itself reuses the existing
calibration path, `moveNeedle(uint16_t degree, true)`, which already bypasses the
RPM→degree mapping and takes a plain degree value (0..kMaximumDegree) — exactly
what a random-move exerciser needs. No raw step-space access is exposed to
`BenchDebug`.

## BenchDebug state additions

```cpp
bool continuousTestActive = false;
float continuousTestStartSeconds = 0;
uint32_t continuousTestElapsedSeconds = 0;
uint32_t lastSecondTick = 0;      // millis() of the last odometer tick
uint32_t nextMoveTime = 0;        // 0 = no move currently scheduled
```

These live alongside the existing `heartbeat`/`heartbeatLedOn` members, same
pattern (millis()-based interval tracking, no timers/interrupts).

## Command behavior

### `co<seconds>`

- Parse `<seconds>` with `atof`, same as `od`.
- Sets `continuousTestActive = true`.
- Sets `continuousTestStartSeconds` to the parsed value.
- Resets `continuousTestElapsedSeconds = 0`, `lastSecondTick = millis()`,
  `nextMoveTime = 0`.
- Immediately displays the start value on the odometer (same call as `od`):
  `odometer->secondsToDigits(continuousTestStartSeconds, digits); odometer->displayNumber(digits);`
- Can be re-issued while already active: this simply restarts the test with the
  new start value (not blocked — see blocking rules below).

### `cx`

- Sets `continuousTestActive = false`.
- No other side effects: any in-flight needle move finishes naturally (next
  `rpmGauge->loop()` calls just stop being followed by new scheduled moves), and
  the odometer display is left showing its last value.
- If typed while not active, it's a silent no-op (no error message).

### Blocking other commands while active

While `continuousTestActive` is true, `rp`, `cl`, `br`, and `od` are rejected:
print `"Continuous test running. Type 'cx' to stop."` and treat the command as
handled (does not fall through to the generic "Unknown command" message).
`?` and `cx` are never blocked. `co` is never blocked either (see re-issue
behavior above).

This check happens at the top of `handleRPMGaugeInput`, before the existing
`rp`/`od`/`cl`/`br` dispatch chain, gated on `strncmp` matching those four
prefixes specifically (not a blanket "anything but co/cx/?").

### Help text

`?` gains two new lines documenting `co<seconds>` and `cx`.

## Runtime behavior (inside `BenchDebug::loop()`)

A new block, alongside the existing LED-heartbeat block, runs only when
`continuousTestActive`:

1. **Odometer tick** (once per real second, via `lastSecondTick`):
   ```cpp
   if (millis() - lastSecondTick >= 1000L) {
       lastSecondTick += 1000L;
       continuousTestElapsedSeconds++;
       float digits[6];
       odometer->secondsToDigits(continuousTestStartSeconds + continuousTestElapsedSeconds, digits);
       odometer->displayNumber(digits);
   }
   ```

2. **Random needle move scheduler** (every `loop()` call), adapted directly from
   the reference sample:
   ```cpp
   if (!rpmGauge->isMoving()) {
       if (nextMoveTime == 0) {
           nextMoveTime = millis() + random(500, 2000);
       } else if (millis() > nextMoveTime) {
           nextMoveTime = 0;
           rpmGauge->moveNeedle(random(0, kMaximumDegree), true);
       }
   }
   ```

`rpmGauge->loop()` (already called unconditionally in `main.cpp`) continues to
drive `motor->loop()` regardless of BenchDebug mode, so no change is needed there.

## Edge cases

- **Invalid/missing numeric value on `co`** (e.g. bare `co`): `atof("")` returns
  `0`, so the test starts at 0 seconds — same fallback behavior `od` already has,
  no special-case needed.
- **`cx` with no `co` ever issued**: no-op, as stated above.
- **Random target draws the same degree as current position**: `moveNeedle`'s own
  `isMoving()` guard and the underlying motor library already handle this as a
  zero-distance move; no special-case needed here.

## Testing

No automated tests exist for this project (per repo convention — `test/` dirs are
scaffolds). Verification is manual on the bench: issue `co0`, confirm the
odometer counts up once/sec and the needle sweeps randomly with pauses; issue
`rp1000` and confirm it's rejected with the guidance message; issue `cx` and
confirm the needle finishes its move and stops, and the odometer freezes at its
last value.
