# Ticket: steppers never de-energise at rest

**Status:** open. Written 2026-09-30, during the final review of `feature/hsi-can`. Repo-wide, not a
single-board fix.

**Type:** power/heat cleanup, no functional change to instrument behaviour when the fix lands.

## The bug

`CheapStepper::run()`'s `stepsLeft == 0` branch:

```cpp
else
{
    // off();
}
```

commented out, inherited unmodified from the upstream fork. Every board built on `CheapStepper` keeps
all four coils energised at rest, all the time — there is no idle timeout anywhere in the call chain.
Measured cost is roughly 200–300 mA per motor; `HSICAN` alone carries three (compass, OBS, HDG), and
an HSI's needles sit still far more often than they move. The only way to de-energise today is a bench
command that calls `off()` directly — nothing automatic.

## Why it's a ticket and not a line in this branch

`CheapStepper` is vendored as a separate copy into every board that uses it — currently
`AltimeterCAN`, `AltimeterDriver`, `HSICAN`, `HSIDriver`, `StepperBoard`, and `WhiskeyCompassCAN`'s
`lib/CheapStepper`. An idle-shutdown timeout is the kind of change that has to land in all of them
together or not at all: half the boards holding torque while the other half don't is a worse, more
confusing state than the current uniform (if wasteful) always-on behaviour. That is a repo-wide sweep,
out of scope for a branch about one board's CAN conversion.

## Shape of the fix, once picked up

Re-enable `off()` in the `stepsLeft == 0` branch, gated behind a short idle delay (a few hundred ms)
rather than firing on the very first zero-steps tick, so a multi-move sequence with brief pauses
doesn't re-energise every step. `moveSteps()`/`moveDegree()` callers already tolerate the coil
sequence being re-primed on the next move (homing does this today after every `stop()` +
`resetPosition()`), so waking from off should not need new logic beyond what `stepCW()`/`stepCCW()`
already do.

## Open question — don't assume it away

A 28BYJ-48 behind its 64:1 gearbox normally holds position with no holding current; that's the whole
premise for doing this. But that's a bench observation on a handful of units, not a datasheet
guarantee, and the failure mode if it's wrong is quiet: a needle creeping off its commanded position
would only be noticed in flight, not on the bench where it's usually checked stationary right after a
move. Whoever picks this up should watch a homed, idle instrument for drift over an extended period
(not just spot-check it) before calling the fix done across all six boards.
