# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

See also the monorepo-level `../CLAUDE.md` for repo-wide conventions, and in particular its
**Motion chain** section for the protocol shared with `MotionGateway` and `MotionActor`.
This file covers what's specific to MotionProviderPlugin.

## Commands

Not a PlatformIO project. Run from this directory:

```bash
./build-macos.sh              # release build
./build-macos.sh debug        # debug build with symbols
./build-windows.sh            # native build on Windows (MSVC)
./build-xc-windows.sh         # cross-compile for Windows from macOS (MinGW)
./build-all.sh                # all platforms
```

The X-Plane SDK is the shared `../XPlaneSDK` — do not vendor a per-plugin copy.

Unit tests (host build, no X-Plane, no rig):

```bash
cmake -S tests -B tests/build && cmake --build tests/build && ctest --test-dir tests/build
```

Eleven suites: `kinematics`, `config`, `washout`, `effects`, `bff`, `safety`, `monitor`, `heartbeat`,
`armramp`, `armgate`, `telemetry`. They link the real source files, so a behaviour change shows up
here first.

## What the plugin is

X-Plane 12 plugin that turns flight state into 6DOF platform motion. One flight-loop callback at
60 Hz (`Plugin.cpp`, `kFlightLoopIntervalSec`) runs the whole chain:

```
DataRefManager ─▶ WashoutFilter ─┐
                 EffectsLayer  ──┴─▶ pose ─▶ ArmRamp blend ─▶ StewartKinematics (IK)
                                                            ─▶ SafetyLimiter ─▶ BffEncoder ─▶ SerialLink
```

`SafetyMonitor` and `ArmGate` sit alongside as the watchdog/latch path.

## Configuration

`configuration.toml` lives in the plugin directory (resolved from the plugin's own `.xpl` path), is
seeded with defaults on first run, and is re-read by the **"Reload config"** button in the status
window — no X-Plane restart, no rebuild. Sections: `[geometry]`, `[servo]`, `[washout]`, `[effects]`,
`[serial]`, `[safety]`, `[telemetry]`.

Reloading resets the stateful filters, so a reload during flight starts the washout from a clean
pose rather than jumping.

## Washout filter (`WashoutFilter.cpp`)

Classical washout: specific forces and body rates in, platform pose out. Stateful; all time
dependence goes through the `dt` argument, and there are no X-Plane dependencies — which is what
makes it host-testable and offline-replayable.

- **Heave** — `(g_nrml − 1)` high-passed, then leaky double integration to mm.
- **Tilt coordination** — sustained surge/sway specific force low-passed into a rate-limited pitch/roll
  tilt, using gravity to sustain a translational cue the platform cannot otherwise produce. The
  low-pass now runs on the raw surge/sway specific force, with the tilt gain applied outside it —
  so the same filtered signal feeds both tilt and the translational onset channel below without
  the gain baking itself into the corner frequency.
- **Translational onset** — the complement of the tilt-coordination low-pass, leaky
  double-integrated to mm and clamped per axis. Renders the onset of a longitudinal or
  lateral acceleration, which tilt coordination cannot: it is low-passed and rate-limited
  by design. Crossover constant is `tilt_lp_tau`, shared with the tilt half, so the two
  cannot double-count. Off by default (`surge_gain`/`sway_gain` = 0). Design:
  `../docs/superpowers/specs/2026-08-31-surge-sway-onset-cues-design.md`.
- **Rotational** — body rates high-passed, integrated, then leaked back toward centre.
- **Output smoothing** — two cascaded one-pole low-passes (`smooth_tau`), currently one shared
  constant for all DOF.

**The washout parameters here are tuned, not defaults.** A 2026-08-30 campaign found four separate
causes of the roughness the platform used to have — a permanently saturated heave channel (fixed by
`heave_*_washout_tau = 0.25`), a tilt rate above the ≈3 °/s rotation-detection threshold
(`tilt_rate_limit_dps = 3`), and two effects that demanded more acceleration than the hardware can
produce. **Read `../docs/motion-tuning/tuning-log.md` before changing any of them**: every value has
a recorded rig verdict, and several obvious-looking moves have already been tried and rejected
(raising `heave_gain`, and the actuator-space velocity/acceleration caps, which add lag by
construction). Design spec: `../docs/superpowers/specs/2026-08-29-motion-heave-tuning-design.md`.
Recording, replay, sweep and measurement harness: `../docs/motion-tuning/README.md`.

Two structural notes that matter when changing this file: the limit clamps write back to **integrator
state** rather than only to the output (windup with no anti-windup), and
`StewartKinematics::clampToReachable` scales all six DOF together by a bisection factor, so one
saturating DOF attenuates the others. `surge_limit_mm`/`sway_limit_mm` bound the translational onset
channel's own contribution (`sat_surge`/`sat_sway` in the tuning harness), but they are sized from
typical (1st-percentile) headroom, not the worst case — measured near-worst-case headroom can fall
below the limit (see `docs/motion-tuning/baseline-metrics.md`) — so with the channel enabled,
`clampToReachable`'s all-DOF bisection can still engage and scale heave and tilt down with it.
`sat_envelope` is what detects that case; it is not guaranteed to stay at zero.

## The acceleration budget (`EffectsLayer.cpp`, and anything that adds motion)

`SafetyLimiter` caps the change in actuator velocity per tick at `max_acceleration_cps2 * dt`. At the
shipped 120 000 counts/s² and the measured 330.7 counts/mm, that is **≈363 mm/s² of heave, hard**.

Peak acceleration of any oscillating or pulsed cue goes as **`(2πf)² · A`** — with the *square* of
the frequency, so frequency is the strong lever and amplitude the weak one. Largest renderable
amplitudes: 12 Hz → 0.064 mm, 8 Hz → 0.144, 6 Hz → 0.255, 4 Hz → 0.575, 3 Hz → 1.021.

Two effects were written without checking this and both were 14× over: the 12 Hz ground rumble
(now `rumble_gain = 0`) and the 6 Hz touchdown bump (still shipped, still clipping). Neither
produced the motion it was named for — the limiter turned each into a slew-limited zigzag, felt as
harshness. **Size any new effect against this budget before tuning it**, and remember the washout is
spending part of the same budget concurrently, so an effect cannot have all of it: the slab-joint
effect measured 39 % limiter engagement at a 300 mm/s² budget and 15 % at 200.

What the pilot feels is acceleration, not displacement. 363 mm/s² is roughly 25–35× the perception
threshold for vertical whole-body vibration, so a cue of a few hundredths of a millimetre is still
clearly felt if it uses the budget. Specify effects in acceleration and let the displacement follow.

## Timing and dt

`elapsedSec` from the flight loop is the real timestep; it is clamped to `max_dt_sec` before reaching
the stateful filters so an X-Plane stall cannot diverge them. Serial reconnect timing uses the
unclamped value. While the sim is paused the callback keeps firing with wall-clock dt against a
frozen flight model — the pose is held instead of integrated.

`onAircraftLoaded` resets washout and effects: stale state from the previous flight would otherwise
take 15–20 s of its own decay constants to wash out, as large jerking right after a flight starts.

## Serial link (`SerialLink.cpp`)

Event-driven, not polled: `setFrame()` marks the frame dirty and signals a condition variable, the
I/O thread writes immediately, capped at `rateHz`, with a ~10 Hz keepalive when the flight loop goes
quiet. One clock, not two — two free-running 60 Hz clocks beating against each other were a measured
source of jitter.

Two extras support the goto path:

- `sendOneShot()` — sends a single frame immediately (used for the `BG` goto frame).
- `holdStream(bool)` — suspends streaming while a profiled move runs on the actors.

**Ordering rule: always `holdStream(true)` before `sendOneShot()`**, and release the hold only after a
fresh frame has been set — releasing earlier lets the I/O thread write the stale pre-goto frame and
produces a jerk at the end of the move.

`HeartbeatDecoder` reads the gateway's 500 ms heartbeat, which carries the physical arm-switch
position. The plugin trusts that position only from **fresh** frames; while the heartbeat is stale it
freezes the last known value, so a transient gap is not mistaken for the pilot cycling the switch.

## Arm / disarm

`ArmRamp` blends between the park pose and the live pose; `ArmGate` latches disarm on a fault and
clears only on a genuine armed→disarmed switch transition (a physical E-stop reset: flip off, then
on). Arming requires a fresh heartbeat **and** the switch armed **and** no latch.

Entering `Arming` or `Disarming` fires **one profiled goto** to the target pose instead of streaming
the blend (`startGotoTransition`). A mid-move reversal simply starts a new goto with the full
duration; the Kangaroo re-profiles from wherever it is. Durations come from `arm_ramp_sec` /
`disarm_ramp_sec`, clamped to 0.1..30 s.

## Tools

- `tools/bff_sender_test.cpp` — host harness linking `SerialLink`/`SerialPort`/`BffEncoder`/
  `HeartbeatDecoder`, with a deliberate 1 s stall to exercise the keepalive path. Build line is in
  its header comment. Test against a `socat` pty.
- `tools/measure_bff_timing.py` — distinct-vs-keepalive frame timing statistics with PASS/FAIL.
