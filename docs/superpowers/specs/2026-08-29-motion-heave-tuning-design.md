# Motion Heave Tuning Campaign — Design Spec

**Date:** 2026-08-29
**Status:** Design approved in chat; implementation not started.
**Scope:** MotionProviderPlugin (telemetry, replay harness, washout structure), plus a
staged tuning campaign executed partly in a separate session on the Sim-PC.

## Problem

In flight the platform's heave channel is harsh and pumps: roughly 2–4 s pushing up,
then 2–4 s pushing down, felt as air-pocket-like turbulence even with benign weather set
in the sim. Pitch/roll/yaw are acceptable by comparison.

Previous attempts to soften it went through `heave_gain` (0.5 → 0.15) and through the
downstream `max_velocity_cps` / `max_acceleration_cps2` limiters. The gain change did not
help; the limiter changes introduced perceptible lag. Both are the wrong lever, for reasons
the analysis below makes explicit.

**Hard constraint: no added perceptible lag.** This is expressed in the campaign as a
numeric gate, not as a judgement call.

## Analysis

### The heave channel saturates permanently

`WashoutFilter::update` builds heave as a high-passed vertical specific force fed into two
leaky integrators (`MotionProviderPlugin/src/WashoutFilter.cpp`):

```
aZ   = heaveGain * (g_nrml - 1) * G
aHp  = aZ - LP(aZ, heaveHpTau)                      // high pass
vel  = vel * leak(heaveVelWashoutTau) + aHp * dt
pos  = pos * leak(heavePosWashoutTau) + vel * dt * 1000
pos  = clamp(pos, ±heaveLimitMm)
```

As a transfer function from vertical specific force to heave position:

```
G(s) = s / ((s + 1/τ_hp) (s + 1/τ_vel) (s + 1/τ_pos))
```

With the shipped values `τ_hp = 1.0`, `τ_vel = τ_pos = 2.0` the poles sit at 1.0, 0.5 and
0.5 rad/s and the magnitude peaks at **|G| ≈ 0.91 s² at ≈ 0.067 Hz**.

The channel therefore stays inside `heave_limit_mm = 30` only while

```
0.15 · |g_nrml − 1| · 9.81 · 0.91 ≤ 0.030 m   →   |g_nrml − 1| ≤ 0.022 g
```

**±0.022 g is noise level.** In calm cruise `g_nrml` already wanders by ±0.05 g; any
control input reaches ±0.2…0.3 g. That is a **10–20× overdrive**, so the heave output is
saturated essentially all the time and behaves as a square wave slamming between +30 mm
and −30 mm.

### The observed period confirms it

An *unsaturated* system with these constants peaks at ≈ 0.067 Hz, i.e. a ~15 s period.
The reported behaviour is a 4–8 s period. That is not the linear resonance — it is the
**limit cycle of the saturated system**: `heavePos_` sits pinned at the clamp, and the only
thing governing how fast it comes back down is the decay of `heaveVel_`, i.e.
`heave_vel_washout_tau = 2`. Half-cycle ≈ τ. Predicted 2 s, observed 2–4 s.

Matching the *period* — not just the character — is what raises saturation from "plausible"
to "load-bearing". Stage 1 of the campaign still measures it before anything is changed.

### Why the previous attempts failed

- **`heave_gain` 0.5 → 0.15** divides by 3.3. Against a 10–20× overdrive that changes
  nothing about the saturation; it only makes the square wave's *source* smaller while it
  still clips.
- **`max_velocity_cps` / `max_acceleration_cps2`** are downstream limiters in actuator
  space (`SafetyLimiter`). When they engage they add phase lag by construction. They treat
  the symptom of an already-saturated signal and pay for it in latency. That is exactly the
  observed "gets laggy fast" outcome.

The correct lever is the **washout time constants**, and counter to the "soften it"
instinct they must get *shorter*, which simultaneously de-saturates the channel and
*reduces* phase lag:

All excursion figures below assume **`heave_gain = 0.15`**, the value in the current
`configuration.toml` — not the `WashoutConfig` struct default of 0.5. Excursion scales linearly
with the gain, so figures measured against the struct defaults are 3.3× larger.

| τ_vel = τ_pos | \|G\|max | sinusoidal peak at 0.3 g | sustained 0.3 g **step** |
|---|---|---|---|
| 2.0 (today) | 0.91 s² | ~400 mm → 13× over the clamp | ~286 mm (t ≈ 3.2 s) → 9.5× over |
| 0.5 | 0.14 s² | ~62 mm → 2× over | — |
| 0.3 | 0.06 s² | ~26 mm | ~46 mm (t ≈ 1.0 s) → still 1.5× over |

The two right-hand columns answer different questions and must not be conflated. `|G|max` is a
*sinusoidal* peak at the response maximum (≈0.067 Hz); a sustained step excites that band only
partially, so its peak is lower. The step figures are the analytic solution of
`A/((s+1/τ_hp)(s+1/τ_vel)(s+1/τ_pos))` and are what a bench measurement reproduces.

**Consequence for the campaign:** shortening τ alone does *not* fully de-saturate. At τ = 0.3 a
sustained 0.3 g step still reaches ~46 mm against a 30 mm limit. Stage 3 removes most of the
overdrive — from ~9.5× to ~1.5× — but the residue is what Stage 5's anti-windup and Stage 8's
amplitude choice have to absorb. Expect Stage 3 to change the character of the motion, not to
eliminate clipping outright.

**Measuring saturation is subtle:** `heavePos_` is a clamped *state*, so `heave_pos_raw` is only
"pre-clamp" within a single tick — the value it is built from was already clipped on the previous
tick. A raw excursion therefore never exceeds the limit by more than one integration step (~42 mm
measured at the shipped settings). To compare filter settings rather than the clamp, run with
`heave_limit_mm` set high enough to be inert. `heave_clamped` remains a valid saturation indicator
either way.

### Secondary findings

1. **The clamp writes back to integrator state.** `heavePos_ = clampd(heavePos_, …)`
   assigns to the filter state, not just to the output — integrator windup with no
   anti-windup. The reversal out of saturation is therefore instantaneous rather than
   smooth. The rotational channels have the same pattern (`angle = clampd(angle, …)`).
2. **`clampToReachable` scales all six DOF together.** The bisection in
   `StewartKinematics::clampToReachable` shrinks the whole pose uniformly, so heave
   excursions that leave the envelope also modulate the pitch/roll/yaw cues.
3. **`smooth_tau` is global.** One constant for heave and all rotations, which makes it too
   blunt to use: a value that helps heave over-damps the rotations.
4. **Heave is the only common-mode DOF.** It drives all six actuators in phase, so it is the
   worst case for drive load — a plausible contributor to why heave specifically feels hard
   while the differential DOF do not.

### Explicitly rejected

**An input low-pass on `g_nrml` is not part of this work.** Double integration attenuates
at −40 dB/decade; 10 Hz grain sits roughly 70 dB below the useful band. The heave problem
is low-frequency. A pre-filter would buy nothing and cost latency.

## Design

### Part 1 — Harness

Everything that can run on the Mac runs on the Mac. The Sim-PC is needed only for rig feel
and production deployment.

#### Telemetry in the plugin

New `MotionProviderPlugin/src/Telemetry.h/.cpp`. CSV writer, one row per flight-loop tick.
Start/stop from a Status-window button plus a `[telemetry]` section in `configuration.toml`
(output directory, auto-start).

Columns, in four groups:

| Group | Columns |
|---|---|
| Time | `t_sec`, `dt_real`, `dt_clamped` |
| Cues (raw) | `g_nrml, g_axil, g_side, P, Q, R, theta, phi, onground, gs, rpm, alpha, paused` |
| Washout internals | `heave_a_hp, heave_vel, heave_pos_raw` (**pre-clamp**), `heave_clamped`, `tilt_pitch, tilt_roll, tilt_rate_active`, `rot_roll_raw, rot_pitch_raw, rot_yaw_raw`, `rot_roll_clamped, rot_pitch_clamped, rot_yaw_clamped` |
| Output | `pose_heave, pose_roll, pose_pitch, pose_yaw`, `reach_scale`, `sp0..sp5` (post-IK), `sent0..sent5` (post-SafetyLimiter), `sl_vel_clip`, `sl_acc_clip`, `arm_state` |

Getting the internals out requires additive accessors, no behaviour change:

- `WashoutFilter` gains a `struct WashoutTrace` and `const WashoutTrace& trace() const`.
- `StewartKinematics::clampToReachable` reports the bisection scale factor it applied.
- `SafetyLimiter` counts how many channels it limited this tick.

The existing `tests/` suite must stay green across this change — that is the proof it is
additive.

Writing at 60 Hz from the flight loop uses a buffered `ofstream` flushed once per second.
Whether that causes frame hitches is measured by the first recording itself, via `dt_real`
outliers. If it does, the write moves onto the existing serial I/O thread.

#### One format for recording and replay

The cue columns **are** the replay input. There is no second recording path: the replay tool
reads the same CSV, takes `dt_real` plus the cue columns, and ignores the rest. A recorded
file is therefore simultaneously a measurement log and a reproducible test case.

#### Replay CLI

`MotionProviderPlugin/tools/washout_replay`, a CMake target linking `WashoutFilter.cpp`,
`EffectsLayer.cpp`, `StewartKinematics.cpp` and `SafetyLimiter.cpp` **directly** — no second
implementation that can drift. None of those four depend on the X-Plane SDK, so it builds
anywhere. Config parsing uses the vendored `toml++`.

```
washout_replay --cues seg_cruise.csv --config configuration.toml \
               --set washout.heave_pos_washout_tau=0.5 --out run.csv

washout_replay --cues seg_cruise.csv --config configuration.toml \
               --sweep washout.heave_pos_washout_tau=0.3,0.5,1.0,2.0

washout_replay --synth chirp:0.02-5.0:0.3g --config configuration.toml --out chirp.csv
```

Additional flags:

- `--synth step|sine|chirp` generates cue streams analytically, so the filter's frequency
  response can be measured with no flight at all.
- `--resample-dt <sec>` re-runs a recording at a different timestep. Mac and Sim-PC run at
  different framerates; replay is faithful to whatever `dt` was recorded, but the *live*
  behaviour of a double integrator is dt-sensitive. This flag checks each candidate against
  a PC-typical framerate.
- `--cues-only` exports just the cue columns, for committing reference segments.

`MotionProviderPlugin/tools/washout_metrics.py` turns run CSVs into the metric table,
spectra and time-series plots.

#### The harness self-test

**Replaying a recorded CSV must reproduce that recording's `pose_*` columns bit-exactly.**
If this fails, offline and live paths have diverged and every number afterwards is
worthless. This test gates all of Stage 0.

### Part 2 — Metrics and acceptance

Per DOF, per segment:

| Metric | What it shows |
|---|---|
| `sat_pct` | Fraction of ticks in a clamp, reported per source (heave limit, rot limit, tilt rate, envelope scaling, SafetyLimiter vel/acc). The primary diagnostic. |
| `wrms` | Band-weighted RMS of heave acceleration, emphasising 0.1–0.63 Hz. The discomfort measure. **Not a conformant ISO-2631 Wk implementation** — it is a documented band emphasis chosen because that is where ISO-2631 puts its vertical peak. Adequate for ranking candidates against each other, which is all it is used for; it is not a comfort figure to quote elsewhere. |
| `band_ratio` | Share of heave power in 0.1–0.5 Hz. States directly whether we sit in the motion-sickness band. |
| `jerk_p95` | 95th percentile of jerk per actuator. The mechanical harshness measure. |
| `lag_ms` | Cross-correlation delay from cue to commanded pose, per DOF. The lag budget. |
| `range_used_pct` | Fraction of the envelope actually used. For the amplitude stage. |

**Gates applied to every candidate:**

1. `lag_ms` ≤ baseline + **15 ms**. Hard limit — a candidate that breaks it is never flown.
   This is what turns "no more lag" into a number.
2. `sat_pct`, `wrms` and `jerk_p95` must all decrease. **Stage 8 inverts this gate:**
   raising `heave_gain` necessarily raises motion, so there the criterion is *thresholds
   held* rather than *values reduced* — heave `sat_pct` stays under its target, `jerk_p95`
   stays at or below the Stage-7 value, and `wrms` may rise as long as `band_ratio` does
   not (more cue is fine; more energy in the sickness band is not).
3. **Rig veto.** If it feels worse on the rig, feel wins over the number. The case gets
   logged: it means a metric fails to capture the problem, which is itself a result.

Target for the diagnosis stage: heave `sat_pct` below **2 %** in the calm-cruise segment.
Today's expectation there is 80–100 %.

### Part 3 — Reference material

Recorded on the Mac's X-Plane 12, with the rig disconnected (the plugin stays disarmed; the
cues are identical either way).

- **Six isolated segments, 60–90 s each:** calm straight-and-level, steep turns,
  climb/descent alternation, defined turbulence, ground roll + takeoff, approach + landing.
  Calm straight-and-level is the most important: by definition nothing may pump there.
- **One continuous 8–10 min mixed flight** as the acceptance case, catching interactions and
  slowly accumulating state that the short segments miss.

**Aircraft: Piper Arrow III (PA28-201R), the vFlightAir custom add-on.** Not a stock X-Plane
model, so the cue characteristics come from a third-party flight model. Every baseline number
in this campaign is therefore valid *for this aircraft*; a different type would need its own
baseline before its numbers mean anything. Weather preset and X-Plane version are recorded in
the tuning log at capture time alongside it.

For rig flights, reproducibility comes from a saved `.sit` at a fixed point, a fixed weather
preset and an autopilot-flown profile where possible. That is enough for the *subjective*
verdict; the objective verdict is offline anyway. This is deliberately not an X-Plane replay
recording — a flight flown twice is never the same twice, while a recorded cue file is
bit-reproducible.

### Part 4 — Campaign stages

Ordered by effect rather than effort: the dominant cause first via free config knobs, then
the structural code changes, then amplitude.

| # | Stage | Knob | Where |
|---|---|---|---|
| 0 | Build the harness | — | Mac, code |
| 1 | Verify the analysis | — (synthetic cues) | Mac |
| 2 | Record and measure baseline | — | Mac |
| 3 | Washout time constants | `heave_vel_washout_tau` + `heave_pos_washout_tau` | Mac → **PC** |
| 4 | High pass | `heave_hp_tau` | Mac → **PC** (same visit as 3 — both are TOML-only) |
| 5 | **SW 1:** anti-windup + soft saturation | code | Mac (rig check rides with 6) |
| 6 | **SW 3:** per-DOF `smooth_tau`, then sweep the heave value | code + config | Mac → **PC** |
| 7 | **SW 4:** decouple envelope scaling | code | Mac (rig check rides with 8); skipped unless `reach_scale < 1` is common |
| 8 | Bring amplitude back | `heave_gain` | Mac → **PC** |
| 9 | Cross-check rotational/tilt channels and effects | various | Mac |
| 10 | Acceptance and freeze the config | — | **PC** |

**Stage 1** tests the analysis before the plan leans on it: a 0.02–5 Hz chirp and g-steps
through the washout, expecting `|G|max ≈ 0.91 s²` at ≈ 0.067 Hz and clamp onset at
`|Δg| ≈ 0.022 g`. If the measurement disagrees, the diagnosis was wrong and the campaign is
re-cut from here. That outcome is explicitly allowed for.

**Stage 3 deliberately moves two values together.** `vel_tau` and `pos_tau` are physically
one pair — the two poles of the same washout — so sweeping them separately would be a 2D
grid with no extra insight. They stay coupled (`τ = 2.0 → 1.0 → 0.6 → 0.4 → 0.25`) and are
decoupled only if the coupled sweep shows no clear optimum. This is the single deliberate
exception to the one-knob-per-change rule.

**Stage 5 is expected to be barely perceptible on its own**, and that is fine. Anti-windup
removes the hard corner at the reversal point, but while the channel is 13× overdriven the
corner is not what hurts. It therefore comes *after* Stage 3, its gate is purely objective
(`jerk_p95` at reversals falls, `lag_ms` unchanged), and it costs no dedicated rig flight —
it travels with Stage 6.

The soft saturation uses a soft knee rather than a plain `tanh`: `tanh` compresses well below
the limit and would cost amplitude. Linear up to `a · limit`, then smoothly asymptotic to
`limit`, with `heave_soft_knee` (default 0.6) as the knob. Anti-windup means the integrator
state is no longer clamped — the leak alone bounds it — and the saturation applies to the
output.

**Rig sessions:** after Stage 3, bundled after 5+6, after Stage 8, plus acceptance. Four rig
visits rather than twenty. Within a session several candidates run without a rebuild: edit
the TOML, press "Reload config", fly.

### Part 5 — Repository artefacts

| Path | Purpose |
|---|---|
| `docs/superpowers/specs/2026-08-29-motion-heave-tuning-design.md` | This design |
| `docs/superpowers/plans/2026-08-29-motion-heave-tuning.md` | The executable plan, stage by stage |
| `docs/motion-tuning/README.md` | Harness operating manual — what a fresh session reads first: how to record, sweep, read metrics, promote a candidate to the rig |
| `docs/motion-tuning/tuning-log.md` | **Living document.** One row per change: date, stage, parameter old→new, offline metrics, rig verdict, decision. Extended on the Sim-PC and committed |
| `docs/motion-tuning/baseline-metrics.md` | Frozen baseline numbers as the comparison anchor |
| `MotionProviderPlugin/reference/*.csv.gz` | Recorded segments, cue columns only, gzipped — ~2 MB total, small enough to commit |

The cue export is deliberately narrow: full telemetry would be ~70 MB, while the 14 cue
columns gzip to ~2 MB. Because replay is deterministic, full telemetry can be reconstructed
from them at any time, so the large files never enter git.

## Failure cases

- **Stage 1 contradicts the analysis.** The campaign stops and is re-cut. No tuning stage
  runs on an unverified diagnosis.
- **Telemetry causes frame hitches.** Detected via `dt_real` outliers in the first recording;
  the writer moves to the serial I/O thread.
- **Harness self-test fails (replay ≠ recording).** Blocking. Nothing proceeds until offline
  and live paths agree bit-exactly.
- **A candidate wins offline and loses on the rig.** Logged as a metric gap, not overridden.
  Repeated occurrences mean the metric set needs a new member.
- **De-saturation makes cues feel weak.** Expected and handled by Stage 8, which raises
  `heave_gain` until the gates break.

## Out of scope

- Input pre-filtering of `g_nrml` (rejected above, with reasoning).
- Actuator-side or Kangaroo-side changes. The transport chain was verified clean in the
  preceding campaign; this one stays in the plugin.
- The separate documentation gap in the motion chain (neither the root `CLAUDE.md` nor the
  board `CLAUDE.md` files describe the motion chain or the `BG` / `actorPairGoto` protocol).
  Tracked, not addressed here.
