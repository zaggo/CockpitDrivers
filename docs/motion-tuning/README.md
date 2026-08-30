# Motion Heave Tuning Harness — Operating Manual

Read this before touching anything in `MotionProviderPlugin/tools/` or `configuration.toml`'s
`[washout]`/`[safety]`/`[effects]` sections. It is the entry point for the heave tuning
campaign described in `docs/superpowers/specs/2026-08-29-motion-heave-tuning-design.md`
(the diagnosis and the stage plan) and
`docs/superpowers/plans/2026-08-29-motion-heave-tuning.md` (the executable, stage-by-stage
task list). This file explains *how to use the harness*; the campaign log is
`docs/motion-tuning/tuning-log.md`.

## The two rules that override everything below

1. **`--verify` must pass against a real recording before any number from that recording is
   trusted.** A replay that doesn't reproduce what the plugin actually computed is measuring
   the replay tool, not the platform.
2. **`lag_ms` must stay within baseline + 15 ms.** Hard gate, no judgement call — a candidate
   that breaks it is never flown, no matter how good its other numbers look.

## Setup

The two tools are host-side (no X-Plane, no rig) and live in `MotionProviderPlugin/tools/`.

**Python is a venv, not the system interpreter.** Always invoke
`MotionProviderPlugin/tools/.venv/bin/python`, never bare `python3` — the system interpreter
has no numpy, and `washout_metrics.py` is numpy-only. The venv is gitignored; if it's missing,
recreate it (`python3 -m venv tools/.venv && tools/.venv/bin/pip install numpy`).

Build the replay CLI:

```bash
cd MotionProviderPlugin
cmake -S tools -B tools/build && cmake --build tools/build
```

This produces `tools/build/washout_replay`. It links `WashoutFilter.cpp`, `EffectsLayer.cpp`,
`StewartKinematics.cpp`, `SafetyLimiter.cpp`, `MotionConfig.cpp` and `Telemetry.cpp` directly
from `../src` — there is no second implementation of the cueing chain that could drift from
the plugin's.

Build and run the plugin's own test suite (proof that nothing in Part A of the campaign
changed the filters' numerical behaviour):

```bash
cd MotionProviderPlugin
cmake -S tests -B tests/build && cmake --build tests/build
ctest --test-dir tests/build --output-on-failure
```

Eleven suites: `kinematics`, `config`, `washout`, `effects`, `bff`, `safety`, `monitor`,
`heartbeat`, `armramp`, `armgate`, `telemetry`. All eleven must stay green for every campaign
change up through Stage 4 (TOML-only stages); Stages 5+ add code and extend this suite as they go.

## 1. Recording in X-Plane

- Load the aircraft the campaign's baseline is built on: **Piper Arrow III (vFlyteAir
  PA28-201R)**. Every number this campaign produces is valid for this aircraft only — a
  different flight model needs its own baseline before its numbers mean anything.
- Start recording with the **Record** button in the plugin's Status window, or set
  `enabled = true` under `[telemetry]` in `configuration.toml` to auto-start as soon as the
  plugin loads (useful for the Sim-PC, where nobody may be at the keyboard to click Record).
  `dir` in that section picks the output directory; leave it empty to write next to
  `configuration.toml`.
  **Caveat on `enabled = true`:** a recording that starts at plugin load necessarily starts
  *disarmed* and therefore spans the arm edge, where `MotionProvider::onFlightLoopTick` resets
  the washout and effects filters. `washout_replay` models that reset from the `arm_state`
  column, so such a file still passes `--verify` — but only if `arm_state` survived into the
  file you replay. A cue-only export made with the `cut` in §2 does **not** carry it, so verify
  against the full recording, not the export. Pressing **Record** after arming avoids the
  question entirely and is the cleaner habit for a reference segment.
- Files are named `motion-YYYYMMDD-HHMMSS.csv` at the moment recording starts.
- **Record in SIM mode, not manual/bench-jog mode.** In manual mode
  (`MotionProvider::manualMode_`) the flight loop drives the pose directly from the hand
  controls and never calls `WashoutFilter::update`/`EffectsLayer::update` — so a recording
  made there has its `live_*` and `eff_*` columns permanently disconnected from the `cmd_*`
  the platform actually followed. Such a file will fail `--verify` and any metric computed
  from `live_heave` will be meaningless.
- **Do not pause X-Plane during a recording.** The recorder keeps writing rows while paused —
  every row still has to exist for `--verify`'s row-for-row comparison to work — but the
  washout state is frozen, so those rows just repeat whatever the last unpaused tick left
  behind. Both `washout_replay` and `washout_metrics.py` exclude paused ticks from the `sat_*`
  saturation statistics (using the recorded `paused` column), but the spectral (`wrms`,
  `band_ratio`) and lag (`lag_ms`) metrics run over the *whole* time series — excising
  scattered paused rows would corrupt the time axis the FFT and cross-correlation depend on,
  which is worse than leaving them in. A pause anywhere in the segment distorts those numbers.
- Press **Stop Rec** to close the file. Check `dt_real` for outliers as a sanity check —
  the campaign's first recording is what tells us whether 60 Hz CSV writes cause frame
  hitches; if they do, the fix is moving the write onto the serial I/O thread, not a change
  to how you record.

## 2. Exporting a cue-only reference file

The design originally planned a `--cues-only` flag; it ended up folded into the recording
format instead, because the cue columns are already a strict, contiguous-enough subset of
every recording — a cue-only export is just a column selection over the same CSV, not a
separate tool.

`washout_replay`'s cue reader (`loadCues`) uses exactly these columns: `dt_real`, `g_nrml`,
`g_axil`, `g_side`, `P`, `Q`, `R`, `theta`, `phi`, `onground`, `gs`, `rpm`, `alpha`, `paused`.
In the header these are fields 2 and 4–16 (field 3, `dt_clamped`, is not a cue and is skipped):

```bash
cut -d, -f2,4-16,58-61 motion-20260901-140322.csv | gzip > MotionProviderPlugin/reference/cruise_calm.csv.gz
```

Because replay is a deterministic function of `(cues, dt, config)`, full telemetry can always
be reconstructed from a cue-only file by replaying it — so the ~70 MB full recordings never
enter git, only the ~2 MB gzipped cue files under `MotionProviderPlugin/reference/`. Two
things a cue-only export cannot do, precisely because it is cue-only: it is **not** valid input
to `washout_metrics.py` (§7 — replay it first), and it drops `arm_state`, so it cannot be the
input to `--verify` on a recording that spans an arm edge (§1, §5). If
`Telemetry::header()` ever changes column order, regenerate this `cut` invocation from the
new header before trusting it — it is a position-based cut, not a name-based one.

## 3. Running a single replay

```bash
cd MotionProviderPlugin
./tools/build/washout_replay --cues reference/cruise_calm.csv --config configuration.toml \
    --set washout.heave_pos_washout_tau=0.5 --out /tmp/run.csv
```

(The examples here and in §4 name `reference/cruise_calm.csv` for brevity. The committed file
is `reference/cruise_calm.csv.gz`; `washout_replay` has no zlib, so expand it first with
`gunzip -c reference/cruise_calm.csv.gz > /tmp/cruise_calm.cues.csv` and point `--cues` there.)

`--set section.key=VALUE` is repeatable and overrides one config value in memory; it never
writes back to `configuration.toml`. Valid keys are the `washout.*`, `safety.*` and
`effects.*` fields in `washout_replay.cpp`'s `kWashoutKeys`/`kSafetyKeys`/`kEffectsKeys`
tables — an unrecognised key is refused with exit code 2, not silently ignored.

`--resample-dt SEC` re-runs the same cues at a fixed timestep instead of the one the file was
recorded at (e.g. `--resample-dt 0.0111` for 90 fps, `--resample-dt 0.0333` for 30 fps). The
Mac and the Sim-PC run at different framerates, and a double integrator's live behaviour is
dt-sensitive — this flag is how a candidate gets checked against a PC-typical framerate before
it is ever flown.

## 4. Sweeping a parameter

```bash
cd MotionProviderPlugin
./tools/build/washout_replay --cues reference/cruise_calm.csv --config configuration.toml \
    --sweep washout.heave_pos_washout_tau=2.0,1.0,0.6,0.4,0.25 --out /tmp/sweep
```

Prints one summary row per value (`sat_heave%`, `peak_raw_mm`, `samples`) and, because `--out`
was given, writes a full telemetry CSV per value as `/tmp/sweep.<value>.csv` for
`washout_metrics.py` to chew on afterward. `--sweep` and `--verify` are mutually exclusive
(see below) — a sweep runs the chain once per value, so there is no single "the" replay to
verify against. If a per-value CSV cannot be written (unwritable `--out` directory), the sweep
stops with `cannot write ...` and exit code 1 rather than printing a table with nothing on disk
behind it. `sat_heave%` prints as `nan`, not `0.00`, if the run contained no unpaused tick at
all — 0 % is this campaign's target outcome and must never be what "no data" looks like.

**`peak_raw_mm` barely moves in a τ sweep, and that is not the sweep failing.** It is one of
only two numeric columns printed here, so it is easy to read it as "the overdrive" — it is not.
`WashoutFilter`'s `heavePos_` is a clamped *state*: the clamp writes back to the integrator, so
`heave_pos_raw` is "pre-clamp" only within a single tick and can exceed `heave_limit_mm` by at
most one integration step (~42 mm measured at the shipped settings), whatever the time
constants do. The design spec's 9.5× / 286 mm overdrive figures are *not* this column at these
settings. To make `peak_raw_mm` measure the filter instead of the clamp, run with the limit
made inert:

```bash
./tools/build/washout_replay --cues reference/cruise_calm.csv --config configuration.toml \
    --set washout.heave_limit_mm=1e9 \
    --sweep washout.heave_pos_washout_tau=2.0,1.0,0.6,0.4,0.25 --out /tmp/sweep
```

`sat_heave%` stays a valid saturation indicator either way — but not in the same run as the
inert limit, which by construction never clamps. Run the sweep twice if you want both.

Stage 3 of the campaign moves `heave_vel_washout_tau` and `heave_pos_washout_tau` together
(the two poles of one washout) rather than sweeping them independently — run the sweep above
for one, then repeat with the other pinned to the same value via `--set` on each invocation.

## 5. `--verify` — the harness self-test

```bash
cd MotionProviderPlugin
./tools/build/washout_replay --cues motion-20260901-140322.csv --config configuration.toml --verify
```

**`--config configuration.toml` means "the config as it is today", not "the config that recording
was flown with".** Those diverge the moment a value is adopted, so verifying an older recording
against the live file reports a mismatch that is a config difference, not a defect — reconstruct the
flown config and verify against that instead. This currently bites every recording made on the
ground before 2026-08-30: `configuration.toml` now carries `slab_spacing_m = 10`, an unflown
candidate, so replay them with `--set effects.slab_spacing_m=0` (and `effects.rumble_gain=0.9` for
anything recorded before the rumble was switched off).

Compares the replay's `live_heave/roll/pitch/yaw` against the recording's, tick for tick, over a
comparison window that skips a leading warm-up (see below), and reports
`verify: PASS (within 1e-04 mm/deg floating-point noise floor)` when the max divergence over that
window stays at or under that tolerance. It checks `live_*` and not `cmd_*` because `live_*` alone
is a pure function of `(cues, dt, config, arm edges)`; `cmd_*` also depends on the arm ramp blend,
which replay deliberately does not model (replay always runs "armed and live").

**The warm-up window, and why `--verify` is not a literal bit-for-bit check.** `runChain` always
starts the washout/effects filters from zero. A real recording usually does not: if Record is
pressed after the platform has been running for a while, row 1 already carries whatever pose the
filters had settled into (measured: `live_heave = 0.1356 mm` at row 1 of one campaign recording,
against replay's `0`). That is an unrecorded initial condition, not a reproduction defect — but it
takes real time to wash out, so `--verify` skips a warm-up window before comparing rather than
comparing from row 1.

The default window is derived from the config's own slowest time constant —
`10 × max(heave_hp_tau, heave_vel/pos_washout_tau, rot_hp_tau, rot_washout_tau, tilt_lp_tau,
smooth_tau)` — never a hard-coded number, because retuning exactly those constants is this
campaign's job; a fixed window would silently go stale the moment a candidate changed them.
Override it with `--verify-warmup SEC` (`0` compares everything, useful for seeing the raw
transient). `--verify`'s output line reports what it skipped on the same line as the result — how
many seconds and samples were excluded, and how many were compared — so the number is never a
silent gate. A recording shorter than its own warm-up is refused (`VERIFY REFUSED`) rather than
"passed" on whatever handful of rows happen to remain: fewer than 100 post-warm-up samples stops
the run.

Even past the warm-up, the divergence does not reach literal `0.0`: it decays asymptotically, and
on the campaign's real recordings a residual on the order of `1e-12`–`1e-5` mm/deg persists deep
into the file — traced to the rotational channels, whose washout carries the config's slowest
time constant (`rot_washout_tau`), showing isolated single-ULP float32-rounding blips from two
independently-converging trajectories. Demanding exact equality would make `--verify` fail on
every real recording forever, which defeats the point of this check, so the pass tolerance is
`1e-4` mm/deg — four orders of magnitude below the `0.415 mm` divergence a genuine reproduction
failure showed before this fix.

**The default warm-up is a guess, and on `cruise_calm` it guessed too short — final-half
discrimination fixes that.** Measured on the campaign's `cruise_calm` recording, the residual is
`5.98e-4 mm/deg` over the 30–50 s window right after the default 30 s warm-up, `2e-6 mm/deg` over
50–70 s, and `0` after 70 s: a large initial state (an unrecorded settled pose, same as the
`0.1356 mm` example above but bigger) that takes about `12.6×` the slowest time constant to fall
under the `1e-4` floor, not the `10×` the default computes. Raising the default factor would only
move the guess — and a longer fixed window eats into short segments — so instead `--verify`, after
the existing warm-up skip, additionally compares the residual over the **final half** of the
compared samples against the same `1e-4` floor:

- **Final-half residual under the floor** → the replay tracks the recording and the remaining
  overall error is an initial condition, not a reproduction defect. Reports `verify: PASS`, and
  states three numbers on the way there: the overall residual, the final-half residual, and the
  warm-up the *old* (pre-this-fix) criterion would have needed to pass this file outright
  (`--verify-warmup`'s equivalent, computed from t=0, not from whatever warm-up was actually
  used) — nothing about the early divergence is hidden.
- **Final-half residual over the floor** → `VERIFY FAILED`, same as before this fix.

`cruise_calm` now passes with the *default* warm-up (no `--verify-warmup 50` override needed):
final-half residual `7.45e-9 mm/deg`, and the reported "old criterion" warm-up is `34.0 s` — a
number that will drift as the config's time constants get retuned, which is exactly why it is
computed, not hard-coded.

**The hazard this has to guard against: a difference that decays *slowly* can look exactly like
an initial condition — even when it isn't one.** A final-half check alone would be fooled by a
divergence that stops being *excited* rather than actually decaying. This is not hypothetical: it
is exactly what happens on `ground_takeoff`. The recording's compared window (post-warm-up) is
about 30% on the ground followed by 70% airborne with the platform already climbing away; the
effects-layer rumble-phase mismatch (see Change 2 below) diverges hard (up to `1.74 mm/deg`)
throughout the ground portion and then reads as an **exact `0.0`** for the rest of the file the
instant `onground` goes false — not because anything decayed, but because the driving condition
went away. Split naively into two equal halves, the entire back half of this file happens to be
airborne: final-half residual `2.3e-10 mm/deg`, comfortably under the floor, which would have been
a false `PASS` — and a criterion that accidentally passed a real, measured defect would itself be a
defect. `--verify` therefore also partitions the compared window into 20 finer buckets and requires
the per-bucket maximum to be non-increasing (allowing only fluctuation that stays under the floor):
a genuine initial-condition transient is the filters' own linear homogeneous response to a wrong
starting *state*, which can only get smaller over time, so any regrowth above the floor — even
regrowth this particular file's tail doesn't happen to show — is proof the divergence is still
being driven by something in the cues, not fading state. On `ground_takeoff` this guard fires at
`t=31.6 s`, correctly turning the false PASS into a FAIL with an explanation distinct from a plain
still-diverging residual. Checked against all three of this campaign's currently-divergent
recordings (`ground_takeoff`, `approach_landing`, `acceptance` — all still FAIL, all for the Change-2
reason below) and the one decaying-transient recording (`cruise_calm` — no bucket ever regrows) before
landing on 20 buckets as the granularity.

**What replay does *not* model, and what that costs.** Replay drives the armed live pose from
tick 1, blend = 1, `arm_state` written as `Armed`. The plugin instead glides from the park pose
to the live pose over `arm_ramp_sec`. Three consequences, all confined to the actuator-space
columns (`sp*`, `sent*`) and the metrics derived from them:

- **A start transient.** The first `arm_ramp_sec` of replayed `sent*` is a slew-limited jump
  from the park seed to the live pose, which inflates `sat_sl_vel`, `sat_sl_acc` and early
  `jerk_p95`. Compare candidates over the same segment so the transient is common-mode, or
  drop the first couple of seconds; never quote an absolute `sat_sl_*` from a short replay.
- **`safety_->reset(gotoTargets_)` at goto completion.** When a profiled arm/disarm move
  finishes, the plugin re-seeds the `SafetyLimiter` from the arrived pose. Replay seeds the
  limiter once, from the park pose, and never re-seeds — so limiter state after any arm or
  disarm differs from the recording's.
- **During a goto the plugin records `sent*` values it never transmitted.** `serial_->setFrame`
  is skipped while `gotoActive_`, but the telemetry row is still written. Those rows are
  measurement artefacts in the recording itself, not just in the replay.

Together these make `jerk_p95` fictional across an arm/disarm — in the recording *and* in the
replay. Measure it over a segment that stays armed throughout.

**Arm edges *are* modelled.** `MotionProvider::onFlightLoopTick` calls
`washout_->reset(); effects_->reset();` on the rising edge out of `Disarmed`, and `runChain`
mirrors it from the recorded `arm_state` column. The reset is applied one sample *after* the
row where `arm_state` first leaves 0, because within a plugin tick `washout_->update()` runs at
the top, the reset ~40 lines later, and `row.armState` is written after `armRamp_.update()` —
so the edge row itself is still a pre-reset row. A file with no `arm_state` column (synthetic
streams, cue-only exports) simply carries no edges, which is a valid input, not an error.

**Effects-layer state is modelled too, but only if the recording carries it (Change 2).**
`EffectsLayer` carries four members between ticks — `prevOnGround_`, `tdActive_`, `tdT_` and
`rumblePhase_` — and unlike the washout's filter state, `rumblePhase_` is a free-running 12 Hz
oscillator with no decay of its own. A warm-up window washes out an unrecorded *filter* initial
condition because it decays; it cannot wash out an unrecorded *phase*, because there is nothing
in the system pulling two independent phases together. An unrecorded `rumblePhase_` mismatch is
therefore not a transient — it is a standing divergence of up to `2 × rumble_gain` (`1.8 mm` at
the shipped `rumble_gain = 0.9 mm`) for as long as the platform stays on the ground, which is
exactly the `ground_takeoff` / `approach_landing` / `acceptance` failures above.

The fix is to record this state and seed a replay from it: `Telemetry` carries four more columns
(`eff_prev_onground`, `eff_td_active`, `eff_td_t`, `eff_rumble_phase`, in that order, appended
after `arm_state` — every existing column keeps its position, so the `cut` invocation in §2 is
unaffected), filled from `EffectsLayer::state()` captured **before** each tick's `update()` call
(so seeding a replay from row 0's values and then processing row 0 reproduces the recording from
the first row, with no off-by-one). `washout_replay`'s `loadCues` reads all four as a group — if
even one is missing, seeding is skipped entirely, not partially — and `runChain` seeds
`EffectsLayer` from the first sample only when all four are present.

**This is a forward-looking fix, not a retroactive one.** Every one of this campaign's seven real
recordings predates these columns, so `haveEffState` is false for all of them and they seed from
zero exactly as they did before Change 2 — their `ground_takeoff` / `approach_landing` /
`acceptance` failures are unchanged and are **expected to stay FAILing** until they are re-recorded
with a build that writes the new columns. See `docs/motion-tuning/baseline-metrics.md` for what is
and isn't decided by the current baseline as a result.

**`--verify` refuses to run in five situations**, each returning a distinct error rather than
a misleading pass. The first three are decidable from the arguments alone and are refused
*before* the chain runs, so nothing reaches stdout first:

- **combined with `--set`** — an overridden config can't reproduce a recording made with a
  different config, so the comparison would be meaningless by construction.
- **combined with `--resample-dt`** — resampling changes `dt` itself, which changes every
  filter output; there is nothing left to verify against.
- **combined with `--sweep`** — a sweep runs multiple configs; there is no single replay to
  compare.
- **when the input carries no recorded `live_*` columns at all** — true for every `--synth`
  stream, and for any cues-only export. Without a `live_*` column to compare against, a naive
  implementation would report a false PASS (zero accumulated error over nothing); this harness
  refuses instead.
- **when fewer than 100 samples remain after the warm-up skip** — a recording shorter than its
  own warm-up cannot be verified; saying so is better than passing on a handful of rows.

Nothing produced by this harness — no sweep result, no metric, no rig recommendation — should
be trusted until it has passed `--verify` against a real recording of the segment in question.

**If `--verify` fails, stop before doing anything else.** In order of likelihood:

- the recording was made with a different `configuration.toml` than the one passed to
  `--verify`;
- the recording spans a **config reload**, which resets the filters mid-file and leaves no
  trace in the CSV for replay to key off;
- the recording spans an **arm edge that replay could not see** — either the `arm_state` column
  was stripped (a cue-only export), or the arm happened in **manual mode**, where the plugin
  skips the reset entirely and the filters never ran at all.

Note that plain *disarmed* ticks are **not** a cause: `WashoutFilter::update` runs
unconditionally every unpaused tick regardless of arm state, so `live_*` is valid throughout a
disarmed stretch. It is the disarmed→armed *transition* that resets, and that is modelled.
Manual-mode ticks are a genuine cause, for the different reason given in §1: there the flight
loop never calls `WashoutFilter::update`/`EffectsLayer::update` at all.

## 6. Synthetic cue generation

```bash
./tools/build/washout_replay --synth chirp:0.02-5.0:0.3:300 --config configuration.toml --out /tmp/chirp.csv
```

`--synth SPEC` synthesises a cue stream with no flight at all, useful for characterising the
filter's frequency response directly (Stage 1 of the campaign). Forms:

- `step:<g>:<durSec>` — a constant `g_nrml` offset of `<g>` starting 1 s in.
- `sine:<hz>:<g>:<durSec>` — a sinusoidal `g_nrml` at `<hz>` with amplitude `<g>`.
- `chirp:<f0>-<f1>:<g>:<durSec>` — a logarithmic sweep from `<f0>` to `<f1>` Hz, amplitude `<g>`.

`--synth-dt SEC` sets the timestep (default `1/60`). A `--synth` run carries no recorded
`live_*` columns, so it can never be the input to `--verify` — see above.

## 7. Reading the metrics

**`washout_metrics.py` measures replay output, never a cue file.** The reference segments under
`reference/` are cue-only exports (§2's `cut`) and carry none of the columns the metrics need —
`live_heave`, `heave_clamped`, `sent*`, `rot_*_clamped`, `sl_*_clip`, `heave_pos_raw`,
`reach_scale`. Pointing the script at one aborts naming the first missing column. Always the
same two steps:

```bash
cd MotionProviderPlugin
# The committed reference files are gzipped and washout_replay reads plain CSV
# only, so expand the cue file first.
gunzip -c reference/cruise_calm.csv.gz > /tmp/cruise_calm.cues.csv
./tools/build/washout_replay --cues /tmp/cruise_calm.cues.csv --config configuration.toml \
    --out /tmp/cruise_calm.csv
tools/.venv/bin/python tools/washout_metrics.py /tmp/cruise_calm.csv
```

`washout_metrics.py` itself *is* gzip-transparent (any argument ending in `.csv.gz` is opened
through `gzip`), which is what makes archived *replay* output convenient to re-measure. That
transparency is not permission to feed it a cue file — a `.csv.gz` used to die on the gzip
magic byte with a `UnicodeDecodeError`, which at least made the mistake loud; now it gets as
far as the honest "missing required column 'live_heave'".

Add `--csv` to get machine-readable output instead of the aligned table. Every file argument
is measured independently and printed as its own row.

The table leads with `rows`, `sec` and `fs` and ends with `peak_raw_mm` and `peak_out_mm`. Read
those first: the refusal warnings below go to **stderr** while the table goes to stdout, so an
operator who redirects the table loses them — a `peak_out_mm` that is tiny, or exactly the park
offset, is what makes a dead or never-armed run obvious in the table itself.

| Metric | What it is | Gate / how to read it |
|---|---|---|
| `sat_heave` | % of unpaused ticks with the heave clamp engaged | **The primary diagnostic.** Target for the diagnosis stage: under 2% in calm cruise. Today's expectation there is 80–100%. |
| `sat_rot`, `sat_tilt_rate`, `sat_sl_vel`, `sat_sl_acc` | same idea for the rotational clamp, the tilt-rate limiter, and the `SafetyLimiter`'s velocity/acceleration clips | Secondary diagnostics — same "must fall" gate as `sat_heave` unless a stage says otherwise. |
| `sat_envelope` (from `reach_scale`) | % of unpaused ticks where the **pre-blend** envelope bisection engaged (`reach_scale < 1.0`) | See "Why `sat_envelope` only sees one of two clamps" below — it is blind to the second, post-blend clamp by design, not by omission. |
| `wrms` | RMS of heave acceleration, band-limited to 0.1–0.63 Hz with a Hann window (RMS-corrected for the window's power loss) | A documented band emphasis, **not a conformant ISO-2631 Wk weighting**. Ranks candidates against each other only — never quote it as a comfort figure. Returns `nan` (stderr warning) when `live_heave` has no variation at all — see below. |
| `band_ratio` | fraction of heave-acceleration spectral power inside that same 0.1–0.63 Hz band | States directly whether motion sits in the motion-sickness band. Stage 8's inverted gate depends on this: `wrms` may rise as amplitude comes back, but `band_ratio` must not. Same `nan` refusal as `wrms`. |
| `jerk_p95` | the **max over the six** streamed BFF demand channels of that channel's 95th-percentile \|third difference\| — a per-channel p95, then the worst channel; not a p95 pooled across channels. Normalised to counts/s³ via the file's own sampling rate | Comparative only — actuator counts, no counts-to-mm conversion exists. Runs at different framerates are still comparable because of the normalisation. **Only meaningful on replay output** (a disarmed live recording pins `sent*` and reports ≈ 0), and fictional across an arm/disarm — see §5. |
| `rot_rate_p95`, `rot_rate_pct_3dps` | how fast the **commanded** platform pose (`live_roll`/`live_pitch`/`live_yaw`) rotates, differentiated against the real `dt_real` time axis. `rot_rate_p95` is the **max over the three axes of each axis's own p95** (°/s) — "max of the p95s", not "p95 of the max", same combination rule as `jerk_p95`. `rot_rate_pct_3dps` is the % of ticks where **any** axis exceeds 3°/s | The total rate a pilot's vestibular system feels, regardless of which internal channel produced it — use this to judge whether the platform *as a whole* is rotating more than it should. The 3°/s threshold is a documented working rule of thumb for ranking candidates, **not a perceptual constant** — published vestibular detection thresholds range roughly 0.5–3°/s depending on axis, waveform and workload. Requires `live_roll`/`live_pitch`/`live_yaw`; not paused-masked, same reasoning as `wrms`/`jerk_p95`/`lag_ms`. Returns `nan` (stderr warning) below 100 rows. **It cannot resolve a tilt-channel-only change**, and that is by construction, not a bug: a tilt-rate-limit sweep (`tr_3.csv` vs `tr_5.csv`) that clearly tightens the isolated tilt channel does not show a corresponding drop here (4.65% → 4.88%, the wrong direction) because the rotational-cueing and effects channels dominate the combined pose. That is exactly why `tilt_rate_p95`/`tilt_rate_pct_3dps` exist below. |
| `tilt_rate_p95`, `tilt_rate_pct_3dps` | the same computation, restricted to the tilt-coordination channel alone (`tilt_pitch`/`tilt_roll`, max over the two axes) | Tilt coordination is **meant** to be felt as sustained acceleration — a re-orientation of gravity, not a turn — so any supra-threshold angular rate on this channel is a leak of the wrong percept, an artefact, unlike the rotational channel where fast rotation is the intent. This is the pair that actually isolates "does tilt coordination itself rotate too fast": on the same `tr_3.csv`/`tr_5.csv` sweep it drops sharply as expected (0.93% → 0.40%), and on the tilt-*limit* fixtures (`ti_tilt5_5`/`5_7`, `ti_tilt7_5`/`7_7`) it rises with the larger limit (0.94%→1.02%, 0.74%→0.93%) — both directions `rot_rate_pct_3dps` alone missed or understated. Same required-column, no-paused-mask and too-short-refusal rules as `rot_rate_*`. |
| `lag_ms` | shift maximising the Pearson-normalised cross-correlation between the drive cue (`g_nrml`) and **`live_heave`**, band-limited to 0.3–1 Hz | **Not a latency.** The washout is a high-pass, not a delay line; only the *delta* between a baseline and a candidate over the *same* segment means anything. Gate: candidate ≤ baseline + 15 ms. It correlates the **live** pose, not the commanded one: `live_*` is the column `--verify` proves replayable, while `cmd_*` carries the arm blend replay deliberately does not model. Returns `nan` (with a stderr warning) below 8 periods at the 0.3 Hz band floor, or when either signal has no energy left in the band (e.g. heave frozen against a clamp) — a stuck signal must never read as "zero added lag". |
| `peak_raw_mm` | `max \|heave_pos_raw\|` | **Structurally pinned near the limit — do not read it as "the overdrive".** `heavePos_` is a clamped *state*, so this can exceed `heave_limit_mm` by at most one integration step (~42 mm at the shipped settings) no matter what the time constants do. To measure the filter rather than the clamp, add `--set washout.heave_limit_mm=1e9`. See the fuller note in §4. |
| `peak_out_mm` | `max \|live_heave\|` — post-clamp, post-smoothing, and **including the effects layer** | Not gated. Its real job is as a liveness check: a `peak_out_mm` that is tiny or flat is how a dead run shows up in the table. |

**Why `lag_ms` uses such a narrow band, and why the number moves when you least expect it.**
An earlier, broadband version of this estimator was frequency-dependent: on a synthetic tone
with a fixed injected delay, changing the tone's frequency from 0.3 Hz to 0.5 Hz moved the
*reported* lag by about 17 ms with no change in the true delay. Since this campaign's whole
job is retuning the washout's corner frequency, that artefact would land squarely on top of
the 15 ms gate and make it meaningless. Restricting the correlation to 0.3–1 Hz and requiring
8 periods at the low end is what makes the number trustworthy enough to gate on.

**The shipped settings' `lag_ms` is ~730 ms, and that is real, not a spurious peak.**
Measured on the campaign's hand-flown recording (replayed, then `washout_metrics.py`):
`lag_ms = 730.0`, uncomfortably close to the estimator's 1000 ms search ceiling — which looks
like exactly the kind of artefact this harness exists to catch. It is not one; it was tested,
not assumed:

*Analytic.* The heave chain (`WashoutFilter::update`) is, ignoring gain and the tiny
`smooth_tau` output low-pass, `G(s) = s / ((s+a)(s+b1)(s+b2))` with `a = 1/heave_hp_tau`,
`b1 = 1/heave_vel_washout_tau`, `b2 = 1/heave_pos_washout_tau` — a differentiator (the
high-pass) followed by two leaky integrators (the vel/pos washouts). Phase delay is
`-angle(G(j2πf)) / (2πf)`. At the shipped settings (`heave_hp_tau=1`,
`heave_vel/pos_washout_tau=2` each) versus a candidate that shortens the washout pair to `0.3`:

| f (Hz) | shipped: phase | shipped: delay | candidate (τ=0.3): phase | candidate: delay |
|---|---|---|---|---|
| 0.3 | −122.3° | 1132.8 ms | −31.0° | 287.3 ms |
| 0.4 | −135.8° |  943.1 ms | −52.3° | 363.4 ms |
| 0.5 | −144.3° |  801.4 ms | −69.0° | 383.1 ms |
| 0.6 | −150.0° |  694.6 ms | −82.2° | 380.5 ms |
| 0.7 | −154.2° |  612.0 ms | −92.9° | 368.6 ms |
| 0.8 | −157.4° |  546.5 ms | −101.6° | 352.9 ms |
| 0.9 | −159.9° |  493.4 ms | −108.9° | 336.2 ms |
| 1.0 | −161.9° |  449.6 ms | −115.1° | 319.6 ms |

(At 0.5 Hz this matches the campaign's own back-of-envelope estimate, ≈800 ms, almost exactly.)
Note the shipped settings' delay at the *low* end of the band (1132.8 ms at 0.3 Hz) already
**exceeds** `xcorr_lag_ms`'s 1000 ms search ceiling — a real, documented limitation, not a bug:
see below.

*Empirical.* `--synth sine:<hz>:0.03:40` at each frequency in the table above, replayed, then
measured with `washout_metrics.py`, for both settings:

| f (Hz) | shipped: measured `lag_ms` | candidate: measured `lag_ms` |
|---|---|---|
| 0.3 | 1000.0 (clamped — see below) | 266.7 |
| 0.4 |  950.0 | 366.7 |
| 0.5 |  800.0 | 383.3 |
| 0.6 |  700.0 | 383.3 |
| 0.7 |  616.7 | 366.7 |
| 0.8 |  550.0 | 350.0 |
| 0.9 |  500.0 | 333.3 |
| 1.0 |  450.0 | 316.7 |

Every measured value tracks its analytic prediction to within one or two lag-resolution steps
(`1000/fs` ≈ 16.7 ms at the 60 Hz synth rate) — the one exception being 0.3 Hz on the shipped
settings, where the true delay (1132.8 ms) is outside what a 1 s search can find at all, so the
estimator reports the ceiling instead of a wrong answer in the wrong direction. That the number
moves smoothly and correctly with frequency, matching a closed-form prediction derived from the
filter's own poles, is what an estimator picking a genuine feature of the signal looks like — not
what a spurious argmax on a flat correlation looks like.

*Peak shape.* On the real hand-flown recording's `g_nrml`/`live_heave` correlation (band-limited
to 0.3–1 Hz), the curve is a single smooth hump: −0.20 at lag 0, rising monotonically to a clear
maximum of 0.334 at 730 ms, then falling monotonically back to 0.26 by 955 ms. That is a
well-defined argmax, not a flat plateau where the location would be noise-sensitive.

**Verdict: the hypothesis holds.** `lag_ms ≈ 730 ms` on the shipped settings is a real property of
the current heave washout's phase response in the band the pilot feels, not an estimator defect.
This reframes the campaign's "no added lag" constraint (`lag_ms` ≤ baseline + 15 ms, above): the
*baseline itself* already carries roughly 0.7–1.1 s of phase delay across 0.3–1 Hz, worst at the
low end. Shortening `heave_vel/pos_washout_tau` toward the candidate value cuts that delay by
roughly half to two-thirds across the same band (e.g. 801 ms → 383 ms at 0.5 Hz) — the intended
fix for saturation *also* improves the thing the "no added lag" gate was written to protect,
rather than trading against it.

**The 1000 ms search ceiling is a real, separate limitation, worth knowing about going forward.**
`xcorr_lag_ms`'s `max_lag_sec` defaults to `1.0`; any true delay beyond that cannot be found and
the estimator reports (at most) the ceiling instead. The shipped settings are already at the edge
of this at the low end of the band (0.3 Hz); a *slower* candidate than shipped (longer washout
time constants) could exceed it more broadly and have its `lag_ms` silently underreported as
"no worse than baseline" when it is actually worse. This campaign only shortens the washout, so it
never approaches the ceiling from the wrong side — but a future change that lengthens
`heave_vel/pos_washout_tau` past roughly 2 s should widen `max_lag_sec` first and re-check.

**Why `sat_envelope` only sees one of two clamps.** `MotionProvider::blendedCommand` calls
`StewartKinematics::clampToReachable` **twice**. The first call runs on `rawLive` — the
washout+effects demand before any arm-blend attenuation — and *that* call's scale is what
gets recorded as `reach_scale`/`sat_envelope`: it answers whether the cueing chain demanded
more than the platform can physically reach, which is what the tuning campaign cares about.
The second call guards the *blended* pose (a mix of the already-reachable park pose and the
already-clamped `live` pose from the first call). That mix is reachable in practice, so
`clampToReachable` short-circuits to scale 1.0 whenever its input is already reachable —
recording *that* scale would pin `sat_envelope` at 0% forever, regardless of what the platform
actually did. The second clamp stays in the code as a guard; its scale is deliberately never
written to telemetry. Don't read a low `sat_envelope` as "the envelope is never a constraint
anywhere in the chain" — it can only ever tell you about the first clamp.

**Missing columns are treated as a safety issue, not a formatting one.** Only `dt_real` and
`g_nrml` may fall back to a default when absent from a CSV. Every other column — most
importantly `heave_clamped` — aborts the script with a clear error naming the file and the
column if it's missing. A silently zero-filled `heave_clamped` would report `sat_heave = 0%`,
which happens to be exactly this campaign's target outcome and is therefore the single most
dangerous number this script could ever report by accident.

The bar for an exemption is that a missing column must not be able to manufacture a
*favourable* value on its own: a missing `dt_real` only assumes 60 Hz, and a missing `g_nrml`
leaves the lag correlation with a dead drive channel, which `lag_ms` already refuses with an
honest `nan`. `reach_scale` used to be exempt and no longer is — defaulting it to 1.0 printed
`sat_envelope = 0.00 %` with no warning at all, and the campaign *skips* its envelope stage
unless `reach_scale < 1` is common, so a silently defaulted column silently skipped a stage.
(The exemption was never needed either: `heave_clamped`, `paused` and `reach_scale` all arrived
in the same commit series, so no recorder emits one without the others.)

**A frozen `live_heave` is refused, not scored.** Pinning `live_heave` to a constant — a heave
locked against a clamp, or a pose that never moved — yields `wrms = 1.24e-20` and
`band_ratio = 7.6e-08`: the *best values either metric can produce*. Stage 8's inverted gate is
precisely "`wrms` may rise as long as `band_ratio` does not", so a dead run would read as an
ideal candidate. `washout_metrics.py` therefore checks the standard deviation of `live_heave`
once and returns `nan` for both metrics, with a stderr warning, when there is no variation to
measure — the same refusal `lag_ms` already applies to the same condition. `peak_out_mm` in the
table is the stdout-visible half of that guard.

## 8. Promoting a candidate to the rig

Edit the relevant value(s) directly in `configuration.toml` on the Sim-PC, then press
**"Reload config"** in the plugin's Status window. No rebuild, no X-Plane restart. Reloading
resets the stateful washout and effects filters, so a reload mid-flight starts from a clean
pose rather than jumping — expect a brief transient right after reloading, not a bug.

Because reload is this cheap, several candidates can be flown in one rig session: edit, reload,
fly, record, repeat. The campaign plan budgets four rig sessions total (after Stage 3, bundled
after Stages 5+6, after Stage 8, plus acceptance) rather than one per stage.

After a candidate is adopted: set it permanently in `configuration.toml`, append a row to
`docs/motion-tuning/tuning-log.md`, and commit both together.

## 9. Where things live

- `docs/motion-tuning/tuning-log.md` — the living, append-only log of every change tried.
- `docs/motion-tuning/baseline-metrics.md` — the frozen comparison anchor, written once by
  Stage 2 and never edited afterward.
- `MotionProviderPlugin/reference/*.csv.gz` — the recorded segments (cue columns only,
  gzipped), committed so anyone can re-run the whole analysis without a rig or a flight.
- `docs/superpowers/specs/2026-08-29-motion-heave-tuning-design.md` — the diagnosis: why the
  heave channel saturates, why `heave_gain` and the `SafetyLimiter` caps are the wrong levers,
  and the full metric/gate rationale.
- `docs/superpowers/plans/2026-08-29-motion-heave-tuning.md` — the stage-by-stage task list
  this manual's commands are drawn from.

## The two rules, once more

- **`--verify` must pass before any number is trusted.**
- **`lag_ms` must stay within baseline + 15 ms.**
