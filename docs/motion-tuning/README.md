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

- Load the aircraft the campaign's baseline is built on: **Piper Arrow III (vFlightAir
  PA28-201R)**. Every number this campaign produces is valid for this aircraft only — a
  different flight model needs its own baseline before its numbers mean anything.
- Start recording with the **Record** button in the plugin's Status window, or set
  `enabled = true` under `[telemetry]` in `configuration.toml` to auto-start as soon as the
  plugin loads (useful for the Sim-PC, where nobody may be at the keyboard to click Record).
  `dir` in that section picks the output directory; leave it empty to write next to
  `configuration.toml`.
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
cut -d, -f2,4-16 motion-20260901-140322.csv | gzip > MotionProviderPlugin/reference/cruise_calm.csv.gz
```

Because replay is a deterministic function of `(cues, dt, config)`, full telemetry can always
be reconstructed from a cue-only file by replaying it — so the ~70 MB full recordings never
enter git, only the ~2 MB gzipped cue files under `MotionProviderPlugin/reference/`. If
`Telemetry::header()` ever changes column order, regenerate this `cut` invocation from the
new header before trusting it — it is a position-based cut, not a name-based one.

## 3. Running a single replay

```bash
cd MotionProviderPlugin
./tools/build/washout_replay --cues reference/cruise_calm.csv --config configuration.toml \
    --set washout.heave_pos_washout_tau=0.5 --out /tmp/run.csv
```

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
verify against.

Stage 3 of the campaign moves `heave_vel_washout_tau` and `heave_pos_washout_tau` together
(the two poles of one washout) rather than sweeping them independently — run the sweep above
for one, then repeat with the other pinned to the same value via `--set` on each invocation.

## 5. `--verify` — the harness self-test

```bash
cd MotionProviderPlugin
./tools/build/washout_replay --cues motion-20260901-140322.csv --config configuration.toml --verify
```

Compares the replay's `live_heave/roll/pitch/yaw` against the recording's, tick for tick, and
reports `verify: PASS (bit-exact)` only on an exact match. It checks `live_*` and not `cmd_*`
because `live_*` alone is a pure function of `(cues, dt, config)`; `cmd_*` also depends on the
arm ramp blend, which replay deliberately does not model (replay always runs "armed and live").

**`--verify` refuses to run in four situations**, each returning a distinct error rather than
a misleading pass:

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

Nothing produced by this harness — no sweep result, no metric, no rig recommendation — should
be trusted until it has passed `--verify` against a real recording of the segment in question.

**If `--verify` fails, stop before doing anything else.** In order of likelihood: the
recording was made with a different `configuration.toml` than the one passed to `--verify`;
the recording spans a config reload (which resets the filters mid-file); the recording
includes disarmed/manual-mode ticks whose `live_*` never came from the washout in the first
place.

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

```bash
cd MotionProviderPlugin
tools/.venv/bin/python tools/washout_metrics.py /tmp/run.csv reference/cruise_calm.csv.gz
```

Add `--csv` to get machine-readable output instead of the aligned table. Every file argument
is measured independently and printed as its own row.

| Metric | What it is | Gate / how to read it |
|---|---|---|
| `sat_heave` | % of unpaused ticks with the heave clamp engaged | **The primary diagnostic.** Target for the diagnosis stage: under 2% in calm cruise. Today's expectation there is 80–100%. |
| `sat_rot`, `sat_tilt_rate`, `sat_envelope`, `sat_sl_vel`, `sat_sl_acc` | same idea for the rotational clamp, the tilt-rate limiter, the pose-wide envelope bisection, and the `SafetyLimiter`'s velocity/acceleration clips | Secondary diagnostics — same "must fall" gate as `sat_heave` unless a stage says otherwise. |
| `wrms` | RMS of heave acceleration, band-limited to 0.1–0.63 Hz with a Hann window (RMS-corrected for the window's power loss) | A documented band emphasis, **not a conformant ISO-2631 Wk weighting**. Ranks candidates against each other only — never quote it as a comfort figure. |
| `band_ratio` | fraction of heave-acceleration spectral power inside that same 0.1–0.63 Hz band | States directly whether motion sits in the motion-sickness band. Stage 8's inverted gate depends on this: `wrms` may rise as amplitude comes back, but `band_ratio` must not. |
| `jerk_p95` | 95th-percentile \|third difference\| of the six streamed BFF demand channels, normalised to counts/s³ via the file's own sampling rate | Comparative only — actuator counts, no counts-to-mm conversion exists. Runs at different framerates are still comparable because of the normalisation. |
| `lag_ms` | shift maximising the Pearson-normalised cross-correlation between the drive cue and the commanded pose, band-limited to 0.3–1 Hz | **Not a latency.** The washout is a high-pass, not a delay line; only the *delta* between a baseline and a candidate over the *same* segment means anything. Gate: candidate ≤ baseline + 15 ms. Returns `nan` (with a stderr warning) below 8 periods at the 0.3 Hz band floor, or when either signal has no energy left in the band (e.g. heave frozen against a clamp) — a stuck signal must never read as "zero added lag". |
| `peak_raw_mm`, `peak_out_mm` | peak pre-clamp and peak post-clamp heave excursion | Context for the saturation numbers, not gated directly. |

**Why `lag_ms` uses such a narrow band, and why the number moves when you least expect it.**
An earlier, broadband version of this estimator was frequency-dependent: on a synthetic tone
with a fixed injected delay, changing the tone's frequency from 0.3 Hz to 0.5 Hz moved the
*reported* lag by about 17 ms with no change in the true delay. Since this campaign's whole
job is retuning the washout's corner frequency, that artefact would land squarely on top of
the 15 ms gate and make it meaningless. Restricting the correlation to 0.3–1 Hz and requiring
8 periods at the low end is what makes the number trustworthy enough to gate on.

**Missing columns are treated as a safety issue, not a formatting one.** Only `dt_real`,
`g_nrml` and `reach_scale` may fall back to a default when absent from a CSV. Every other
required column — most importantly `heave_clamped` — aborts the script with a clear error
naming the file and the column if it's missing. A silently zero-filled `heave_clamped` would
report `sat_heave = 0%`, which happens to be exactly this campaign's target outcome and is
therefore the single most dangerous number this script could ever report by accident.

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
