# Baseline Metrics

**This file is filled by Stage 2 of the campaign** (see
`docs/superpowers/plans/2026-08-29-motion-heave-tuning.md`) **and must not be edited after
that.** Every later stage's gates — the `lag_ms` ≤ baseline + 15 ms budget, and the
"`sat_heave`/`wrms`/`jerk_p95` must fall" comparisons — are measured against exactly the
numbers recorded here. If a number here later turns out to have been measured wrong, add a
dated correction section at the bottom rather than editing the frozen table; the table itself
is the anchor every other document points at, and it needs to stay whatever it was actually
compared against.

Until Stage 2 runs, this file is a placeholder — the section headers below name what goes in
each one.

## Recording environment

Fill in at the moment the reference set is captured; all seven files (six segments plus the
acceptance flight) should share one environment.

- **Aircraft:** Piper Arrow III (vFlyteAir custom PA28-201R add-on) — not a stock X-Plane
  model. Every number in this file, and everything measured against it, is valid for this
  aircraft only; a different type needs its own baseline before its numbers mean anything.
- **X-Plane version:** 12.4.3
- **Weather preset:** per segment, not one setting for the whole set:
  - `cruise_calm` — clear, wind speed 2 kt, everything else 0
  - `turbulence` — wind 4 kt, gusts 7 kt, shear 4 kt, turbulence **severe**
  - `acceptance` — **real weather** (downloaded). This is the one file that cannot be
    re-flown to the same conditions. As a recorded cue stream it stays fully usable; only a
    rig session trying to reproduce the *flight* cannot match it.
  - `steep_turns`, `climb_descent`, `ground_takeoff`, `approach_landing` — recorded as "calm";
    the exact wind/turbulence values were not noted at capture time and cannot be recovered.
    Treat these four as reproducible only approximately: their cue files remain exact, but a rig
    session cannot recreate the flight condition they were flown in.
- **Machine:** sim-PC — AMD Ryzen 7 5700X, 8 cores / 16 threads, 3.4 GHz, GeForce RTX 4060 Ti,
  Windows 11. All seven files are CRLF and carry `arm_state = 0` throughout: the rig was switched
  off, so the plugin never armed.
- **Framerate** (median `1/dt_real`): **45.2 fps** across the set; per segment 30.7 / 45.2 / 47.4 /
  47.9 / 43.8 / 43.9 / 50.9. `cruise_calm` at 30.7 is the outlier — worth knowing before comparing
  its `jerk_p95` against another segment's.
- **`configuration.toml` git revision:** **`f516263`** ("stage-4 tuning baseline after clean
  transport"). Confirmed two ways: the file the recordings were flown with is byte-identical to
  the committed one, and `--verify` passes bit-exactly on the airborne segments — a 0.1 % change
  to `heave_gain` produces 455× the noise-floor tolerance, so any washout difference would have
  failed the gate.

## Reference segments

Recorded on the Mac's X-Plane 12 with the rig disconnected. One file per row, cue-only and
gzipped into `MotionProviderPlugin/reference/`:

| Segment | Duration target | Notes |
|---|---|---|
| `cruise_calm` | 60–90 s | The segment that matters most — by definition nothing may pump here. |
| `steep_turns` | 60–90 s | |
| `climb_descent` | 60–90 s | Alternating climb/descent. |
| `turbulence` | 60–90 s | Defined turbulence setting, not ambient chop. |
| `ground_takeoff` | 60–90 s | Ground roll through takeoff. |
| `approach_landing` | 60–90 s | |
| `acceptance` | 8–10 min | Continuous mixed flight; catches interactions and slow state buildup the short segments miss. Also the file Stage 10 replays against the final config. |

**Recording with the rig disconnected leaves the plugin disarmed, and that is why every number
in this file comes from a *replay*, never from the live recording.** The cue columns are
identical either way — that is what makes a cue-only reference file legitimate — but the
output columns are not. `armIntent` needs a fresh gateway heartbeat, so with no gateway the
plugin never leaves `Disarmed`, `armRamp_.blend()` stays 0, `blendedCommand` returns the park
pose on every tick, and `sp*`/`sent*` are therefore constant for the whole file. Read straight
off such a recording, `jerk_p95` is ≈ 0 and `sat_sl_vel` = `sat_sl_acc` = 0 %.

That is not a good result, it is *no* result — and `jerk_p95` is one of the three gated "must
decrease" metrics, so a zero baseline would make its gate impossible for any armed candidate
to satisfy and the gate would quietly get dropped.

**`jerk_p95`, `sat_sl_vel` and `sat_sl_acc` are meaningful only when the baseline and the
candidate are both replay outputs. Never read them off a disarmed live recording.** Replay
always runs armed and live (`arm_state` is written as `Armed` and no park blend is applied),
so the two-step below produces them honestly. The washout-derived metrics (`sat_heave`,
`wrms`, `band_ratio`, `lag_ms`) are unaffected by the arm state — `live_*` is computed every
tick regardless — but they come from the same replay anyway, so the whole table is one
consistent measurement.

## `--verify` status of the reference set — a limitation this baseline cannot outrun

All seven files in this baseline **predate `Telemetry`'s effects-state columns**
(`eff_prev_onground`, `eff_td_active`, `eff_td_t`, `eff_rumble_phase`, added alongside the
final-half `--verify` discrimination — see `docs/motion-tuning/README.md`'s `--verify` section).
That fix lets a replay seed `EffectsLayer`'s free-running rumble oscillator from a recording
instead of always starting it at zero, but only when the recording actually carries the new
columns. It does not, and cannot, retroactively add those columns to a file that was written
before they existed.

**Practical consequence: `ground_takeoff`, `approach_landing` and `acceptance` cannot be
bit-verified against this baseline, and re-recording them would not be the fix.** All three
contain ground segments — `ground_takeoff` by design (it *is* the ground roll), the other two in
passing (a landing, and a mixed flight touching down/taking off) — and on all three, `--verify`'s
residual is confined entirely to on-ground ticks, tops out at `1.8 mm` (the theoretical max at the
shipped `rumble_gain = 0.9 mm`; measured `1.744 mm` on `ground_takeoff`), and is entirely the
effects-layer's contribution: on `ground_takeoff`, `max |live_heave| diff` in the air is `0.0000 mm`
across 1017 ticks against `1.7442 mm` on the ground across 408 ticks, and `max |eff_heave| diff` is
the identical `1.7442 mm` — the washout itself is bit-exact throughout, in the air *and* on the
ground; only the unrecorded rumble phase diverges. This is a property of *when these seven files
were recorded*, not of the aircraft or the config, so flying the same segments again on the current
build would not change it — the fix only helps recordings made **after** it landed. **Do not
re-fly these three** on the strength of this note; a future baseline refresh, if one happens, is
the right time to pick up bit-verified ground segments, not an ad hoc re-recording now.

**What this baseline actually decides on.** The four segments that verify cleanly —
`cruise_calm`, `steep_turns`, `climb_descent`, `turbulence`, all airborne throughout — are the ones
this campaign's stage gates (`sat_heave`, `wrms`, `band_ratio`, `lag_ms`, the `jerk_p95` family) are
measured against, and all four are unaffected by this limitation: nothing in it touches airborne
ticks. `ground_takeoff` and `approach_landing` still contribute their `sat_*`/`peak_*` figures (those
come straight from the replay's own internal state, not from a comparison against a recording), the
same way they did before this note — only the *verification* of `live_heave` against the recording
on the ground is unavailable, not the metrics themselves.

## Stage 1 — sanity check against the analysis

Before recording anything, Stage 1 runs a synthetic chirp and a sequence of sine-amplitude
sweeps through the washout to test the diagnosis in
`docs/superpowers/specs/2026-08-29-motion-heave-tuning-design.md` before any tuning stage
leans on it.

- **Predicted:** `|G|max ≈ 0.91 s²` near 0.067 Hz; clamp onset near `|Δg| ≈ 0.022 g`.
- **How it was actually settled:** the synthetic chirp/sine sequence was not run as a separate
  stage. The real recordings answered the same question more directly and more strongly, so the
  verdict below rests on flight data rather than on a bench signal. The synthetic path remains
  available (`--synth chirp:0.02-5.0:0.3:300`) and was exercised while validating `lag_ms`.
- **Measured, from the recordings:**
  - A Mac recording on autopilot in dead-still air reached `max |g_nrml − 1| = 0.0005 g`, i.e.
    44× *below* the predicted 0.022 g threshold, and saturated **0.00 %** of ticks — the model's
    prediction for that input, exactly.
  - `cruise_calm` on the sim-PC (clear, 2 kt wind) reached `mean |Δg| = 0.043 g` and saturated
    **19.92 %** of ticks. Two knots of air is enough to cross the threshold.
  - Hand-flown with wind: `max |Δg| = 0.20 g`, saturation 40.66 %, `live_heave` pinned at exactly
    ±30.00 mm, `heave_pos_raw` peaking at 36.41 mm — a clamped state can exceed the limit by at
    most one integration step, and it does.
  - Median half-cycle of the heave reversals: **3.69 s** (p25 1.70, p75 6.75), against the
    reported symptom of "2–4 s up, then 2–4 s down". The spec predicted ≈ τ_vel = 2 s; the real
    limit cycle is ~1.8× that, because it is driven by the actual g spectrum and not by τ alone.
- **Verdict: the diagnosis holds.** Heave saturates in every segment (19.92 % to 88.27 %), and it
  is the *only* channel that does so at the shipped settings — `sat_envelope` is 0.00 % everywhere
  and `sat_rot` is 0.00 % in level flight. The campaign proceeds.
- **Second finding, not in the original diagnosis:** the rotational channels *do* saturate in
  manoeuvring — 20.02 % in `steep_turns`, 15.44 % in `climb_descent`, 6.64 % in `turbulence`. The
  plan's Stage 9 anticipated this case and requires it to get its own stage rather than a quick
  fix appended here.

## Metrics table

**Two steps, in this order.** `washout_metrics.py` measures *replay output*; the reference
files are cue-only exports and carry none of the columns it needs (`live_heave`,
`heave_clamped`, `sent*`, `rot_*_clamped`, `sl_*_clip`, `heave_pos_raw`, `reach_scale`).
Pointing the script at `reference/*.csv.gz` directly is not a shortcut — it aborts naming the
first missing column. Replay first, then measure the replay:

```bash
cd MotionProviderPlugin
mkdir -p /tmp/cues /tmp/replay
for seg in cruise_calm steep_turns climb_descent turbulence \
           ground_takeoff approach_landing acceptance; do
    gunzip -c "reference/$seg.csv.gz" > "/tmp/cues/$seg.csv"
    ./tools/build/washout_replay --cues "/tmp/cues/$seg.csv" \
        --config configuration.toml --out "/tmp/replay/$seg.csv"
done
tools/.venv/bin/python tools/washout_metrics.py /tmp/replay/*.csv
```

(The cue-only intermediates and the replay output must land in separate directories: both used to
be named `/tmp/$seg.cues.csv` / `/tmp/$seg.csv`, and a glob of `/tmp/*.csv` over that layout also
matches the cue-only files -- which are missing every output column `washout_metrics.py` needs, so
`metrics()` raises `SystemExit` from inside a list comprehension and the whole batch dies with no
table at all. The same broad glob would also sweep up any other `/tmp/*.csv` left over from the
README's own single-file examples, such as `/tmp/run.csv` or `/tmp/chirp.csv`.)

(`washout_replay` reads plain CSV only, hence the `gunzip -c`; `washout_metrics.py` itself is
gzip-transparent and takes `.csv.gz` directly, which is useful for archived *replay* output.)

Pass the exact `configuration.toml` revision named above — the baseline is the shipped
settings, so no `--set` here.

**Frozen baseline, measured 2026-08-29 at `configuration.toml` revision `f516263`:**

| segment | rows | sec | fs | sat_heave | sat_rot | sat_envelope | sat_sl_acc | wrms | band_ratio | jerk_p95 | lag_ms | peak_raw_mm | peak_out_mm |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `cruise_calm` | 3906 | 103.1 | 30.7 | 19.92 | 0.00 | 0.00 | 3.02 | 0.0067 | 0.298 | 6969988.4 | 880.8 | 31.7 | 30.0 |
| `steep_turns` | 3211 | 71.6 | 45.2 | 79.66 | 20.02 | 0.00 | 33.45 | 0.0472 | 0.109 | 4163390.5 | 994.9 | 43.1 | 30.0 |
| `climb_descent` | 4008 | 85.6 | 47.4 | 88.27 | 15.44 | 0.00 | 33.06 | 0.0565 | 0.032 | 5096508.6 | 506.8 | 76.8 | 30.0 |
| `turbulence` | 4309 | 91.0 | 47.9 | 61.82 | 6.64 | 0.00 | 86.19 | 0.0915 | 0.055 | 7036577.5 | 605.3 | 45.3 | 30.0 |
| `ground_takeoff` | 2725 | 61.4 | 43.8 | 25.54 | 0.00 | 0.00 | 51.63 | 0.0233 | 0.001 | 10845129.0 | 502.2 | 33.7 | 30.0 |
| `approach_landing` | 6860 | 155.8 | 43.9 | 35.44 | 0.00 | 0.00 | 25.90 | 0.0406 | 0.148 | 4830679.0 | 728.5 | 36.5 | 31.5 |
| `acceptance` | 21859 | 463.5 | 50.9 | 37.71 | 0.59 | 0.00 | 31.42 | 0.0293 | 0.080 | 24107006.9 | 589.6 | 43.4 | 31.2 |

`steep_turns`'s `lag_ms` of 994.9 sits at the estimator's 1000 ms search ceiling — read it as a
lower bound, not a value. Anything a candidate measures there is still a valid *comparison*,
because the ceiling only truncates upward.

### `--verify` status of the frozen set

`cruise_calm`, `steep_turns`, `climb_descent` and `turbulence` verify bit-exactly. The three
segments with ground contact do **not**, and cannot be made to: the ground-roll rumble is a
free-running 12 Hz oscillator whose phase was not recorded when these files were captured.
Measured on `ground_takeoff`, the entire discrepancy is the effects contribution and is confined
to on-ground ticks — in the air, across 1017 ticks, `max |live_heave| difference = 0.0000 mm`;
on the ground, across 408 ticks, 1.7442 mm, matching `rumble_gain`'s 0.9 mm amplitude at a full
phase mismatch. The washout itself is bit-exact throughout.

The plugin now records that oscillator state (columns 58–61) and the replay seeds from it, so
recordings made from here on verify everywhere. These seven predate those columns. **Do not
re-fly them for this** — the four segments the campaign's gates are decided on all verify, and
the ground segments' known deviation is bounded, characterised and irrelevant to the heave
washout under test.

## Headline number

`cruise_calm`'s `sat_heave%` is what the rest of the campaign is measured against. The design
spec predicts 80–100% at the shipped settings.

**Measured `sat_heave%` (`cruise_calm`): 19.92 %.**

The prediction was too high — but it assumed "calm cruise" wanders by ±0.05 g, and this segment
was flown deliberately calm (clear, 2 kt wind). It reached `mean |Δg| = 0.043 g`. The mechanism
is confirmed and the direction is right; the specific 80–100 % figure was an overestimate for
*this* flight condition. The manoeuvring segments land in the predicted band: 79.66 % and
88.27 %.

## Reachable surge/sway envelope (2026-08-31, `tools/envelope_probe`)

Measured on the shipped `[geometry]`, with the real `StewartKinematics::solve`. This section was
revised after fix round 1 (below, "Fix round 1"): the corner definition in the first pass was the
per-axis *theoretical* clamp maximum, not an operating point, and the shipped config never gets
near it in flight. It stays in this section as a documented, deferred defect (Ruling 2), and the
per-axis limits are now derived from the *measured* operating envelope instead (Ruling 1).

### Home pose — measurement

Bare upper bound, never available in flight (heave/tilt/rotational all zero):

```
home pose                    surge + 141.14 / - 174.41    sway + 146.89 / - 146.89  (mm)
```

| Base pose | surge + | surge − | sway + | sway − |
|---|---|---|---|---|
| home | 141.14 | −174.41 | 146.89 | −146.89 |

### Theoretical clamp corner — measurement, kept as a documented defect (Ruling 2)

Heave at its configured limit, tilt-coordination and rotational stacked at their combined
per-axis limit (`tilt_limit_deg = 7` + `rot_limit_deg = 7` = 14° combined roll/pitch,
`rot_limit_deg = 7` yaw alone) — the theoretical maximum two independently-clamped channels can
reach together, not a pose ever measured in a recording:

```
corner h+30 r/p+14 y+7       BASE POSE UNREACHABLE
corner h+30 r/p-14 y-7       BASE POSE UNREACHABLE
corner h-30 r/p+14 y+7       BASE POSE UNREACHABLE
corner h-30 r/p-14 y-7       BASE POSE UNREACHABLE

largest symmetric reachable roll=pitch (all else zero): 6.62 deg
```

| Base pose | surge + | surge − | sway + | sway − |
|---|---|---|---|---|
| corner heave +30, roll/pitch +14, yaw +7 | UNREACHABLE | UNREACHABLE | UNREACHABLE | UNREACHABLE |
| corner heave +30, roll/pitch −14, yaw −7 | UNREACHABLE | UNREACHABLE | UNREACHABLE | UNREACHABLE |
| corner heave −30, roll/pitch +14, yaw +7 | UNREACHABLE | UNREACHABLE | UNREACHABLE | UNREACHABLE |
| corner heave −30, roll/pitch −14, yaw −7 | UNREACHABLE | UNREACHABLE | UNREACHABLE | UNREACHABLE |

All four corner base poses are unreachable with zero surge/sway added. Confirmed with an ad-hoc
leg-level dump (not committed) against the same linked `StewartKinematics`: `roll=+14, pitch=+14`
alone (no heave, no yaw, no surge/sway) already leaves legs 1 and 2 unreachable. `heave=±30` alone
and `yaw=±7` alone are each independently reachable; the combined `roll+pitch = ±14°` is what
exceeds the geometry — the platform can hold at most `roll=pitch=6.62°` symmetrically, less than
half the theoretical 14° two independently-clamped channels are allowed to sum to.

That 14° figure is not a probe assumption: `WashoutFilter.cpp:87-88` sums the tilt-coordination
and rotational channels onto the same pose fields (`tiltRoll_ + rollAngle_`,
`tiltPitch_ + pitchAngle_`) after each is independently clamped to its own limit, so the *config*
allows commanding it — it is a real latent defect in the shipped `configuration.toml`
(`tilt_limit_deg = 7` and `rot_limit_deg = 7`, each tuned and flown in isolation on 2026-08-30 per
`tuning-log.md`, with nothing in that campaign checking their combined worst case against the
physical envelope). `clampToReachable` means it is not a crash risk in production — the platform
degrades gracefully by scaling all six DOF down together — but per Ruling 2 this defect is not
this campaign's to fix, and it is not what the limits below are derived from.

### Measured operating envelope, all seven reference recordings — measurement

`max |live_heave|` (mm) and `max |live_roll|`, `|live_pitch|`, `|live_yaw|` (deg), over the full
duration of each `washout_replay` run against `configuration.toml` (the shipped, adopted config):

| File | rows | max \|live_heave\| | max \|live_roll\| | max \|live_pitch\| | max \|live_yaw\| |
|---|---|---|---|---|---|
| `acceptance` | 21859 | 22.04 | 7.29 | 5.52 | **7.00** |
| `approach_landing` | 6860 | 27.51 | 3.03 | 7.64 | 1.82 |
| `climb_descent` | 4008 | **30.00** | 2.98 | **9.10** | 1.51 |
| `cruise_calm` | 3906 | 6.50 | 0.92 | 0.85 | 0.42 |
| `ground_takeoff` | 2725 | 11.74 | 2.21 | 6.38 | 0.86 |
| `steep_turns` | 3211 | **30.00** | **7.68** | 3.19 | 3.88 |
| `turbulence` | 4309 | **30.00** | 5.98 | 3.86 | 3.90 |
| **overall max** | | **30.00** | **7.68** | **9.10** | **7.00** |

All seven files replayed cleanly (`washout_replay --cues ... --config configuration.toml --out
...`, exit 0 on each); none needed to be dropped. The `live_heave` maxima of exactly 30.00 mm
(`climb_descent`, `steep_turns`, `turbulence`) are the configured `heave_limit_mm` clamp, genuinely
reached and held, not a search-ceiling artifact — same for `live_yaw`'s 7.00° in `acceptance`
against `rot_limit_deg = 7`.

**These four per-axis maxima never co-occur.** The `reach_scale` column (`clampToReachable`'s own
diagnostic scale factor, computed live inside the real replayed chain) stays at exactly `1.0` for
six of the seven files throughout, and dips only to `0.9968` in `steep_turns` — i.e. across
41,878 replayed ticks spanning every phase of flight this campaign recorded, the plugin's actual
commanded pose was, at its closest, 0.32 % away from needing any clamping at all. The shipped
config has never come anywhere near demanding an unreachable pose in flight.

### Empirical corner — attempted, found unreachable (second blocking finding)

Per Ruling 4, the empirical corner is each of the four maxima above, independently rounded up to
the next whole unit, combined into one pose and probed with `envelope_probe --pose`, in the same
four sign combinations the theoretical corner used (heave sign independent, roll/pitch/yaw sharing
a sign):

heave = 30, roll = 8, pitch = 10, yaw = 7 (ceil of 7.6778 → 8, 9.0986 → 10; 30.00 and 7.00 already
whole).

```
$ ./tools/build/envelope_probe --config configuration.toml --pose 30,8,10,7
pose h+30.00 r+8.00 p+10.00 y+7.00 BASE POSE UNREACHABLE
$ ./tools/build/envelope_probe --config configuration.toml --pose 30,-8,-10,-7
pose h+30.00 r-8.00 p-10.00 y-7.00 BASE POSE UNREACHABLE
$ ./tools/build/envelope_probe --config configuration.toml --pose -30,8,10,7
pose h-30.00 r+8.00 p+10.00 y+7.00 BASE POSE UNREACHABLE
$ ./tools/build/envelope_probe --config configuration.toml --pose -30,-8,-10,-7
pose h-30.00 r-8.00 p-10.00 y-7.00 BASE POSE UNREACHABLE
```

All four sign combinations of the empirical corner are unreachable too — down to `roll=+8,
pitch=+10` alone (no heave, no yaw), confirmed by the same leg-level dump used above (legs 1 and 2
fail). This matches the "largest symmetric reachable roll=pitch = 6.62°" figure directly: 8° and
10° both individually exceed it, and Ruling 2's finding is that even the *smaller*, tuned
`tilt_limit_deg`/`rot_limit_deg` values already exceed the physical envelope on the roll/pitch
diagonal, so any corner built by summing independent per-axis peaks — theoretical or measured — on
that diagonal is at risk of landing outside it.

**This is not, however, the "shipped config cannot hold a pose it demonstrably reached"
finding Ruling 4 was checking for.** The `reach_scale` evidence in the section above rules that
out directly: no single recorded tick in any of the seven files ever asked for anything close to
this combination — `max |live_roll| = 7.68°` happened in `steep_turns`, `max |live_pitch| = 9.10°`
happened in `climb_descent`, at different moments, in different flights. The empirical corner as
constructed unions each channel's own independent peak the same way the theoretical corner did,
just with tuned/measured numbers instead of configured clamp values — it inherits the same
"never actually co-occurs" structure Ruling 1 was correcting for, one level down. Building a
corner that is both empirically grounded *and* jointly reachable (e.g. from the actual joint
extreme of a chosen metric across ticks, rather than the union of four independently-timed
per-axis peaks) is a decision beyond this task's brief and is not made here.

### Chosen limits: still not determined — second blocking finding, needs a ruling

`surge_limit_mm` and `sway_limit_mm` cannot be computed as 70 % of the smallest surge/sway travel
across the empirical-corner rows, because none of those rows solve. Tasks 2 and 3 remain blocked
on a decision this task cannot make on its own: how to construct a corner that is simultaneously
(a) grounded in the measured operating envelope, per Ruling 1, and (b) actually jointly reachable —
options include a per-tick joint extreme (e.g. the recorded tick with the largest
`|roll|+|pitch|` or largest `reach_scale`-adjacent margin) rather than a per-axis-peak union, or a
smaller, explicitly conservative fraction of the four measured maxima chosen to land inside the
6.62° symmetric limit.

### Fix round 1 (2026-08-31)

The above supersedes the original write-up, which used only the theoretical clamp corner and
concluded "chosen limits: not determined" for that reason (Ruling 1: the corner definition was
wrong, not the geometry — every recording in this campaign measures `sat_envelope = 0.00 %` /
`reach_scale ≈ 1.0`, so flight never approaches the theoretical clamp corner). This round's
operating-envelope measurement and empirical-corner attempt are new; the theoretical corner and
its finding are retained per Ruling 2, unchanged from the original pass.
