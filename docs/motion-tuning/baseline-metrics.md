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

- **Aircraft:** Piper Arrow III (vFlightAir custom PA28-201R add-on) — not a stock X-Plane
  model. Every number in this file, and everything measured against it, is valid for this
  aircraft only; a different type needs its own baseline before its numbers mean anything.
- **X-Plane version:**
- **Weather preset:**
- **Machine (model, OS version):**
- **Framerate** (median `1/dt_real` from the recordings, fps):
- **`configuration.toml` git revision** (`git rev-parse HEAD` at capture time):

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

## Stage 1 — sanity check against the analysis

Before recording anything, Stage 1 runs a synthetic chirp and a sequence of sine-amplitude
sweeps through the washout to test the diagnosis in
`docs/superpowers/specs/2026-08-29-motion-heave-tuning-design.md` before any tuning stage
leans on it.

- **Predicted:** `|G|max ≈ 0.91 s²` near 0.067 Hz; clamp onset near `|Δg| ≈ 0.022 g`.
- **Measured clamp onset:**
- **Measured peak frequency:**
- **Verdict:** (within ~2× of the prediction → diagnosis holds, continue; an order of
  magnitude off → stop the campaign and re-derive)

## Metrics table

**Two steps, in this order.** `washout_metrics.py` measures *replay output*; the reference
files are cue-only exports and carry none of the columns it needs (`live_heave`,
`heave_clamped`, `sent*`, `rot_*_clamped`, `sl_*_clip`, `heave_pos_raw`, `reach_scale`).
Pointing the script at `reference/*.csv.gz` directly is not a shortcut — it aborts naming the
first missing column. Replay first, then measure the replay:

```bash
cd MotionProviderPlugin
for seg in cruise_calm steep_turns climb_descent turbulence \
           ground_takeoff approach_landing acceptance; do
    gunzip -c "reference/$seg.csv.gz" > "/tmp/$seg.cues.csv"
    ./tools/build/washout_replay --cues "/tmp/$seg.cues.csv" \
        --config configuration.toml --out "/tmp/$seg.csv"
done
tools/.venv/bin/python tools/washout_metrics.py /tmp/*.csv
```

(`washout_replay` reads plain CSV only, hence the `gunzip -c`; `washout_metrics.py` itself is
gzip-transparent and takes `.csv.gz` directly, which is useful for archived *replay* output.)

Pass the exact `configuration.toml` revision named above — the baseline is the shipped
settings, so no `--set` here.

(paste the table here)

## Headline number

`cruise_calm`'s `sat_heave%` is what the rest of the campaign is measured against. The design
spec predicts 80–100% at the shipped settings.

**Measured `sat_heave%` (`cruise_calm`):**
