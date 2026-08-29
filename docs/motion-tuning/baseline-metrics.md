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

Recorded on the Mac's X-Plane 12 with the rig disconnected (the plugin stays disarmed; cues
are identical either way). One file per row, cue-only and gzipped into
`MotionProviderPlugin/reference/`:

| Segment | Duration target | Notes |
|---|---|---|
| `cruise_calm` | 60–90 s | The segment that matters most — by definition nothing may pump here. |
| `steep_turns` | 60–90 s | |
| `climb_descent` | 60–90 s | Alternating climb/descent. |
| `turbulence` | 60–90 s | Defined turbulence setting, not ambient chop. |
| `ground_takeoff` | 60–90 s | Ground roll through takeoff. |
| `approach_landing` | 60–90 s | |
| `acceptance` | 8–10 min | Continuous mixed flight; catches interactions and slow state buildup the short segments miss. Also the file Stage 10 replays against the final config. |

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

Output of, over all seven reference files:

```bash
MotionProviderPlugin/tools/.venv/bin/python MotionProviderPlugin/tools/washout_metrics.py \
    MotionProviderPlugin/reference/*.csv.gz
```

(paste the table here)

## Headline number

`cruise_calm`'s `sat_heave%` is what the rest of the campaign is measured against. The design
spec predicts 80–100% at the shipped settings.

**Measured `sat_heave%` (`cruise_calm`):**
