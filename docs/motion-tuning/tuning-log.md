# Tuning Log

One row per change. Append, never rewrite. Offline metrics come from
`washout_metrics.py`; the rig verdict is a sentence in the pilot's own words.
See `docs/motion-tuning/README.md` for how to produce every number in this table
and `docs/superpowers/plans/2026-08-29-motion-heave-tuning.md` for the stage plan
these rows track.

The first row below is a worked example illustrating the format — **not real data**. Leave it in
place as documentation; real entries follow after it.

| Date | Stage | Parameter | Old → New | sat_heave% | wrms | jerk_p95 | lag_ms (Δ) | Rig verdict | Decision |
|---|---|---|---|---|---|---|---|---|---|
| *(example)* | 3 | `washout.heave_pos_washout_tau` + `washout.heave_vel_washout_tau` | 2.0 → 0.5 | 91.4 → 38.2 | 0.081 → 0.049 | 612 → 470 | +4 | "Cruise is noticeably calmer; steep turns still snap a bit on reversal." | Adopted for cruise; carry the reversal snap into Stage 5 (anti-windup) rather than re-tuning here |
| 2026-08-29 | 2 | — (baseline capture) | — | — | — | — | — | not applicable | Seven segments recorded on the sim-PC at revision `f516263`, cue-only exports committed to `MotionProviderPlugin/reference/`, metrics frozen in `baseline-metrics.md`. Stage 1 was settled from the real recordings instead of the synthetic chirp — see that file. |
| 2026-08-29 | 3 | `washout.heave_vel_washout_tau` + `washout.heave_pos_washout_tau` | 2.0 → 1.0 | 19.92 → 2.84 | 0.0067 → 0.0045 | 6969988 → 4602496 | **−32.6** | not yet flown | Offline only. Passes every gate but leaves the least margin of the four; kept as a fallback, not a finalist. |
| 2026-08-29 | 3 | `washout.heave_vel_washout_tau` + `washout.heave_pos_washout_tau` | 2.0 → 0.6 | 19.92 → 0.00 | 0.0067 → 0.0034 | 6969988 → 3254581 | **−195.7** | not yet flown | Offline only. Fallback if 0.25 and 0.4 both feel too sharp at the rig. |
| 2026-08-29 | 3 | `washout.heave_vel_washout_tau` + `washout.heave_pos_washout_tau` | 2.0 → 0.4 | 19.92 → 0.00 | 0.0067 → 0.0026 | 6969988 → 2332930 | **−326.2** | not yet flown | **Finalist 2.** Second choice for the first rig session. |
| 2026-08-29 | 3 | `washout.heave_vel_washout_tau` + `washout.heave_pos_washout_tau` | 2.0 → 0.25 | 19.92 → 0.00 | 0.0067 → 0.0017 | 6969988 → 1785700 | **−489.3** | not yet flown | **Finalist 1.** Strongest de-saturation and lowest lag; first candidate to fly. |

### Stage 3 sweep — the other segments

The rows above use `cruise_calm`, the log's default segment. The same sweep across the other
three airborne segments, for the two finalists:

| segment | sat_heave (2.0 → 0.4 → 0.25) | lag_ms (2.0 → 0.4 → 0.25) | sat_sl_acc (2.0 → 0.4 → 0.25) |
|---|---|---|---|
| `steep_turns` | 79.66 → 11.62 → 0.34 | 994.9 → 486.4 → 309.5 | 33.45 → 13.58 → 8.19 |
| `climb_descent` | 88.27 → 64.12 → 34.86 | 506.8 → 168.9 → 105.6 | 33.06 → 22.88 → 15.32 |
| `turbulence` | 61.82 → 20.47 → 4.36 | 605.3 → 375.7 → 292.2 | 86.19 → 91.34 → **93.85** |

`sat_rot` is unchanged across the whole sweep (20.02 / 15.44 / 6.64) — a useful check that the
sweep moved only what it claimed to.

**Two things to carry into the rig session:**

1. **`lag_ms` falls by 60–70 %, it does not rise.** The campaign's hard constraint was "no added
   perceptible lag", and the fix for saturation *reduces* the washout's phase delay rather than
   trading against it. The 15 ms gate is not the binding consideration here; it is passed in the
   favourable direction by two orders of magnitude.
2. **`sat_sl_acc` rises in turbulence** — 86.19 % at the shipped setting to 93.85 % at τ = 0.25,
   against the expectation that a de-saturated channel would demand *less* acceleration. A shorter
   τ raises the corner frequency, so the channel follows faster content: smaller excursion, higher
   acceleration. This matters because `lag_ms` is computed from `live_heave`, which is **before**
   the limiter — more limiter engagement adds lag the metric cannot see. **Feel this in turbulence,
   not in cruise.** It is the one place the offline numbers may flatter a candidate.

`climb_descent` stays the hardest segment at 34.86 % even at τ = 0.25. The design spec predicted
exactly this: Stage 3 removes most of the overdrive but not all of it, and the residue is what
Stage 5's anti-windup and Stage 8's amplitude choice have to absorb.

**Column meanings:**

- **Date** — when the candidate was decided, not necessarily when it was recorded.
- **Stage** — the campaign stage number from the plan (0–10). Ad-hoc exploration outside the
  staged plan still gets a row; write the nearest stage and a note in Decision.
- **Parameter** — the exact `section.key` form used with `washout_replay --set`/`--sweep`, so
  the row doubles as the reproduction command. Multiple parameters changed together (Stage 3's
  coupled pair; a code change with no single knob) get multiple keys or a short description.
- **Old → New** — the config value(s) before and after, in the same order as Parameter.
- **sat_heave%, wrms, jerk_p95** — `old → new` from `washout_metrics.py` on the segment named
  in the row (default: `cruise_calm` unless stated otherwise) . These three must all fall for a
  normal stage; Stage 8 (bringing amplitude back) inverts this — see the design spec.
- **lag_ms (Δ)** — the *change* in `lag_ms` versus the frozen baseline in
  `baseline-metrics.md`, not the absolute value. Must stay ≤ +15 ms or the candidate is
  disqualified before it is ever flown, regardless of every other number.
- **Rig verdict** — one sentence, the pilot's own words, from an actual rig flight. Write
  "not yet flown" for an offline-only candidate; a row with a real decision needs a real
  rig verdict, not a restatement of the metrics.
- **Decision** — adopted / rejected / needs another rig session, plus what happens next. If
  the rig verdict disagrees with the offline metrics (rig veto), say so explicitly here — that
  disagreement is itself a campaign finding, not something to smooth over.
