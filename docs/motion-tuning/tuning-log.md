# Tuning Log

One row per change. Append, never rewrite. Offline metrics come from
`washout_metrics.py`; the rig verdict is a sentence in the pilot's own words.
See `docs/motion-tuning/README.md` for how to produce every number in this table
and `docs/superpowers/plans/2026-08-29-motion-heave-tuning.md` for the stage plan
these rows track.

The first row below is a worked example illustrating the format — **not real data** (Stage 2
hasn't run yet). Leave it in place as documentation; real entries append after it.

| Date | Stage | Parameter | Old → New | sat_heave% | wrms | jerk_p95 | lag_ms (Δ) | Rig verdict | Decision |
|---|---|---|---|---|---|---|---|---|---|
| 2026-08-29 | 3 | `washout.heave_pos_washout_tau` + `washout.heave_vel_washout_tau` | 2.0 → 0.5 | 91.4 → 38.2 | 0.081 → 0.049 | 612 → 470 | +4 | "Cruise is noticeably calmer; steep turns still snap a bit on reversal." | Adopted for cruise; carry the reversal snap into Stage 5 (anti-windup) rather than re-tuning here |

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
