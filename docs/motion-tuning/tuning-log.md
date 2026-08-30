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

### Stage 3 — flown at the rig 2026-08-30, **adopted**

Two flights, same session, same conditions (clear, 2 kt wind, 3 kt gust), runway takeoff then level
flight with a few turns; recording stopped, turbulence set to medium mid-flight, second recording
started. Four files in `MotionProviderPlugin/measurementCSVs/heave-washout/`. Rig armed
(`arm_state = 2` throughout), plugin rebuilt from `9369a3f`, so all four carry the effects-state
columns and **all four verify bit-exactly** — the first recordings for which that holds on ground
segments too.

| | base (τ=2.0) | **0.25** | | base turb | **0.25 turb** |
|---|---|---|---|---|---|
| sat_heave | 49.72 % | **0.00 %** | | 75.77 % | **1.19 %** |
| sat_sl_acc | 31.91 % | 21.31 % | | 49.60 % | **28.35 %** |
| wrms | 0.0401 | 0.0153 | | 0.0789 | 0.0554 |
| jerk_p95 | 45.6 M | 20.4 M | | 28.1 M | 19.3 M |
| lag_ms | 641.2 | **261.8** | | 863.5 | **358.5** |
| peak_out_mm | 30.0 (pinned) | **27.0** (free) | | 30.0 | 30.0 |

**Rig verdict (pilot, before seeing any number):** *"base ruppig wie immer, Flugzeug kaum zu
kontrollieren mit dem ganzen Gewackel. 0.25 — plötzlich ist die Motion so smooth wie es sein
sollte. Kein Problem mehr das Flugzeug beim Cruise ruhig zu halten. Wenn es was auszusetzen gäbe,
dann eher, dass die Amplituden wieder etwas größer sein könnten. Und auf dem Ground scheint es
immer noch etwas ruppig, aber nicht unhandlebar."*

**Decision: adopted.** τ = 0.25 for both `heave_vel_washout_tau` and `heave_pos_washout_tau`.

**Two findings from this session worth carrying forward:**

1. **A prediction of mine was wrong.** The offline sweep warned that `sat_sl_acc` would *rise* in
   turbulence (86.19 → 93.85 % on the `turbulence` reference segment). At the rig it *fell*
   (49.60 → 28.35 %). The reason: the clamped square wave is itself the largest acceleration
   demand, and removing it outweighs the higher corner frequency. The offline segment used
   **severe** turbulence, where the limiter saturates either way; this flight used **medium**. The
   caution may still hold at severe — untested.
2. **The pilot is in the loop, and the offline replay cannot see it.** In the turbulence pair the
   *input* was rougher for the 0.25 flight (mean |g−1| 0.2572 vs 0.2244 g, peak 1.073 vs 0.814 g),
   yet mean pitch rate dropped from 5.972 to 3.713 °/s — the aircraft was flown 38 % more calmly
   because the platform allowed it. Replay works from recorded cues and therefore *understates* the
   real improvement.

### Stage 8 candidate — `heave_gain` sweep on the new rig recordings

Swept against `0.25_noTurb` and `0.25_turb` with τ = 0.25 fixed:

| gain | noTurb sat_heave | noTurb peak_out | turb sat_heave | **turb sat_sl_acc** |
|---|---|---|---|---|
| 0.15 (current) | 0.00 | 27.0 mm | 1.19 | 32.79 % |
| **0.20** | 0.42 | **30.0 mm** | 5.69 | 59.14 % |
| 0.25 | 1.28 | 30.0 mm | 15.33 | 66.08 % |
| 0.30 | 2.56 | 30.0 mm | 23.34 | 81.04 % |
| 0.40 | 5.90 | 30.0 mm | 35.79 | 82.48 % |

The binding constraint is the acceleration limiter in turbulence, not the heave clamp. `0.20` fills
the ±30 mm envelope in cruise at negligible saturation; `0.25` starts bringing the pumping back in
turbulence and is the stretch candidate to feel against it.

`lag_ms` varies across this sweep (261.8 → 221.6 ms) — that is **not** physical. Gain is a linear
scalar and cannot change a linear filter's phase; the steps are exactly 20.1 ms, one sample bin at
49.6 fps. Estimator quantisation, not an improvement.

### Stage 8 — flown at the rig 2026-08-30, **rejected: `heave_gain` stays 0.15**

Four recordings in `MotionProviderPlugin/measurementCSVs/heave-gain/`: 0.15 and 0.20, each without
and with medium turbulence, τ = 0.25 throughout.

**Rig verdict (pilot, before seeing any number):** *"0.15 im Start und Cruise sehr smooth. Cruise
mit 0.2 brachte eine Ruppigkeit zurück. Definitiv 0.15 ist besser. Bei Turbulenzen fühlte sich der
0.2 gain überraschenderweise nicht schlechter an als 0.15. Overall würde ich aber den 0.15 für
normale Flüge bevorzugen."* 0.25 was not flown.

**The four flights are not comparable to each other.** The 0.20 flights carried roughly three times
the mean g excursion of the 0.15 flights (0.1839 vs 0.0596 g without turbulence; 0.4068 vs 0.1477 g
with), and `0.15-noTurb` was 36 % ground roll — it contained the takeoff — while `0.2-noTurb` was
entirely airborne. A raw side-by-side of their metrics attributes to the gain what was mostly the
air and the flying.

**Isolated by replaying every recording at both gains**, which is what the harness exists for:

| cues @ gain | sat_heave | sat_sl_acc | wrms | jerk_p95 | peak_out |
|---|---|---|---|---|---|
| `0.15-noTurb` @ 0.15 | 0.00 | 24.80 | 0.0310 | 33.4 M | 22.9 |
| `0.15-noTurb` @ 0.20 | 0.13 | 25.06 | 0.0367 | **38.5 M (+15 %)** | 30.0 |
| `0.15-turb` @ 0.15 | 0.00 | 52.92 | 0.0663 | 27.2 M | 25.8 |
| `0.15-turb` @ 0.20 | 0.55 | 74.45 | 0.0883 | **28.2 M (+4 %)** | 30.0 |
| `0.2-noTurb` @ 0.15 | 3.03 | 2.87 | 0.0156 | 29.7 M | 30.0 |
| `0.2-noTurb` @ 0.20 | 3.93 | 4.27 | 0.0196 | **35.7 M (+20 %)** | 30.0 |
| `0.2-turb` @ 0.15 | 0.19 | 80.96 | 0.0454 | 89.3 M | 30.0 |
| `0.2-turb` @ 0.20 | 3.15 | 87.57 | 0.0588 | **110.5 M (+24 %)** | 30.0 |

On identical cues, gain 0.20 raises `jerk_p95` by 4–24 % in every case, and `wrms` by 18–30 %.
`jerk_p95` is the mechanical-harshness measure, and harshness is exactly what the pilot reported
returning. **The rig verdict is confirmed by the isolated comparison, not merely respected.**

**A weighting error on my part, recorded because it changed a recommendation.** The +18 % rise in
`jerk_p95` was already visible in the Stage 8 sweep table above. I recommended 0.20 anyway, because
I weighted `sat_heave` staying near zero and `peak_out` reaching the full 30 mm. Amplitude and
saturation were the wrong things to weight; jerk was the one that mattered. The gates say
"`sat_heave`, `wrms` and `jerk_p95` must all fall" — `wrms` and `jerk_p95` both *rose*, so the
candidate should never have been recommended on the metrics either.

**Turbulence:** the pilot found 0.20 no worse there. The milder turbulence cue set agrees (+4 %
jerk), the rougher one does not (+24 %). No clean "turbulence masks it" rule; the perception matches
the case actually flown and should not be generalised.

**Method, adopted from here on.** Matched flights are not required and should not be attempted. The
rig session produces the *feel*; the numbers come from replaying each recording at every candidate
setting, so the comparison runs on identical cues. This was the first session where that separation
was actually needed, and it turned an unusable A/B into a clean one.

**Decision: rejected. `heave_gain` stays 0.15.** Stage 8 closes with "no change", which is a result.
The pilot's amplitude wish was about roll/pitch/yaw, not heave — that moves to the rotational stage.

### Rotational amplitude — measured, not yet a candidate

The pilot's "amplitudes could be bigger" was mainly about roll/pitch/yaw, not heave. Measured on
`0.25_noTurb`, the rotational channel is **not saturated** (`sat_rot` 0.00 %), which is why gain
works there and did not for heave:

| `rot_*_gain` | roll | pitch | yaw | rot clamped |
|---|---|---|---|---|
| 0.42 (current) | 3.04° | 4.63° | 2.32° | 0.00 % |
| 0.60 | 3.45° | 5.47° | **3.00°** | 4.11 % |
| 0.70 (code default) | 3.75° | 5.74° | **3.00°** | 8.21 % |
| 0.90 | 3.85° | 6.00° | **3.00°** | 15.90 % |

Yaw pins at `rot_limit_deg = 3` from gain 0.60 on, so gain and limit have to rise together — two
halves of one intent, like the coupled washout pair. Raising the limit alone gains almost nothing
(pitch 4.63 → 5.51° at limit 5, nothing beyond; roll and yaw unchanged), because the signal never
reaches 3° at the current gain. `sat_envelope` stays 0.00 % even at a 12° limit, so the kinematics
are not the constraint — but that only proves the pose is *reachable*, not that it is mechanically
sensible at speed. This needs its own stage and its own rig session.

Also recorded while looking: `WashoutFilter::update` sets `p.surge = 0` and `p.sway = 0`
unconditionally. The platform is a 6DOF machine driven in 4 DOF; longitudinal and lateral
accelerations are rendered as tilt coordination, never as translation. That explains the pilot
never consciously noticing sway or surge — they do not exist. Adding translational onset cues would
be a feature, not tuning.

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
