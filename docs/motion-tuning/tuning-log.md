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

### Rotational amplitude — candidate prepared 2026-08-30, **not yet flown**

Swept offline against `reference/steep_turns` (the segment with the most rotational content) and
the armed `heave-gain/0.15-noTurb` recording, at the adopted τ = 0.25 / `heave_gain` 0.15.

Where each knob binds depends on the flight condition, which is why one probe was not enough:

- **Steep turns:** the rotational channel is saturated. `rot_roll` reaches 3.13° and `rot_yaw`
  3.02° against `rot_limit_deg = 3`, so both are clipped. Yaw has no tilt contribution at all, so
  `live_yaw` sits at exactly 3.00.
- **Calm cruise:** the rotational channel is *not* saturated (`sat_rot` 0.31 %), but the tilt
  channel is — `tilt_pitch` sits at exactly 3.00 against `tilt_limit_deg = 3`.

**2D sweep on `steep_turns`** (`rot_*_gain` × `rot_limit_deg`):

| gain | limit | roll | pitch | yaw | sat_rot | sat_envelope | jerk_p95 |
|---|---|---|---|---|---|---|---|
| 0.42 | 3 (current) | 4.41° | 3.03° | 3.00° | 20.02 % | 0.00 % | 1.76 M |
| 0.42 | 5 | 6.41° | 3.03° | 3.88° | 6.98 % | 0.00 % | 1.94 M (+11 %) |
| **0.42** | **7** | **7.50°** | 3.03° | **3.88°** | **0.69 %** | **0.00 %** | **1.94 M (+11 %)** |
| 0.60 | 3 | 4.41° | 3.79° | 3.00° | 31.77 % | 0.00 % | 2.13 M (+21 %) |
| 0.60 | 5 | 6.41° | 4.12° | 5.00° | 13.45 % | 0.75 % | 2.41 M (+37 %) |
| 0.60 | 7 | 8.41° | 4.12° | 5.54° | 7.47 % | 3.27 % | 2.68 M (+53 %) |
| 0.70 | 3 | 4.41° | 3.89° | 3.00° | 35.38 % | 0.00 % | 2.47 M (+40 %) |
| 0.70 | 5 | 6.41° | 4.73° | 5.00° | 20.02 % | 1.62 % | 2.56 M (+46 %) |
| 0.70 | 7 | 8.41° | 4.73° | 6.47° | 9.69 % | 4.73 % | 2.96 M (+68 %) |

**Candidate: `rot_limit_deg = 7`, gains unchanged at 0.42.** Roll 4.41 → 7.50°, yaw 3.00 → 3.88°,
and saturation *falls* from 20.02 % to 0.69 % — more amplitude and less clipping from one knob, the
same shape of fix as the heave one. Cost: +11 % `jerk_p95` in turns, +0.4 % in cruise; `sat_envelope`
stays 0.00 %. Conservative fallback `rot_limit_deg = 5` (roll 6.41°, sat_rot 6.98 %, identical jerk)
if 7.5° of roll feels excessive — the only question the metrics cannot answer.

**The gains are deliberately not touched**, applying the Stage 8 lesson. Raising them costs 21–68 %
`jerk_p95`, which is what the pilot perceives as harshness, and at gain ≥ 0.60 with limit ≥ 5
`sat_envelope` leaves zero (0.75–4.73 %). Because `StewartKinematics::clampToReachable` scales all
six DOF together, envelope clipping would attenuate the just-repaired heave channel as a side
effect. At limit 3 more gain buys no roll amplitude at all (stays 4.41°, clipped) and only raises
clipping to 48.55 % — the heave mistake repeated.

**Held back for a separate step:** `tilt_limit_deg` 3 → 5 raises cruise pitch from 6.00 to 7.79° and
`tilt_roll` from 3.00 to 4.95° (nothing further above 5; the tilt naturally caps near 5.4°, and the
rate limiter never engages). It is a different sensation from the rotational cue — tilt coordination
renders *sustained* acceleration as a steady lean rather than a motion onset, and since
`WashoutFilter` zeroes surge and sway it is the only longitudinal and lateral information the
platform gives. Whether 5° of sustained lean reads as acceleration or merely as "tilted" is a
question for its own test.

### Rotational limit — flown 2026-08-30, **adopted, but the verdict carries almost no evidence**

Two recordings in `measurementCSVs/rotation-limit/` at `rot_limit_deg = 7`, both verifying
bit-exactly. `rot7-turb`: takeoff, climb and cruise with turbulence. `rot7-noTurb`: cruise and
landing after turbulence was switched off mid-flight.

**Rig verdict:** *"Fühlte sich mit 7 rund an. Kein Problem."*

**That verdict is about a change which did essentially nothing in these two flights.** Replaying
both recordings at limit 3 and limit 7 on identical cues:

| | limit 3 | limit 7 |
|---|---|---|
| `rot7-turb` | roll 2.96° pitch 4.19° yaw 1.75° | roll 2.95° pitch 4.19° yaw 1.75° |
| `rot7-noTurb` | roll 3.50° pitch 4.37° yaw 2.85° | roll 3.55° pitch 4.37° yaw 2.85° |

The rotational channel peaked at 3.44–3.45° raw, so it crossed the old 3° limit only barely and
rarely: `sat_rot` was 0.55 % and 0.98 % at limit 3, and 0.00 % at limit 7, with jerk unchanged
(+1.7 % / −0.7 %). The large gain measured in the sweep (roll 4.41 → 7.50°) came from
`reference/steep_turns`; these flights contained no steep turns, so nothing exercised it.

**Decision: adopted.** It removes real clipping in manoeuvring at no measurable cost, and it is
demonstrably harmless in normal flight. But "no problem" here means "no effect", not "an
improvement felt" — the amplitude benefit remains untested until a flight with steep turns.

### The tilt channel is saturated 56–63 % of the time — the real amplitude constraint

Looking at the same two recordings for why normal flight felt unchanged:

| | max `tilt_pitch` | ticks at the 3° tilt limit |
|---|---|---|
| `rot7-turb` | 3.00° | **56.05 %** |
| `rot7-noTurb` | 3.00° | **62.68 %** |

`tilt_pitch` sits pinned at exactly `tilt_limit_deg` for the majority of the flight. This is the
heave pattern found again in a different channel — and it is two orders of magnitude more binding
than the rotational limit that was just changed (0.5–1 %). The amplitude the pilot is missing in
normal flight was never in the rotational channel; it is here.

`tilt_limit_deg` swept on the same recordings:

| tilt_limit | live pitch (`noTurb`) | ticks at limit | jerk_p95 | sat_envelope | sat_sl_acc |
|---|---|---|---|---|---|
| 3 (current) | 4.37° | 58.83 % | 18.36 M | 0.00 % | 15.07 % |
| **5** | **5.58° (+28 %)** | 6.16 % | 18.52 M (**+1.6 %**) | 0.00 % | 15.35 % |
| 7 | 7.26° (+66 %) | 1.56 % | 18.65 M (+1.6 %) | 0.00 % | 15.74 % |

On `rot7-turb` the tilt caps naturally at 4.63°, so limits 5 and 7 are identical there.

**+28 % pitch amplitude for +1.6 % jerk** — an order of magnitude better trade than `heave_gain`,
which wanted 15–24 % jerk for its amplitude and was rejected for exactly that. The reason is
structural: tilt is a low-passed, rate-limited channel that changes slowly by construction, so
raising its clamp adds no high-frequency content. A gain, by contrast, scales the fast content too.

**Candidate: `tilt_limit_deg = 5`, stretch `7`.** The open question is qualitative and only the rig
can answer it: tilt coordination renders *sustained* acceleration as a steady lean, not a motion
onset, and since `WashoutFilter` zeroes surge and sway it is the platform's only longitudinal and
lateral information. At 7° the platform leans back noticeably. Whether that reads as acceleration or
merely as "tilted" depends on whether the visual agrees. **Judge it on takeoff and approach**, where
the sustained longitudinal accelerations are.

### Tilt limit + rate limit — flown 2026-08-30, **adopted: `tilt_limit_deg = 7`, `tilt_rate_limit_dps = 3`**

Three rig flights, in order.

**Flight 1, `tilt_limit_deg = 5`.** Pilot: *"5 ist gut."*

**Flight 2, `tilt_limit_deg = 7`.** Pilot: *"Bei 7 kommt definitiv wieder mehr Ruppigkeit ins Spiel.
Der erhöhte Tilt fühlt sich aber definitiv gut an, z.B. beim Setzen der Flaps, fühlt sich das
wirklich wie eine Bremswirkung an (nicht nur ein Tilt). Bei 7 mehr als bei 5. Wenn sich irgendwie
die zusätzliche Ruppigkeit verhindern liese, definitiv 7."*

That is the trade the section above predicted, plus the qualitative answer the offline sweep could
not give: the extra tilt **does** read as acceleration rather than as "tilted" — the flap-extension
cue was felt as braking. The cost was roughness.

**Hypothesis for the roughness.** A larger clamp means the tilt travels further, so for the same
low-pass it travels *faster*. Above roughly 3 °/s the vestibular system detects the motion as
rotation instead of interpreting it as gravity — the cue stops being a substitute for acceleration
and becomes a felt lean. `tilt_rate_limit_dps` bounds exactly that rate, and it was sitting at 5.

**This hypothesis was weak on the evidence available when it was made, and that was said before the
flight.** The two flown recordings showed `tilt_rate_pct_3dps` of 0.94 (limit 5) and 0.93 (limit 7)
— indistinguishable, because they were different flights with different cues. The commitment made
to the pilot was: if `tilt_rate_limit_dps = 3` does not remove the roughness, the rotation rate was
not the cause and the search continues elsewhere.

**Flight 3, `tilt_limit_deg = 7` + `tilt_rate_limit_dps = 3`.** Pilot: *"Voller Erfolg: Viel Tilt,
Ruppigkeit bleibt gering."*

**Isolated afterwards on that flight's own cues** (`tilt7-ratelimit3-motion-20260830-143423.csv`),
all four combinations replayed from one identical cue stream:

| limit / rate | `tilt_rate_pct_3dps` | `tilt_rate_p95` | `jerk_p95` | `sat_sl_acc` | `lag_ms` |
|---|---|---|---|---|---|
| 5 / 5 | 1.53 % | 1.06 | 22.21 M | 27.10 % | 226.3 |
| 5 / 3 | 0.51 % | 1.07 | 22.21 M | 26.99 % | 226.3 |
| **7 / 5** | **1.80 %** | 1.12 | 22.21 M | 27.32 % | 226.3 |
| **7 / 3** | **0.58 %** | 1.15 | 22.21 M | 27.23 % | 226.3 |

Both halves of the hypothesis hold once the cues are held fixed: raising the limit 5 → 7
**increases** the supra-threshold fraction (1.53 → 1.80 %), and tightening the rate limit 5 → 3 cuts
it by about two thirds at either limit. The effect two separate flights could not resolve is plainly
visible on one cue stream — which is the entire reason for the replay method.

**And it costs no amplitude:**

| limit / rate | max `tilt_pitch` | p95 | max `tilt_roll` | p95 |
|---|---|---|---|---|
| 5 / 5 | 5.00° | 2.97 | 5.00° | 4.85 |
| 7 / 5 | 5.55° | 2.97 | **7.00°** | 4.85 |
| **7 / 3** | 5.49° | 2.97 | **7.00°** | 4.82 |

The full 7° is still reached with the rate limit at 3; only the flanks are slower. `jerk_p95`,
`sat_sl_acc` and `lag_ms` are unchanged to three digits — the rate limit acts on a channel the
heave-derived `lag_ms` does not see, so **the pilot's judgement was the only instrument that could
have caught a delay here.** The warning given before the flight was explicit: 7° at 3 °/s takes
≈2.3 s instead of ≈1.4 s, and the pilot was asked to check that the braking cue still arrives *in
time*. It does.

**Note on the history of this parameter.** `tilt_rate_limit_dps` had already been 3 once and was
raised to 5 in the belief that it was masking jitter. It was not — the jitter was the saturated
heave channel, fixed in Stage 3. This is the third instance of the same pattern in this campaign
(see the next section): a parameter moved to suppress a symptom whose cause lay in a different
channel. With the cause removed, the parameter could go back.

Verified bit-exact against the adopted `configuration.toml`: `max |replay − recorded| = 0 mm/deg`
over 10 939 compared samples — the flown settings and the committed settings are the same settings.

### Why the gains are at 30–60 % of their defaults — history, from the pilot

The reduced amplitudes in `configuration.toml` (`heave_gain` 0.15 of 0.5, `rot_*_gain` 0.42 of 0.7,
`tilt_surge/sway_gain` 0.3 of 1.0) were **not** a considered amplitude choice. They were an attempt
to get the platform's harshness under control. Damping every channel was a reasonable response to a
real symptom; it simply acted on the wrong quantity, because the cause — channels pinned at
arbitrary clamps — was not visible without measurement.

That explains a contradiction in the original complaint: the platform felt harsh **and** too weak at
the same time. Reducing a gain into a clamped channel does not unclamp it. It only makes everything
smaller while the clipping, and therefore the harshness, stays.

**Consequence for the remaining stages: limit before gain, always.** Raising a gain on a channel
that is still clipping buys nothing but more clipping — literally what happened when `heave_gain`
went 0.5 → 0.15 and the pumping did not change.

**The gains are not equally expensive**, and this is structural rather than empirical:

| gain | status | cost in `jerk_p95` |
|---|---|---|
| `heave_gain` | tested at the rig, **rejected** | +15–24 % |
| `rot_*_gain` | swept offline | +21–68 % |
| `tilt_surge_gain` / `tilt_sway_gain` | **untested** | expected small |

A gain scales everything passing through its channel, including the fast content — which is why the
heave and rotational gains buy amplitude with harshness. The tilt channel is low-passed and
rate-limited, so it carries no fast content to amplify. That is the same reason raising
`tilt_limit_deg` cost only 1.6 % jerk, and it is why the tilt gain is the one worth trying once its
limit no longer clips.

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
