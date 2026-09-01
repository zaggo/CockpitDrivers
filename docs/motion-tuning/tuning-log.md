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

### Tilt gain — swept 2026-08-30, **candidate `tilt_surge_gain` / `tilt_sway_gain` = 0.4, not yet flown**

Swept together (they are the same channel on two axes: surge → pitch, sway → roll) on three cue
sets, replayed from the adopted `configuration.toml`. `at_limit` is the share of pitch and roll
samples within 0.01° of the 7° clamp; `pinned%` is the share of ticks with either axis at the clamp.

| gain | `rot7-noTurb` pitch p95 | `rot7-turb` pitch p95 | `tilt7-r3` roll p95 | `rot7-turb` pinned | `rot7-turb` jerk_p95 | `rot7-turb` sat_sl_acc |
|---|---|---|---|---|---|---|
| 0.30 (current) | 5.88° | 4.40° | 4.82° | **0.00 %** | 32.5 M | 37.77 % |
| **0.40** | 7.00° | **5.87° (+34 %)** | **6.15° (+28 %)** | **0.00 %** | **34.6 M (+6 %)** | **37.92 %** |
| 0.50 | 7.00° | 7.00° | 7.00° | 10.87 % | **46.6 M (+43 %)** | **48.07 %** |
| 0.60 | 7.00° | 7.00° | 7.00° | 44.32 % | 41.4 M | 45.26 % |
| 0.70 | 7.00° | 7.00° | 7.00° | 54.98 % | 38.7 M | 44.47 % |
| 1.00 | 7.00° | 7.00° | 7.00° | 77.03 % | 39.6 M | 46.51 % |

**The expectation stated in the section below — that the tilt gain would be the one cheap gain —
holds only up to 0.4, and is wrong above it.** Between 0.4 and 0.5 the turbulence cue set goes from
no clipping at all to 10.87 % pinned, and `jerk_p95` jumps 34.6 → 46.6 M (+35 %) with `sat_sl_acc`
+10 pp. That is the same order of cost that got `heave_gain = 0.20` rejected.

**The mechanism is not established, and one obvious explanation was tested and failed.** The natural
guess was that entering and leaving the clamp creates corners, so the cost should scale with clamp
*entries*. Counted, it does not: `rot7-turb` has 0 entries at 0.4 and only **2** at 0.5, 2 at 0.6,
3 at 0.7. Few long excursions, not many corners. What does track the cost is **time spent pinned**,
and that is also why `jerk_p95` *falls* again above 0.5 (46.6 → 41.4 → 38.7 M): with the channel
pinned 44–55 % of the time it simply moves less. **A lower `jerk_p95` at a higher gain is not a
better setting here — it is a more thoroughly clipped one.** The metric can be gamed by clipping
harder, which is worth remembering before reading any jerk number in isolation.

**Why 0.4 and not more.** It is the largest value with zero clipping on the turbulence set, and it
costs +6 % jerk and +0.15 pp acceleration-limiter load for +28–34 % of the amplitude that is
actually felt (p95, not peak). Above it, the binding constraint stops being the gain and becomes the
7° clamp again — limit before gain, for the fourth time in this campaign.

**Consequence for later.** The tilt channel is close to structurally maxed out at
`tilt_limit_deg = 7`. If more longitudinal/lateral cue is wanted after 0.4 has been flown, the next
lever is a *higher* `tilt_limit_deg`, not a higher gain — and the previous section established that
the roughness is governed by `tilt_rate_limit_dps`, not by the limit, so a 9° limit at 3 °/s is
plausible on paper. It is a large static lean and needs its own rig judgement; do not fold it into
the gain test.

**To fly it:** set `tilt_surge_gain = 0.4` and `tilt_sway_gain = 0.4` in `configuration.toml`,
"Reload config", no rebuild. Judge takeoff, flap extension and approach — the sustained longitudinal
accelerations — and whether turbulence brings roughness back.

### Tilt gain — flown 2026-08-30, **adopted at 0.4, but on weak evidence**

Pilot: *"War soweit ok. Nicht großartig anders wie vorher."* No roughness complaint.

Three recordings, `surgesway-a/b/c`. All three verify **bit-exact** against a config with the gains
at 0.4 and differ by 0.47–1.27 mm/deg against 0.3, so the setting was genuinely active — the first
thing to rule out when a change is not felt. `arm_state = 2` throughout; the rig was live.

**Which recording is which** (the pilot was unsure): `a` is calm end to end. Its `|g_nrml − 1|` RMS
per 20 s block runs 0.004 → 0.043 → 0.112 → 0.124 → 0.173 → 0.099 → 0.034 → 0.026 — it rises and
comes back down, which is takeoff, rotation and initial climb, not weather. `b` and `c` sit at
0.12–0.32 throughout: turbulence was on for **both**. The break falls between `a` and `b`.

**Why it was not felt:**

| | tilt p95 | actuator p95 excursion | 0.3→0.4 difference signal vs. what was already moving |
|---|---|---|---|
| `a` (calm, 144 s, 36 % on ground) | +33.4 % | **+23.6 %** | **55.8 %** |
| `b` (turbulent) | +33.4 % | +1.2 % | 23.6 % |
| `c` (turbulent) | +33.4 % | +3.9 % | 15.5 % |

**The extra tilt is not being clamped away.** `sat_envelope` is 0.00 % on all six replays,
`sat_heave` and `sat_rot` essentially 0, and `sat_sl_acc` moves by 0.2–2 pp. The difference signal
is physically present: 2600–5200 counts p95 per leg, 4–8 % of full stroke. It is simply **masked**.
In turbulence the legs already travel about three times as far, and a slow lean added to fast
turbulence content is perceptually hopeless.

And the pilot judged mostly in turbulence. The calm recording covers only takeoff and initial climb
— the flap-extension and approach cues, where the tilt channel had previously been unmistakable,
were never flown at this setting.

**Adopted anyway**, because it costs nothing measurable (no clipping, `sat_sl_acc` +0.2 pp, no
roughness reported) and is measurably larger in calm air. **The rig verdict behind this row is weak
and should not be cited as confirmation** — it establishes that 0.4 does no harm, not that it helps.
If a later stage wants to revisit the tilt channel, fly the calm-air flap/approach case first.

**Method note.** The sweep reported tilt-space p95 and the change looked uniform at +33 % everywhere.
Actuator space, which is what the pilot actually feels, told a completely different story per
segment (+23.6 % vs +1.2 %). **Report a candidate in the space the platform moves in, not only in
the space the parameter acts on.**

### Ground roughness — diagnosed 2026-08-30: the rumble effect is not physically renderable

The pilot has reported roughness on the ground twice, across settings that fixed roughness in the
air. Measured on `surgesway-a`, restricted to the 1793 ticks with `onground = 1` and
`groundspeed > 0.5 m/s` — whole-file metrics are useless here, the airborne portion drowns it out:

| `rumble_gain` | actuator excursion | `jerk_p95` | acceleration limiter active | velocity limiter |
|---|---|---|---|---|
| 0.0 (off) | 7887 | 3.3 M | **7.70 %** | 5.97 % |
| 0.45 | 7902 | 14.1 M (+327 %) | **84.77 %** | 15.11 % |
| **0.9 (shipped)** | 7879 | **14.5 M (+339 %)** | **92.14 %** | 32.07 % |

**The excursion is identical** (7879 vs 7887). The rumble adds no felt movement whatsoever; it
quadruples the jerk and pins the acceleration limiter across almost every ground tick. What the
pilot feels as "rumble" is the limiter clipping a sine it cannot follow.

**Why halving the gain did not help.** Peak acceleration is `(2πf)² · A` — it scales with the
*square of the frequency*, so amplitude is the weak lever and frequency is the strong one. Scale
factor taken from the replays themselves (least squares of `sp0..5` against `cmd_heave`; the six
legs give 304–586 counts/mm, and **the smallest was used, which is the assumption most favourable
to the rumble**): 330.7 counts/mm, so the 120 000 counts/s² limit is **363 mm/s² of heave**.

| rumble | demanded acceleration | vs. limit |
|---|---|---|
| **0.9 mm @ 12 Hz (shipped)** | 5116 mm/s² | **14.1×** |
| 2.0 mm @ 12 Hz (code default) | 11 370 mm/s² | 31.3× |
| 0.9 mm @ 3 Hz | 320 mm/s² | 0.9× |

Largest renderable amplitude by frequency: 12 Hz → **0.064 mm**; 8 Hz → 0.144 mm; 6 Hz → 0.255 mm;
4 Hz → 0.575 mm; 3 Hz → 1.021 mm. At the shipped 12 Hz the platform can render 64 µm. The effect
has never once produced a rumble on this hardware.

That also explains the frequency sweep, which looked paradoxical: lowering `rumble_freq_hz` to 3 Hz
still left 69 % acceleration clipping while *raising* velocity clipping 32 → 49 %. The two limiters
are cascaded — at 12 Hz the acceleration limiter crushes the signal so hard that little is left for
the velocity limiter to clip.

**The same defect applies to the touchdown bump**, which has not been separately investigated:
3.6 mm at 6 Hz demands 5117 mm/s², also **14×** the limit, against 0.255 mm renderable. The landing
thump is clipping too.

**Test, one knob: `rumble_gain = 0`.** If the ground roughness goes away, the diagnosis holds and
nothing of value was lost — the excursion numbers say the pilot cannot be giving up felt motion,
only clipping. If it does *not* go away, the cause is elsewhere and this analysis was a dead end;
the next suspect would be the washout responding to real runway bumps in `g_nrml`.

If the rumble is missed afterwards, the renderable corner is `rumble_freq_hz = 3` at
`rumble_gain = 0.9`. But 3 Hz is not a rumble — it is a slow wallow, and that is a separate
judgement to make after the null test, not folded into it.

### Ground roughness — flown 2026-08-30, **adopted: `rumble_gain = 0`**

Pilot: *"Ruppigkeit ist weg."* Recording `rumble0-motion-20260830-170250.csv` (landing first, then
takeoff), bit-exact against a config with `rumble_gain = 0` and 0.879 mm/deg off the 0.9 config, so
the setting was active.

Replayed at both values on those same cues, ground ticks only:

| `rumble_gain` | actuator excursion | `jerk_p95` | acceleration limiter |
|---|---|---|---|
| 0.9 | 34 960 | 11.8 M | **89.61 %** |
| **0** | **34 930** | **4.81 M (−59 %)** | **14.43 %** |

The prediction from the diagnosis held on an independent recording: the excursion is unchanged to
0.09 %, so nothing felt was given up, and the limiter engagement collapses. **The effect had never
rendered as a rumble on this hardware** — it was 14× over the acceleration limit and produced only
clipping.

**Where the campaign's original complaint now stands.** The pilot named ground roughness twice while
air roughness was being fixed; it survived every washout change because it was never a washout
problem. This is the fourth distinct cause found in one campaign, and the second one where the
symptom was a *limiter* rather than the parameter being tuned.

### Touchdown bump — measured 2026-08-30, **not changed, decision deferred**

Same recording, which contains two `onground` rising edges. Windowed 1.5 s from each edge:

| event | `touchdown_gain` | `jerk_p95` | acceleration limiter |
|---|---|---|---|
| firm landing, t = 133.1 s | 3.6 (shipped) | 17.5 M | 98.46 % |
| firm landing, t = 133.1 s | 0 | 14.9 M | **95.38 %** |
| gentle touch, t = 186.8 s | 3.6 (shipped) | 11.4 M | **71.64 %** |
| gentle touch, t = 186.8 s | 0 | 3.56 M | **0.00 %** |

Two separate things, and they need separating:

1. **On the firm landing the limiter is already pinned at 95 % without any effect at all.** That is
   the washout responding to the real vertical deceleration of a landing, and a 30 mm platform
   physically cannot render it. Clipping there is arguably the correct behaviour, not a defect.
2. **On the gentle touch the bump is the entire story** — 0.00 % clipping without it, 71.64 % with.
   The effect manufactures harshness on an event that should feel soft. Peak `eff_heave` is 3.04 mm,
   so the amplitude is delivered; it is the 6 Hz rate that cannot be, at 14× the acceleration limit
   against 0.255 mm renderable at that frequency.

**No change made at the time.** The pilot had not reported a problem with landings. Picked up later
the same day for the acceptance flight — candidates swept on the two real touchdowns above, gentle
touch only (the firm landing is 95.38 % saturated with the effect *disabled*, so nothing set here
changes it):

| gain / freq / tau | delivered | acceleration limiter | `jerk_p95` |
|---|---|---|---|
| 3.6 / 6 / 0.25 (shipped) | 3.01 mm | **71.64 %** | 11.4 M |
| 6.0 / 1.0 / 0.40 | 3.46 mm | 41.79 % | 9.9 M |
| 4.0 / 1.2 / 0.35 | 2.37 mm | 32.84 % | 10.0 M |
| **3.5 / 0.8 / 0.55** | **2.11 mm** | **10.45 %** | 4.79 M |
| 2.5 / 1.0 / 0.40 | 1.44 mm | 7.46 % | 4.26 M |
| 1.8 / 1.5 / 0.30 | 1.10 mm | 8.96 % | 5.06 M |
| effect disabled | 0 | 0.00 % | 3.56 M |

**Candidate `3.5 / 0.8 / 0.55`**: 70 % of the displacement the shipped setting delivers at a seventh
of the clipping. Note that "delivered" is well below `touchdown_gain` because the exponential
envelope decays before the sine reaches its first peak — at 0.8 Hz and τ = 0.55 the first peak lands
at ≈0.31 s, by which point the envelope is at 0.57. Fallback if it feels floaty: `2.5 / 1.0 / 0.4`.

This is no longer a thump; it is a settle over roughly a second, which is closer to what an oleo
strut actually does. Whether that reads as a landing is the rig's call.

`touchdown_freq_hz` accepts fractional values — `getDouble` in `MotionConfig.cpp` tries `double`
first and falls back to `int64_t`, so both `6` and `0.8` parse.

### Runway slab joints — new effect, built 2026-08-30, **not yet flown**

The pilot's idea, from the BFF Motion notes: drive ground texture off *distance* instead of a clock,
the way an unrealistically large wheel radius fakes concrete expansion joints. Then the rate follows
groundspeed rather than fighting the acceleration limit at a fixed 12 Hz.

**The pilot's refinement is what makes it work, and it corrects a worse design of mine.** My first
proposal was a bump that returns to zero. The pilot proposed instead that each joint *steps* — up at
one joint, down at the next, alternating. A bump costs four accelerate/decelerate phases in one
window; a step costs two, and the return trip is simply the next joint. Same 1 mm, 2.5× less time:

| | amplitude | duration at full budget |
|---|---|---|
| bump, out and back | 1 mm | 330 ms |
| **alternating step** | 1 mm | **132 ms** |

Shape: `x(τ) = from + Δ·(τ − sin(2πτ)/2π)`, `τ = t/T`. Velocity is zero at both ends, peak
acceleration is `2π|Δ|/T²`, so `T = √(2π|Δ| / budget)` is the fastest move that respects the budget.
**The effect is specified in acceleration and the displacement follows** — the opposite of the
rumble, which specified displacement and let the acceleration land wherever it landed (14× over).

Measured on the real ground roll in `rumble0-motion-20260830-170250.csv`, ground ticks only:

| | actuator excursion | `jerk_p95` | acceleration limiter | step delivered |
|---|---|---|---|---|
| no effect | 34 930 | 4.81 M | 14.43 % | — |
| **slab 10 m / 1 mm / 200** | 34 200 | **5.04 M (+5 %)** | **15.11 %** | **1.00 mm** |
| slab 10 m / 1 mm / 300 | 34 189 | 6.51 M | 38.93 % | 1.00 mm |
| old rumble 0.9 @ 12 Hz | 34 960 | 11.8 M | 89.61 % | — |

**A prediction of mine was wrong and the measurement caught it.** I claimed the design would never
clip. At a 300 mm/s² budget it clips on 39 % of ground ticks, because **the budget is shared** — the
washout is spending part of the same 363 mm/s² at the same time, which my sizing ignored. At 200 the
effect costs +0.7 pp of limiter engagement over having no effect at all, and still delivers the full
step. 200 is therefore the default.

Reach, at 10 m spacing and a 200 budget: the full 1 mm holds to ≈56 m/s (110 kn), past the Arrow's
rotation speed. Above that the step shrinks itself to fit rather than overrunning the budget, and a
joint arriving while the previous move is still running is skipped — interrupting mid-move would
restart a zero-start-velocity profile from a moving platform, which is a velocity step and exactly
what the limiter would then clip.

Off by default (`slab_spacing_m = 0`), five new state columns (66-column telemetry schema), six new
`test_effects` checks that assert the acceleration budget rather than "does it move". Every earlier
recording still verifies bit-exact against the config it was flown with.

**To fly it:** `slab_spacing_m = 10` in `configuration.toml`. **This one needs a rebuild on the PC.**
Judge taxi (distinct thuds), the takeoff roll (they should merge into a fine texture, not a buzz),
and whether anything is felt in the air — nothing should be.

#### First rig session, 2026-08-30 — step raised to 2 mm, and a defect found

Pilot, on `Slab1-motion-20260830-174216.csv` (1 mm, bit-exact against the shipped config, so the
effect ran exactly as built): *"Mit 1 mm war so gut wie nichts zu bemerken."* Then, unrecorded,
3 mm — *"sehr gut zu merken aber zu viel und fühlte sich seltsam an"* — and 2 mm: *"bei manchen
Geschwindigkeiten ganz gut, bei schnelleren geht es im allgemeinen Movement unter (muss nicht der
schlechteste Ausgang sein)."*

Measured on that recording's cues, replayed at each step size, ground ticks only, excluding
touchdown-active ticks (the limiter is saturated there, which inflates the comparison for reasons
unrelated to the slab). The share column is the slab's contribution as a fraction of what the
actuators were already doing:

| speed | step 1 mm | step 2 mm | step 3 mm |
|---|---|---|---|
| 0.5–5 m/s | 1.9 % | 3.8 % | 5.5 % |
| 10–15 m/s | 4.7 % | 9.4 % | 14.1 % |
| 20–30 m/s | 13.0 % | 17.7 % | 17.7 % |
| 30–45 m/s | 8.4 % | 13.8 % | 15.4 % |

**A defect, found while explaining "seltsam".** Joints were being silently dropped:

| step | joints expected | fired | skipped |
|---|---|---|---|
| 1 mm | 155 | 148 | 5 % |
| 2 mm | 155 | 134 | **14 %** |
| 3 mm | 155 | 127 | **18 %** |

The move's duration is fitted to `spacing / groundspeed` measured **at the moment the joint fires**,
but on a takeoff roll the aircraft is accelerating, so the next joint arrives sooner than that
estimate and gets skipped because the previous move is still running. A bigger step means a longer
move means more skips — which is a stuttering rhythm, and a plausible part of why 3 mm felt strange
rather than merely strong.

Fixed by fitting the move into 80 % of the estimated gap (`kSlabDuty`). That fires essentially every
joint at every step size, at the cost of a smaller step where the budget was already binding: above
30 m/s, 2 mm goes from 1.78 to 1.14 mm. Above ~30 m/s the step size setting stops mattering anyway —
the acceleration budget caps it near 1.1 mm whether it is set to 2 or 3.

A `test_effects` case now flies a simulated accelerating roll and asserts that ≥ 95 % of expected
joints fire; it was confirmed to fail with the margin removed.

**`slab_step_mm = 2` adopted as the candidate**, still unflown *with the skip fix in place* — the
2 mm the pilot judged was the stuttering version. **Needs another rebuild on the PC.** The fade at
speed is partly inherent: at 10 m spacing, joints arrive at 3–4.5 per second on the takeoff roll, so
distinct thuds necessarily blur into texture. Larger `slab_spacing_m` would keep them distinct on
the roll at the cost of being sparse while taxiing; that is a separate experiment.

### Acceptance flight, 2026-08-30 — **campaign closed, both remaining candidates adopted**

`abnahme-motion-20260830-185827.csv`, 66 columns, bit-exact against `configuration.toml`, so both
unflown candidates (slab joints at 2 mm, touchdown at 3.5 / 0.8 / 0.55) were genuinely active.

Pilot, before seeing any numbers:

> Beim Rollen die Beton-Kanten gefühlt, wobei man schon ungefähr wissen muss, auf was man "achten"
> muss. Bei größerer Geschwindigkeit faden die "Bumps" dann aus. Keine Bumps mehr sobald man
> abgehoben hat. Amplituden und Softness für Kurven und Steig/Sinkflug sind ausreichend und
> "glaubhaft". Cruise bei ruhigem Wetter geht sehr(!) ruhig. Das ist sehr positiv. Lande-Effekt ist
> in der jetzigen Version viel besser als vorher. Es ist tatsächlich ein Aufsetzen und kein
> Aufschlagen mehr. Sowohl bei weicher wie harter Landung.

**Saturation by phase, against the frozen baseline.** The baseline column is the comparable segment
from `baseline-metrics.md`, not the same flight, so treat it as an order-of-magnitude comparison:

| phase | heave clamp | rot clamp | tilt at limit | envelope | accel limiter | baseline heave |
|---|---|---|---|---|---|---|
| Anflug | 0.00 % | 0.00 % | 0.87 % | 0.00 % | 5.32 % | — |
| Landung 1 | 0.00 % | 0.00 % | 0.00 % | 0.00 % | 7.33 % | — |
| Start | 0.00 % | 0.00 % | 0.00 % | 0.00 % | 0.00 % | — |
| Steilkurven | 3.01 % | 1.17 % | 0.00 % | 0.00 % | 7.39 % | **79.66 %** |
| Steig/Sinkwechsel | 9.60 % | 0.00 % | 0.00 % | 0.00 % | 6.41 % | **88.27 %** |
| **Cruise** | **0.00 %** | 0.00 % | 0.00 % | 0.00 % | **0.00 %** | **19.92 %** |
| Alignment | 0.00 % | 0.00 % | 0.00 % | 0.00 % | 0.00 % | — |
| Final Approach | 0.00 % | 0.00 % | 0.00 % | 0.00 % | 0.00 % | — |
| Landung 2 + Rollen | 0.00 % | 0.00 % | 8.90 % | 0.00 % | 15.06 % | — |

Every clamp that was pinned at the start of this campaign is now essentially free. `sat_envelope` is
0.00 % in every phase, so nothing is stealing from anything else. The tilt channel, which sat pinned
at its limit for 56–63 % of the flight at the shipped settings, now reaches it only on approach
(0.87 %) and during the braking rollout (8.90 %) — which is where a sustained lean *should* be at
its limit. `climb_descent` remains the hardest case at 9.60 %, exactly as the design spec predicted.

**Slab joints on a real flight:** 1608 m of ground roll, 160 joints expected, **162 fired, none
skipped**, step held at the full 2.00 mm. The margin fix works on real data, not only in the test.

**One prediction of mine was wrong, and the pilot's report is what caught it.** I had written that
the touchdown setting could not matter on a firm landing, because the limiter is already pinned at
95 % with the effect disabled. It does matter, and the reason is that **a saturated limiter clips
the rate, not the accumulated position** — the effect still displaces the actuators by 1500–2600
counts. On the hard landing at t = 878.5 s:

| | peak contribution | when |
|---|---|---|
| old, 3.6 / 6 Hz / 0.25 | 2601 counts | **0.25 s after contact** |
| new, 3.5 / 0.8 / 0.55 | 1600 counts | **1.49 s after contact** |

The old setting dumped its whole contribution into the impact instant; the new one spreads it and
peaks over a second later. That is "ein Aufsetzen und kein Aufschlagen", measured. This is the
second time in this campaign I concluded "the limiter is saturated, so this cannot matter" and was
wrong — the first was claiming the slab effect would never clip.

**A measurement error of mine, worth recording because it produced exactly the hoped-for answer.**
The first version of this phase table reported **0.00 % saturation in every channel and every
phase** — a perfect result, and entirely fabricated. `heave_clamped` and `rot_*_clamped` are written
as `putI(..., flag ? 1 : 0)`; they are booleans, not values. Testing them for `>= 29.99` can only
ever return zero. The table above is the corrected one. Three of the campaign's eleven documented
traps were this same shape, and knowing that did not stop me walking into a fourth: **check what a
column contains before predicating on it, especially when the answer looks like success.**

### Parked ideas — measured or reasoned, deliberately not pursued

Not rejected, just not worth a rig session yet. Each one has its reasoning here so it does not have
to be re-derived.

- **`slab_spacing_m` 15–20 m.** At 10 m the joints arrive 3–4.5 per second on the takeoff roll, so
  distinct thuds necessarily blur into texture — which is what the pilot reported and also what real
  slab joints do. Wider spacing keeps them distinct at roll speed at the cost of being sparse while
  taxiing. Try only if the roll ends up feeling like a buzz rather than a surface.
- **`tilt_limit_deg = 9` at `tilt_rate_limit_dps = 3`.** The tilt channel is structurally maxed out
  at 7: the gain sweep showed everything above 0.4 buys clipping rather than cue, so more
  longitudinal/lateral information would have to come from a larger limit. The campaign established
  that roughness is governed by the *rate* limit and not the limit itself, so 9° at 3 °/s is
  plausible on paper. It is a large static lean and needs its own rig judgement — do not fold it
  into another change.
- **Surge/sway onset cues.** A feature, not a setting: `WashoutFilter::update` hard-zeroes
  `p.surge` and `p.sway`, so the platform runs 6DOF hardware in 4DOF and renders longitudinal and
  lateral force only as a sustained lean. Written up as its own ticket:
  `../superpowers/specs/2026-08-30-surge-sway-onset-cues-ticket.md`.

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

## Surge/sway onset channel

### First rig session, 2026-09-01 — sign confirmed, gain 0.5 is far too high, and a tool defect found

`surge-sway-motion-20260901-215605.csv`, 78 columns, 7229 rows, 173.3 s at ~41 fps. Flown with
`surge_gain = sway_gain = 0.5`, everything else as shipped. Contains a takeoff push, a braking
application and rudder input. Cue export committed as `reference/onset_events.csv.gz`; it replays
bit-identically to the full recording, so it is a valid input for future sweeps.

**Sign: confirmed, not guessed.** Bench jog first — `+surge` moves the platform forward (the pilot
pressed `+` six or seven times, then Reset, and watched the platform travel back). In the filter,
positive `g_axil` produces positive `surgePos_`, so the platform accelerates forward and presses the
pilot back into the seat, which is what forward acceleration is. The cross-check is the tilt channel,
already signed off on 2026-08-30: the same positive `g_axil` produces nose-up pitch through
`asin(tiltSurgeGain * surgeLpRaw_ / G)`, leaning the pilot back. Both channels push in the same
direction, so the onset sign stands as written. Pilot's verdict in flight: *"eher schwach, und somit
nicht einfach auszumachen, wenn sich die Platform gleichzeitig auch noch neigt. Es hat sich aber auf
jedenfall nicht 'falsch' angefühlt."*

**A defect in `washout_replay`, found by this recording.** The first `--verify` failed with a
residual of exactly 43 mm — `surge_limit_mm`. Cause: `runChain` composed its live pose from four DOF,
so replay emitted a constant zero on `live_surge`/`live_sway`. Task 6 had widened the comparison to
six DOF but not the source of the values being compared, and both reviews missed it because the
offending line sat outside the diff. Fixed (`a218af4`); the same recording then verifies **PASS at
1.9e-06 mm/deg**. That residual also confirms the flown gains were exactly 0.5/0.5 — a reconstruction
with any other value would not land on the floating-point floor.

**The cue is not weak, it is clipping.** At 0.5 it spends 5.98 % of ticks against `surge_limit_mm`
and 3.91 % against `sway_limit_mm`, both pinned at their exact limit values. A clipped transient
reads as mushy rather than strong — the same mechanism that ruined the 12 Hz rumble and the 6 Hz
touchdown. Gain sweep over this recording:

| `surge_gain` = `sway_gain` | peak surge | peak sway | surge clamped | `sat_envelope` |
|---|---|---|---|---|
| **off (cue-off baseline)** | — | — | — | **4.32 %** |
| 0.1 | 27.3 mm | 40.9 mm | 0.00 % | 4.30 % |
| 0.15 | 40.9 mm | 41.0 mm | 0.00 % | 4.40 % |
| 0.2 | 43.0 mm | 41.0 mm | 0.54 % | 4.52 % |
| 0.3 | 43.0 mm | 41.0 mm | 1.48 % | 5.49 % |
| 0.4 | 43.0 mm | 41.0 mm | 3.28 % | 6.38 % |
| 0.5 (flown) | 43.0 mm | 41.0 mm | 5.98 % | 7.15 % |

`sat_heave`, `sat_rot`, `sat_tilt_rate`, `wrms`, `band_ratio`, `lag_ms` and both rate metrics are
**identical** across every row: they are computed from `live_*`, and the onset channel does not touch
the heave or rotational paths. Only `sat_envelope` and `sat_sl_acc` (21.5 % → 27.8 % at gain 0.5) see
it, which is exactly why the envelope gate is the one that matters here.

**The channel is much stronger than the design assumed.** Leaky double integration with
`trans_vel_washout_tau = trans_pos_washout_tau = 0.25` delivers roughly 62 mm per m/s² of high-passed
acceleration. At gain 0.1 sway is already at its 41 mm limit.

**Two things this session changes about the campaign's assumptions:**

1. **The cue-off `sat_envelope` baseline on this flight is 4.32 %**, against 0.00–0.34 % across the
   seven reference recordings the limits were derived from. This flight is far more demanding than
   the corpus, so the p1-sized limits (43 / 41 mm) are too generous for it. The gate — `sat_envelope`
   must not exceed its own cue-off baseline — is only met at gain 0.1; 0.15 misses it by 0.08 pp.
2. **The first rig candidate is `surge_gain = sway_gain = 0.1`**, not the ~0.4 the
   `configuration.toml` comment suggests. That comment reasoned from the tilt/onset gain ratio and
   did not account for the translational channel's own amplitude, which this session measured for the
   first time. Expect a cleanly shaped 27 mm impulse to read as *more* distinct than today's clipped
   43 mm one; if it then feels too small, the next lever is shortening `trans_*_washout_tau`, not
   raising the gain.

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
