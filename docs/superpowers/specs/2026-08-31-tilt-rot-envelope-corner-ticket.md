# Ticket: the tilt + rotational envelope corner

**Status:** open, **optional — nice to have**. Nothing depends on it. Written 2026-08-31 as a side
finding of the surge/sway onset campaign
(`2026-08-31-surge-sway-onset-cues-design.md`), which measured the platform's reachable envelope for
the first time.

**Type:** correctness of intent, not of behaviour. The platform does the right thing today; the
configuration just promises more angle than the mechanism has, and `clampToReachable` quietly makes
up the difference.

## The finding

`tilt_limit_deg` and `rot_limit_deg` are two separate channels that add on the same axis:

```cpp
out[3] = tiltRoll_  + rollAngle_;   // each clamped to ±7 -> ±14 possible
out[4] = tiltPitch_ + pitchAngle_;
```

Both were tuned and signed off in isolation during the 2026-08-30 campaign. Their **sum** was never
checked against the geometry, and the sum does not exist: the platform cannot hold 14° of roll and
14° of pitch at once. It cannot hold 7° and 7° either.

The reachable region is not a box. Measured 2026-08-31 with `tools/envelope_probe --pose` on the
shipped `[geometry]`:

| Pose (heave mm, roll°, pitch°, yaw°) | Reachable |
|---|---|
| roll alone | up to ≈9.5° |
| pitch alone | up to ≈15° |
| yaw alone | ≥10° |
| heave +30 alone | yes |
| roll = pitch, symmetric diagonal | **6.62°** |
| 0, 7, 7, 0 | no — just barely |
| 0, 6, 6, 7 | no |
| 0, 4, 4, 7 | yes |
| +30, 4, 4, 0 | no |
| +30, 3, 3, 0 | yes |
| −30, 6, 6, 7 | yes |

Roll costs more than pitch, heave upward costs more than downward, and yaw eats into the same
budget. The 6.62° figure applies only to the symmetric diagonal with everything else at zero.

## Why it is benign today

`reach_scale` never falls below **0.9968** across 46,878 ticks of all seven reference recordings —
0.3 % attenuation, occurring in 0.34 % of the ticks of `steep_turns`, the most aggressive one, and
in 0.00 % of the other six. `clampToReachable` scales all six DOF by one bisection factor, so the
only real artefact is that heave is briefly attenuated by that same 0.3 % during a roll/pitch corner
it had no part in. Not perceptible.

## Why "just lower both limits to 6.62" is the wrong fix

Two reasons, and the second is the expensive one.

1. **6.62° is the per-axis SUM of both channels, not one channel's value.** A guaranteed-reachable
   box would need roughly `tilt_limit_deg = rot_limit_deg = 3.3`, and with heave and yaw in play at
   the same time, closer to 1.5. That is below the pre-2026-08-30 values (3 + 3) — a step back
   behind the change that made the platform noticeably smoother.
2. **A box sized on the diagonal punishes the common case.** Almost always only one axis is large: a
   turn rolls, a flare pitches. Single-axis travel is 9.5° and 15°. Clamping every axis to the worst
   corner throws that away permanently to protect a corner that occurs 0.34 % of the time.

## Options

1. **Do nothing** — recommended, and the reason this ticket is optional. The finding is documented in
   `docs/motion-tuning/baseline-metrics.md`, `sat_envelope` measures it per reference file, and the
   scaler degrades gracefully. Cost: none.
2. **An elliptical bound instead of a box.** After summing tilt and rotation, check whether
   `sqrt(roll² + pitch²)` exceeds the measured diagonal and, only then, pull both back
   proportionally. Around ten lines in `WashoutFilter`. Keeps the single-axis 9.5°/15°, removes only
   the corner, and takes the reduction out of the channels that caused it instead of dragging heave
   along. This is the option to pick if the ticket is ever picked up.
3. **Priority scaling in `blendedCommand`** — on an unreachable pose, back the rotational channel off
   first and spare tilt and heave. More effective, but it adds a second place where poses are
   modified; the surge/sway design deliberately avoided exactly that complexity.
4. **Lower the limits to a safe box.** Largest cue loss for the smallest gain. Not recommended.

## If it is picked up, what it would take

A small campaign of its own, using the harness that already exists:

- Measure the diagonal per sign combination with `tools/envelope_probe --pose` (the tool already
  does this; only the sweep is missing). The region is asymmetric — `+heave` and roll are the
  expensive directions — so one number will not cover all four quadrants.
- Implement option 2 behind its own config key, defaulting to inert, the same way the onset channel
  shipped disabled.
- Replay all seven reference recordings and show `sat_envelope` falling to 0.00 % on `steep_turns`
  **without** `rot_rate_p95` or `tilt_rate_p95` rising — an ellipse that quiets the metric by simply
  removing cue would be a regression wearing a green gate.
- No rig time strictly required: the change is only active in a condition that measurably occurs in
  0.34 % of one recording. A rig session would be about confirming nothing was lost, not about
  tuning.

## Related

- `docs/motion-tuning/baseline-metrics.md` — the envelope measurements and the `sat_envelope`
  cue-off baselines.
- `docs/superpowers/specs/2026-08-31-surge-sway-onset-cues-design.md` — the campaign this came out
  of, and the per-axis-limit approach chosen there for surge and sway.
- `MotionProviderPlugin/CLAUDE.md` — the note that `clampToReachable` scales all six DOF together.
