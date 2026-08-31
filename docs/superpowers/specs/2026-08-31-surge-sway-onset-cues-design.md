# Surge/sway onset cues — design

**Status:** designed 2026-08-31, not implemented. Supersedes the ticket
`2026-08-30-surge-sway-onset-cues-ticket.md`, which stays as the problem statement.

**Scope:** through a signed-off acceptance flight, the same end state the 2026-08-30 heave campaign
reached — code, offline gates, rig sessions, adopted values in `configuration.toml`, a
`tuning-log.md` entry.

## The problem

`WashoutFilter::update()` hard-zeroes surge and sway. Longitudinal and lateral specific force reach
the pilot only through tilt coordination, which is low-passed (`tilt_lp_tau = 1.5`) and rate-limited
(`tilt_rate_limit_dps = 3`, deliberately below the ≈3 °/s rotation-detection threshold). Those two
properties are what make it a *sustained* cue — and what make it unable to render an **onset**. The
first half-second of a takeoff push, a braking application or a crosswind gust arrives as a slow
lean that starts after the event, or not at all.

The tilt channel has no headroom left to fix this: the 2026-08-30 campaign took it to
`tilt_limit_deg = 7` at gains 0.4 and found that raising either further buys clipping rather than
cue. Translation is the only remaining path for longitudinal and lateral information.

The classical structure is a split: translation renders the onset, tilt takes over for the sustained
part, and the washout hides the handover. That split is what this design adds.

## Chosen approach: complementary split

Three structures were considered.

- **A, complementary split (chosen).** The tilt channel already computes a low-passed horizontal
  acceleration. The onset is exactly its complement: `aHp = a − aLp`. One crossover constant serves
  both halves, so `LP + HP = 1` by construction and the same acceleration cannot be counted twice.
- **B, an independent high-pass mirroring the heave channel.** Familiar shape, independently tunable,
  but the high-pass constant and `tilt_lp_tau` become two free constants that can overlap — and in
  the overlap band the same acceleration drives both tilt and translation. That is precisely the
  double-count the ticket warns about; B would have to *measure* its absence rather than guarantee it.
- **C, tilt-error-driven handover.** Translation carries while the tilt rate limiter is catching up
  and fades as tilt reaches its target. Theoretically the cleanest "one cue, not two" handover, at
  the cost of feedback between two channels, more state, and a cue whose behaviour depends on the
  rate limiter — harder to replay and harder to attribute in a rig verdict.

A is chosen: it removes the double-count risk structurally instead of by tuning, costs one constant
fewer, and leaves amplitude fully controllable through its own gain and limit. C stays available as
a later addition if Stage 4 finds the handover is felt as two cues; A does not foreclose it.

## Stage 0: measure the envelope first

Nobody has measured how much surge and sway this geometry actually has. `clampToReachable` scales
**all six DOF** by one bisection factor, so an unreachable surge demand silently shrinks heave and
tilt as well. This feature can degrade cues that currently work, and that is its main risk.

New host tool `MotionProviderPlugin/tools/envelope_probe.cpp`, linking the real
`StewartKinematics.cpp` (same rule as `washout_replay`: no second implementation of the geometry).
It walks a grid and reports where `solve()`'s `allReachable` flips:

- **Bare:** max |surge| and |sway| at the home pose. An upper bound, never available in flight.
- **In the corner:** the same, with `heave = ±30 mm`, `roll = ±14°`, `pitch = ±14°`, `yaw = ±7°`
  applied concurrently — the sum of `tilt_limit_deg = 7` and `rot_limit_deg = 7` that the existing
  channels are already allowed to occupy on one axis. **This** number, with margin, becomes
  `surge_limit_mm` / `sway_limit_mm`.

Results go into `docs/motion-tuning/baseline-metrics.md` as a table. Stage 1 does not start without
them.

### The acceleration budget does not bind here

`SafetyLimiter` allows ≈363 mm/s² (120 000 counts/s² at 330.7 counts/mm). Peak acceleration of a cue
at the crossover frequency is `A/τ²`. With `tilt_lp_tau = 1.5 s` the corner is 0.106 Hz, so a 20 mm
cue peaks at **8.9 mm/s² — a factor of 40 under the ceiling.** Unlike the touchdown and rumble
effects, which were both 14× over, the binding constraint here is geometry, not the budget.

The budget becomes binding again only when the effective time constant drops below `sqrt(A/363)`
≈ 0.24 s at 20 mm. Any future change that shortens the crossover past that has to re-check this.

## Filter structure

The tilt path today low-passes the **gained** signal. For independent gains it moves to the raw
signal; a one-pole low-pass is linear, so `LP(k·x) ≡ k·LP(x)` and the tilt behaviour is unchanged:

```cpp
const double aRawX = c.surgeG * G;                 // raw, ungained
surgeLpRaw_ += lpAlpha(dt, cfg_.tiltLpTau) * (aRawX - surgeLpRaw_);

// Tilt: same behaviour as before, gain applied outside the filter
tgtPitch = asin(clampd(cfg_.tiltSurgeGain * surgeLpRaw_ / G, -1.0, 1.0)) * kRad2Deg;

// Onset: exactly the complement, with its own gain
const double aHpX = cfg_.surgeGain * (aRawX - surgeLpRaw_);
surgeVel_ = surgeVel_ * leak(dt, cfg_.transVelWashoutTau) + aHpX * dt;
surgePos_ = surgePos_ * leak(dt, cfg_.transPosWashoutTau) + surgeVel_ * dt * 1000.0;
// clamp writes back into the state -- same windup semantics as heave, deliberately
```

Sway is the same on the roll axis.

New state: `surgeLpRaw_`, `swayLpRaw_` (replacing `surgeLp_`/`swayLp_`), `surgeVel_`, `surgePos_`,
`swayVel_`, `swayPos_`. All cleared in `reset()`.

Three consequences this drags along:

- **Output smoothing becomes six wide.** `sm1_[4]`/`sm2_[4]` → `[6]`, ordered
  `{surge, sway, heave, roll, pitch, yaw}`. The "surge/sway are always 0 here" comment goes.
- **`k·LP(x)` is mathematically identical but not bit-identical in floating point.** If the `washout`
  suite carries an exact assertion on the tilt axis it will fail here — that is this line, not a
  behaviour change. Gate: the suite stays green, with a tolerance instead of equality if needed.
- **The sign is open and will not be guessed.** Whether platform `+X` points forward and whether
  `g_axil > 0` is forward acceleration is settled in Stage 1 by a bench jog against the existing tilt
  axis (nose-up on forward acceleration is the reference), not derived from the geometry on paper.

## Configuration

New keys under `[washout]`:

```toml
surge_gain = 0                 # OFF as shipped
sway_gain = 0
trans_vel_washout_tau = 0.25   # shared by surge and sway
trans_pos_washout_tau = 0.25
surge_limit_mm = <from Stage 0>
sway_limit_mm  = <from Stage 0>
```

No new crossover key: `tilt_lp_tau` is both halves. That is the point of approach A.

One shared washout pair for both horizontal axes rather than two — they are the same physical
channel, and every extra constant is rig time. Gain and limit stay per-axis, because the geometry is
not symmetric in X and Y (`base_angle_deg` vs `anchor_angle_deg`) and takeoff push and crosswind may
be wanted at different strengths.

**Gains default to 0.** Through Stages 1–3 the shipped `configuration.toml` therefore stays
numerically identical to the 2026-08-30 acceptance flight. Adopted values are written in Stage 5,
each with a rig verdict behind it, like every other value in that file. `MotionConfig.cpp` needs both
parsing and writeback, or the keys vanish on the next seed.

## Telemetry

`WashoutTrace` gains eight fields (`surgeAHp`, `surgeVel`, `surgePosRaw`, `surgeClamped` and the sway
equivalents), plus `live_surge`, `live_sway`, `cmd_surge`, `cmd_sway`. Twelve new columns, 66 → 78.

**All of them are appended, not inserted.** The cue export in `docs/motion-tuning/README.md` §2 is a
position-based `cut -d, -f2,4-16,58-61`. Appending leaves it valid; inserting breaks it silently, and
a silently wrong cue export is the kind of defect that surfaces three replays later. The cost is that
column order no longer reads like the pipeline. Accepted.

`eff_surge`/`eff_sway` are **not** added: `EffectsLayer` produces no horizontal component. The
`eff_*` group stays four wide while `live_*`/`cmd_*` become six — the asymmetry is explained in the
header comment rather than papered over with zero columns.

`test_telemetry`'s exact column-count assertion moves with it.

## Replay and `--verify`

- `washout_replay`'s verify comparison covers `live_heave/roll/pitch/yaw` today. It is extended to
  all six. Without that, `--verify` does not check the new channel at all and still reports PASS —
  the most dangerous outcome this tool can have.
- `kWashoutKeys` in `washout_replay.cpp` gains the six new keys, or `--set washout.surge_gain=...`
  is refused with exit code 2 and nothing can be swept.
- The 20-bucket monotonicity guard and the warm-up window need no change: the new states are leaky
  integrators like heave, so their initial condition decays.
- Gate: `--verify` PASS on a **fresh** recording made with the feature enabled. Older recordings keep
  their known status.

## Metrics

Two new columns in `washout_metrics.py`: `sat_surge` and `sat_sway`, defined like `sat_heave` (% of
unpaused ticks with the clamp engaged), under the same abort-on-missing-column rule — both could
manufacture a favourable value by being absent, so neither gets a default.

The decisive gate is an existing one: **`sat_envelope` must stay 0.00 %.** If it does not, surge and
sway are stealing from heave and tilt, and no per-channel metric shows it. Alongside it, unchanged:
`lag_ms ≤ baseline + 15 ms`, and `band_ratio` must not rise.

No `wrms`/`lag_ms` analogues for the horizontal axes. Both are defined on `g_nrml`/`live_heave` and a
heave band; a surge counterpart would be a new, unvalidated metric with no baseline, and Stage 4
carries the verdict. Add one only when a concrete question needs it.

## Tests

`washout` suite, four new cases:

- **Complementarity.** Step `g_axil`, then check that `surgeLpRaw_ + aHp/surgeGain` reconstructs the
  raw input acceleration at every tick — the low-pass the tilt path consumes plus the high-pass the
  translation path consumes, before either channel's own gain. This is the test that pins the
  no-double-count property permanently.
- **Zero-gain regression.** With `surge_gain = sway_gain = 0`, `p.surge == 0 && p.sway == 0` and
  heave/roll/pitch/yaw match a reference sequence taken before the restructure. Proves the shipped
  configuration is untouched.
- **Clamp.** Sustained acceleration parks `surgePos_` on `surge_limit_mm` with `surgeClamped` set and
  the state clamped with it.
- **`reset()`** clears all six new states.

`config` suite: the six keys parse, defaults hold, round-trip through writeback. `telemetry` suite:
column count and names. Eleven suites stay eleven; all green is a precondition for every stage.

## Stages

| Stage | Content | Done when |
|---|---|---|
| 0 | `envelope_probe`; envelope measured bare and in the corner | numbers in `baseline-metrics.md`, limits chosen |
| 1 | filter + config + tests, feature off; sign settled by bench jog | suites green, `configuration.toml` numerically unchanged |
| 2 | telemetry columns, replay keys, six-DOF verify, metric columns | `--verify` PASS on a fresh recording with the feature on |
| 3 | offline sweeps over `cruise_calm`, `ground_takeoff`, `approach_landing` | a candidate; `sat_envelope` 0.00 %, `lag_ms` inside the gate |
| 4 | rig: takeoff push, braking, crosswind. The question: is the translation→tilt handover felt as **one** cue | verdict in `tuning-log.md`, back to Stage 3 if not |
| 5 | acceptance flight, values adopted, `tuning-log.md`, `CLAUDE.md`, ticket closed | recording archived as on 2026-08-30 |

Each stage is separately committable and leaves the rig flyable.

## Risks

1. **The envelope is too small.** If Stage 0 reports only a few millimetres in the corner, the cue is
   too small to feel at 0.1 Hz and the feature dies in Stage 0 instead of after three rig sessions.
   That is why Stage 0 is a stage of its own.
2. **Envelope coupling eats cues that work.** Capped by the hard per-axis limits, watched by
   `sat_envelope`. If it does not stay at zero in Stage 3, the limits come down — not the gate.
3. **The handover is felt as two cues.** No offline criterion catches this; only Stage 4. The
   fallback is approach C, which approach A does not foreclose.
4. **Rig time.** Three flight phases, two axes, one campaign — Stages 4 and 5 are the same order of
   effort as the heave campaign.

## Related

- `docs/superpowers/specs/2026-08-30-surge-sway-onset-cues-ticket.md` — the problem statement.
- `docs/motion-tuning/README.md` — the recording/replay/metrics harness.
- `docs/motion-tuning/tuning-log.md` — why the tilt channel is at its limit; the acceleration budget.
- `MotionProviderPlugin/CLAUDE.md` — the budget and the filter's structural notes.
