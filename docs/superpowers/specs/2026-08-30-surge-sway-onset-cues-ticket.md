# Ticket: surge/sway onset cues

**Status:** not started. Written 2026-08-30 as a follow-up to the heave-tuning campaign, to be picked
up in its own session.

**Type:** feature, not tuning. Nothing in `configuration.toml` can produce this today.

## What is missing

`WashoutFilter::update()` ends by hard-zeroing two of the six degrees of freedom:

```cpp
Pose p;
p.surge = 0.0f;
p.sway  = 0.0f;
p.heave = static_cast<float>(out[0]);
```

The platform is 6DOF hardware driven in 4DOF by design. Longitudinal and lateral specific force
reach the pilot **only** through tilt coordination — leaning the platform so gravity substitutes for
the acceleration. `StewartKinematics` and `Pose` already carry surge and sway end to end, and
`clampToReachable` already scales all six DOF together, so the plumbing exists; only the washout
declines to use it.

## Why it is worth doing

Tilt coordination is a *sustained* cue by construction: it is low-passed (`tilt_lp_tau = 1.5`) and
rate-limited (`tilt_rate_limit_dps = 3`, deliberately, to stay under the ≈3 °/s rotation-detection
threshold — see `docs/motion-tuning/tuning-log.md`). Those two constraints are exactly what makes it
unable to render an **onset**: the first half-second of a takeoff push, a braking application, or a
crosswind gust arrives, if at all, as a slow lean that starts after the event.

The classical split is that translation renders the onset and tilt takes over for the sustained
part, with the handover hidden by the washout. That is the standard structure this filter is missing.

The 2026-08-30 campaign also established that the tilt channel is now **structurally maxed out**:
`tilt_limit_deg = 7` with the gains at 0.4, and raising either further buys clipping rather than
cue. So the tilt path has no headroom left to absorb this; translation is the only way to add
longitudinal and lateral information.

## What has to be decided or measured first

1. **The translational envelope is unknown.** Nobody has measured how much surge and sway this
   geometry actually has at the pose the platform normally sits in — the campaign only ever
   characterised heave (±30 mm) and the angular limits. `clampToReachable` scales *all six* DOF by a
   single bisection factor, so a surge demand that does not fit will silently shrink heave and tilt
   as well. **Measure the reachable surge/sway range before designing amplitudes**, and treat the
   coupling as the main risk: this feature can degrade cues that currently work.
2. **The acceleration budget applies here too.** `SafetyLimiter` allows ≈363 mm/s² of platform
   motion (120 000 counts/s² at 330.7 counts/mm); an onset cue is by definition a fast transient and
   will run into it. Size it in acceleration first — see the budget section in
   `MotionProviderPlugin/CLAUDE.md`. Two effects in this codebase were written without doing that
   and both were 14× over.
3. **The handover to tilt has to be explicit.** Surge high-passed for the onset, tilt low-passed for
   the sustained part, crossing over so the sum does not double-count the same acceleration. The
   existing `tilt_lp_tau` is one half of that pair and the new high-pass constant is the other; they
   are not independent.
4. **Whether it is wanted at all.** The pilot has not asked for this. It came out of the campaign as
   an observation about the filter's structure. Confirm the goal before building.

## How to verify it

The harness from the campaign covers this without extension: record on the rig, replay offline with
`washout_replay`, and gate on `washout_metrics.py`. Three specifics:

- `--verify` must pass bit-exact on a recording made with the feature enabled, which means the new
  filter state needs telemetry columns exactly as `EffectsLayer`'s slab state did (see the 66-column
  schema and `test_telemetry`'s exact-count assertion).
- Watch `sat_envelope`, which is currently 0.00 % everywhere. **If it becomes non-zero, surge and
  sway are stealing from heave and tilt** — that is the failure mode to catch, and it is invisible
  in any per-channel metric.
- Judge it at the rig on takeoff push, braking, and crosswind — and specifically on whether the
  handover from translation to tilt is felt as one cue or two.

## Related

- `docs/motion-tuning/README.md` — the recording/replay/metrics harness and how to use it.
- `docs/motion-tuning/tuning-log.md` — why the tilt channel is at its limit; the acceleration budget
  and the two effects that ignored it.
- `MotionProviderPlugin/CLAUDE.md` — the budget, and the per-board file conventions.
