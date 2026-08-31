# Ticket: measure the acceleration ceiling instead of assuming it

**Status:** open. This is the one lever that would change what motion cues are possible on this
platform. Written 2026-09-01.

**Type:** measurement first, then a safety-relevant tuning decision. Do not raise the limit before
the measurement exists.

## The number nobody derived

`SafetyConfig.h`:

```cpp
double maxVelocity     = 30000.0;   // counts / second
double maxAcceleration = 120000.0;  // counts / second^2
```

The velocity limit has a documented derivation — `configuration.toml` says "keep < ~32000: actor-side
speed cap is 0.5*stroke/s ≈ 32640 counts/s", i.e. it is set just under a real property of the
actuator path.

**The acceleration limit has none.** `120000` first appears in
`docs/superpowers/plans/2026-07-19-motion-provider-phase4-serial-safety.md:306` as a plan-time
default and has never been questioned since. Every experiment recorded in
`docs/motion-tuning/tuning-log.md` moved it **downward** — 80000, 60000, 42000 (tracked cleanly),
12600 (lagged) — as a jerk-shaping lever against harsh reversals. Nobody has ever tried raising it,
and nobody knows whether the actuators would follow if it were raised.

## What that number costs today

At the measured 330.7 counts/mm, 120 000 counts/s² is **≈363 mm/s² of platform acceleration**, and
that is the hard ceiling on every oscillating or pulsed cue, because peak acceleration goes as
`(2πf)²·A`:

| Frequency | Largest renderable amplitude | In actuator counts |
|---|---|---|
| 12 Hz | 0.064 mm | 21 |
| 8 Hz | 0.144 mm | 48 |
| 7 Hz | 0.188 mm | 62 |
| 6 Hz | 0.255 mm | 84 |
| 4 Hz | 0.575 mm | 190 |
| 3 Hz | 1.021 mm | 338 |
| 40 Hz (engine order) | 0.0057 mm | **1.9** |

Two shipped effects were written without checking this and were both 14× over: the 12 Hz ground
rumble (now `rumble_gain = 0`, replaced by the slab-joint step effect) and the 6 Hz touchdown bump.
Neither produced the motion it was named for — the limiter turned each into a slew-limited zigzag,
felt as harshness.

Two effects remain unbuilt behind the same wall (`engine_gain`, `buffet_gain`, both reserved
placeholders with no code — `EffectsLayer.cpp:126`):

- **Engine vibration is unreachable at any limit that is plausible.** A piston engine's dominant
  order sits around 40 Hz, above the 30 Hz Nyquist of the 60 Hz command stream, and at 1.9 actuator
  counts it is under the quantisation of the drive. This ticket will not rescue it — see
  `2026-09-01-yoke-haptics-ticket.md` for the approach that could.
- **Buffet is already renderable** at 6–8 Hz within today's budget — it needs an implementation
  sized in acceleration, not a bigger budget. Its trigger input (`MotionCues::alphaDeg`) is already
  collected and recorded.

So the payoff of a higher ceiling is not "the two missing effects". It is that every cue in the
0.5–8 Hz band — touchdown, slab joints, buffet, and the onset transients of the new surge/sway
channel — gets more amplitude before clipping. Doubling the ceiling doubles the renderable amplitude
at a given frequency, or buys √2 in frequency at a given amplitude.

## What to measure

The real question is what the **actuators and the Kangaroo** can follow, which is a bench
measurement, not a flight one.

The instrument exists: `MotionActor`'s test bench (`MOTION_TESTBENCH=1`,
`env:nanoatmega328new_testbench`), triggered over CAN from the gateway console and dumped over CAN
afterwards so that measuring does not perturb the link being measured. Strategies 1–4 stream step
commands with the production and the exact speed algorithms, with and without Kangaroo streaming;
`sampleMode 3` interleaves `getP()` position samples into a timing run.

The shape of the experiment:

1. Command a series of step reversals of increasing demanded acceleration, well below the current
   ceiling to start.
2. Sample commanded position against `getP()` actual position.
3. Find the acceleration at which measured position stops tracking the commanded profile — the point
   where the mechanism, not the configuration, becomes the limit.
4. Repeat at both ends and in the middle of the stroke: a linkage's effective inertia is not constant
   over its travel, and the six legs measured 304–586 counts/mm, so the worst leg governs.

Report the tracking limit in counts/s² and, converted through the **smallest** counts/mm figure, in
mm/s² — the same conservative convention `docs/motion-tuning/tuning-log.md` already uses.

## What the answer does not settle

A measured mechanical limit is an upper bound, not a new setting. Three things stand between the
measurement and a config change:

- **The heave campaign found that lowering this value smooths reversals.** Whatever the mechanism can
  do, more acceleration may reintroduce the harshness that campaign removed. The tuning question
  ("what feels right") is separate from the capability question ("what is possible") and needs its
  own rig session and acceptance flight.
- **It is the safety limiter.** `max_acceleration_cps2` is the last thing between a cueing bug and
  the mechanism. A raise should keep a documented margin below the measured tracking limit, not sit
  at it.
- **Wear and thermals.** A bench run of a few minutes says nothing about what sustained operation at
  a higher acceleration does to the linkages or the drives. Worth a deliberate look before adopting.

## Suggested sequencing

1. Measure the tracking limit (bench, no flight).
2. Record it in `docs/motion-tuning/baseline-metrics.md` next to the counts/mm figures, whatever it
   turns out to be — even "120 000 is already at the limit" is a valuable answer that closes this
   ticket permanently.
3. Only if there is real headroom: build the buffet effect first, sized in acceleration against the
   **current** budget. It is renderable today and is the cheapest way to find out whether the band
   is worth investing in at all.
4. A ceiling change, if any, gets its own campaign with the existing replay/metrics harness and an
   acceptance flight — the same shape as the 2026-08-30 heave campaign.

## Related

- `MotionProviderPlugin/CLAUDE.md` — the acceleration budget section and the `(2πf)²·A` table.
- `docs/motion-tuning/tuning-log.md` — the two 14×-over effects, and every downward experiment on
  this value.
- `MotionActor/src/TestBench.h` — strategies and sample modes.
- `docs/superpowers/specs/2026-09-01-kangaroo-baud-migration-ticket.md` — the companion; that one
  buys transport margin, this one buys cue amplitude.
