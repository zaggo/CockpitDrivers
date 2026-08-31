# Ticket: engine and buffet as yoke haptics, not platform motion

**Status:** open, **optional — nice to have**. An idea captured so it is not lost, not a design.
Written 2026-09-01, out of the discussion of why `engine_gain` and `buffet_gain` were never built.

**Type:** new subsystem. If it is ever picked up it needs a proper brainstorm → spec → plan pass,
not this file.

## The idea

Render engine vibration — and, more interestingly, stall buffet — with a small transducer on the
control yoke instead of with the motion platform.

## Why it is the right instinct

The platform is a mass problem. To produce 40 Hz it has to accelerate the frame, the seat and the
pilot, which is why the acceleration budget leaves **0.0057 mm — under two actuator counts** at that
frequency (see `2026-09-01-acceleration-ceiling-ticket.md`). A yoke transducer moves grams.

And it is not a worse approximation, it is a better one. In a real aircraft engine vibration reaches
the pilot through the airframe into the seat and the controls, not as whole-body translation. Stall
buffet is felt most strongly **in the yoke**, because the control surfaces sit in the separated flow.
Putting these two cues at the contact point is closer to the real percept than putting them under
the whole rig, at a fraction of the energy.

## Frequencies worth hitting

For the campaign's reference aircraft, the Piper Arrow III (vFlyteAir PA28-201R, Lycoming IO-360,
four cylinders, two-blade constant-speed prop, direct drive):

| Order | Formula | At 2400 RPM |
|---|---|---|
| Crankshaft, 1st | `RPM / 60` | 40 Hz |
| Firing (4-cyl, 4-stroke) | `RPM / 60 × cyl / 2` | 80 Hz |
| Propeller blade passing | `RPM / 60 × blades` | 80 Hz |

Buffet is not a tone: narrowband noise, roughly 7–15 Hz, is the right shape.

Both bands are trivial for a transducer and impossible for the platform.

## Actuator choice is the whole question

"Controllable" rules out the obvious cheap part:

- **ERM** (eccentric rotating mass, the phone-buzzer type) — amplitude and frequency are the same
  knob: PWM sets the speed, and the speed sets both. It cannot follow RPM without also getting
  louder, and it has 50–100 ms spin-up lag. Cheap, and precisely not controllable.
- **LRA** — amplitude controllable, frequency locked to its resonance (typically 175–235 Hz). Wrong
  band.
- **Voice coil / tactile transducer**, driven from an audio amplifier with a synthesised waveform —
  frequency and amplitude independent, full waveform control. This is the one that does what the
  idea needs.

## Two ways to drive it

1. **Sound card.** The plugin synthesises the waveform, audio out → small amplifier → transducer.
   No firmware, no CAN ID, no board. An afternoon to the first thing you can feel.
2. **CAN node.** A small board, PWM or DAC into an amplifier, fed by a new CAN message carrying
   frequency and amplitude. Fits the repo's conventions (`CanMessageId.h` as the single source of
   truth, the per-board file layout in the root `CLAUDE.md`), and the DCU could drive it rather than
   only X-Plane. More work.

Prototype with (1). It answers the only question that matters — whether it feels right — before any
hardware is committed to.

## The input datarefs already exist

`MotionCues` already collects `engineRpm` (`sim/cockpit2/engine/indicators/engine_speed_rpm[0]`) and
`alphaDeg` (`sim/flightmodel/position/alpha`), and both are already written into the telemetry CSV.
Whatever drives this can read them from the same place the effects layer would have.

## Consequence for the existing placeholders

`engine_gain` and `buffet_gain` currently sit in `[effects]`, which is platform motion, as reserved
fields with no code behind them (`EffectsLayer.cpp:126`). If these two cues move to haptics, the two
keys should leave that struct rather than linger as implied platform capability — either deleted, or
moved into a section owned by whatever subsystem ends up rendering them.

That would take both cues off the platform's acceleration budget entirely, which is worth more than
the engine effect alone: it settles the two effects that have been open since Phase 3, and it settles
them in the place where the platform was never going to win.

## Open questions before anyone designs this

Three things that shape the design and cannot be guessed:

- **How is the yoke built** — return springs, force feedback, or free? Added mass on a moving control
  changes its feel and brings its own resonance.
- **Cable routing to a moving column** — strain relief and flex cycles.
- **Is there a free audio output** on the sim PC, or is it already spoken for?

A seat transducer is easier to mount (nothing moves) and works for engine roughness, but it is the
weaker choice for buffet, which is the more valuable half of this ticket.

## Related

- `docs/superpowers/specs/2026-09-01-acceleration-ceiling-ticket.md` — why the platform cannot render
  engine vibration at any plausible limit, and why buffet at 6–8 Hz is renderable on the platform
  today if anyone wants to compare the two approaches directly.
- `MotionProviderPlugin/CLAUDE.md` — the acceleration budget and the `(2πf)²·A` relationship.
- `docs/superpowers/plans/2026-07-19-motion-provider-phase3-washout-effects.md` — where the two
  reserved gains came from.
