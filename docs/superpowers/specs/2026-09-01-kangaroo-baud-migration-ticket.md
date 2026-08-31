# Ticket: raise the actor→Kangaroo baud rate

**Status:** open, worth doing. Not cosmetic — the link runs at roughly 94 % occupancy while the
platform is moving. Written 2026-09-01.

**Type:** robustness. Nothing is broken today; there is simply almost no margin left, and the margin
can be bought with a firmware flash.

## The measurement

`MotionActor` talks to its Kangaroo x2 over the Nano's hardware `Serial` at
`kKangarooBaudRate = 19200` (`MotionActor/src/Configuration.h:19`). One streamed
`p(position, speed)` in Kangaroo packet serial costs, counted from the vendored library
(`KangarooCommandWriter::writeToBuffer`, `KangarooChannel::setNoReply`, `writeBitPackedNumber`):

| Part | Bytes |
|---|---|
| address, command, length | 3 |
| channel name + move flags | 2 |
| motion type + position, 6-bit packed | 4 |
| limit type + speed, 6-bit packed | 4 |
| CRC | 2 |
| **per channel** | **≈15** |
| **two channels per tick** | **≈30** |

The 6-bit packing costs 3 bytes for any value above 2048 counts, which both the position and the
speed normally are; smaller values would shave a byte each.

At 19200 baud, 8N1, that is 1920 bytes/s — **32 bytes per 16.7 ms tick**, of which ≈30 are used.
Put in time: **≈15.6 ms of every 16.7 ms tick is pure transmission** when both channels move. About
1 ms of slack.

`setDemands` skips unchanged targets, so a stationary channel costs nothing — but in motion, which
is the case that matters, both move every tick.

This is also the mechanism behind the original jerkiness recorded in `MotionActor/CLAUDE.md`: a
confirmed (non-streaming) `p()` adds a ~20 ms blocking round-trip per channel and throttled the
demand path to ~25 Hz. `setStreaming(true)` removed the round-trip but not the transmission time.

## What is available

`KangarooChannel::baudRate(int32_t)` (`lib/Kangaroo/Kangaroo.h:714`,
`lib/Kangaroo/KangarooChannel.cpp:160`) sends system command `0x20` with a rate index. Supported:
9600, 19200, 38400, 115200. **No DEScribe, no electrical disconnection** — the actor node
reconfigures its own Kangaroo over the link it already has.

Two properties of that call matter:

- `systemCommand(..., expectReply = false, ...)` — **no confirmation comes back.** The only evidence
  the change took is whether the link answers at the new rate afterwards.
- Whether the setting is persistent or lasts until the next power cycle is **not documented in the
  header**. Five minutes at the bench settles it: set it, power-cycle, see which rate answers.

## The design that makes the unknown irrelevant

Do not migrate by setting a value and hoping. Make the firmware **baud-agnostic at boot**:

1. Try 115200, then 38400, then 19200, then 9600. On each, open the UART and issue one confirmed
   `getP()` with `streaming(false)`, with a short timeout.
2. The first rate that answers wins. Log it.
3. Optionally then send `baudRate(38400)` and reopen at 38400.

This removes the lockout risk entirely: a Kangaroo left on any of the four rates — by a half-finished
migration, a power cycle that reverted a volatile setting, or a future DEScribe session — is found
again on the next boot. The migration becomes a firmware flash rather than a hardware session, and
it is reversible by flashing back.

Note the ordering constraint: the probe must run with streaming **off**, because streamed commands
produce no replies and therefore cannot answer the probe.

## Target 38400, not 115200

- 38400 takes occupancy from ≈94 % to ≈47 % — from 1 ms of slack to 34 ms.
- It is the smaller step on rig wiring, which is the reason 19200 was chosen in the first place.
- The firmware cannot use more anyway: `setDemands` clamps `dtMs` to 10..100 ms, so above 100 Hz the
  speed formula (`delta * 1100 / dtMs`) systematically under-computes. 38400 covers 100 Hz with
  room to spare. Raising the rate past that would require changing the clamp too, which is a
  separate decision with its own tuning consequences.

## What this does NOT buy

**No new effects.** The wall for oscillating cues is acceleration, not sample rate: peak acceleration
is `(2πf)²·A` against the ≈363 mm/s² the `SafetyLimiter` allows, and that ceiling does not move when
the link gets faster. Engine vibration at 40 Hz remains 0.006 mm — under two actuator counts —
at any baud rate. Buffet at 7 Hz is already renderable at 60 Hz today. See the companion ticket
`2026-09-01-acceleration-ceiling-ticket.md` for the constraint that actually binds.

What this buys is a transport path that is not running at 94 %.

## How to verify

The instrument already exists: `MotionActor`'s test bench (`MOTION_TESTBENCH=1`,
`env:nanoatmega328new_testbench`). Strategy 9 instruments the production `0x110` demand path and
records command duration per cycle; `sampleMode 2` dumps the timing series over CAN after the run,
so measuring does not perturb the link being measured.

Expected result: command duration per tick roughly halves at 38400. If it does not, the transmission
time was not the dominant cost and this ticket's premise is wrong — which is itself worth knowing.

Then confirm nothing downstream changed: a rig recording replayed through `washout_replay --verify`,
and `jerk_p95` no worse than the pre-change baseline.

## Risks

- **Three nodes, three Kangaroos.** A partial migration leaves the fleet on mixed rates. The boot
  probe makes that harmless, which is the main argument for doing the probe first and the rate change
  second — in two separate firmware releases if you want to be careful.
- **No confirmation from the baud command.** If a node comes back mute, recovery is the boot probe.
  If the firmware predates the probe, recovery is the reset-trick below.
- **Recovery path without DEScribe:** the Kangaroo sits on the Nano's D0/D1, the same pins as the
  USB-serial chip (this is why debug logging uses a SoftwareSerial — see `MotionActor/CLAUDE.md`).
  Holding `RESET` to `GND` puts the AVR's pins high-impedance and lets a PC talk straight through to
  the Kangaroo. Whether that is clean depends on the series resistor in the USB TX line, which most
  Nano clones have.

## Related

- `MotionActor/CLAUDE.md` — the Kangaroo command path, the streaming/round-trip history, the
  SoftwareSerial debug arrangement.
- `MotionActor/src/TestBench.h` — strategy and sample-mode documentation.
- `docs/superpowers/specs/2026-09-01-acceleration-ceiling-ticket.md` — the companion, and the one
  that actually affects what cues are renderable.
