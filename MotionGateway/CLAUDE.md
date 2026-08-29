# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

See also the monorepo-level `../CLAUDE.md` for repo-wide conventions, and in particular its
**Motion chain** section for the protocol shared with `MotionActor` and `MotionProviderPlugin`.
This file covers what's specific to MotionGateway.

## Commands

Run from this directory:

```bash
pio run -e megaatmega2560          # production build (USB link to the plugin)
pio run -e megaatmega2560_bench    # bench console build (-D BENCHDEBUG=1)
pio run -t upload -e megaatmega2560
pio device monitor -b 115200
```

`SERIAL_RX_BUFFER_SIZE=256` is set for both envs. The 64-byte default holds only four BFF frames
(~67 ms at 60 Hz); a single long stall silently corrupts frames, and the Mega has 8 KB of RAM to
spare.

Debug logging goes to **`Serial1`** in the production build — `Serial` carries the live plugin link
and must stay clean. Attach an FTDI adapter to pins 18/19 to read it.

## What MotionGateway is

The bridge between `MotionProviderPlugin` (USB serial) and the three `MotionActor` nodes (CAN). It
is a gateway, not an instrument: `CAN` subclasses `BaseCAN` directly and implements the gateway half
of the heartbeat protocol with the motion-specific IDs (`0x300` out, `0x301` tracked per node).

## Modes

Pins 24/25 (`kMode1Pin`/`kMode2Pin`) select the mode, re-read periodically:

- **`mode0`** — off. Bytes are still drained and counted as `discardedBytes`; without that the RX
  buffer overflows and the first frame after a mode switch is corrupt.
- **`mode1`** — BFF Motion Driver compatible. The production mode. Goto frames are valid here only.
- **`mode2`** — SimTools.

Each mode has its own actor mapping table (`actorMappingMode1` / `actorMappingMode2`) translating
the six BFF actuator slots to (nodeId, motorIndex) pairs.

## Serial parser (`handleSerialInput`)

Byte state machine over `Serial`, budgeted to `kSerialProcessingBudgetMs = 100` per pass. In
`mode1`, `'B'` then:

- `'C'` → BFF demand path: reserved byte, 12 data bytes, `0x0D`.
- `'G'` → goto path: 14 data bytes (6 MSB, 6 LSB, duration_ms BE), `0x0D`.
- anything else → resync.

`GotoCR` differs from `CR` on a framing miss: it also **poisons the frame** (`idx = 0`). A goto is a
one-shot command, so one whose CR check failed must never execute; a dropped demand frame is
harmless because the next one arrives 16 ms later.

## Forwarding rules

Two rules here were bought with rig measurements — don't undo them casually:

- **`kDemandBatchIntervalMs = 0`.** Every received frame is forwarded immediately. A 30 ms gate
  aliased against the 60 Hz input into irregular 17/33/50 ms forwarding gaps, which is felt directly
  as jerky motion. Per-pair change dedup in `processDemands()` still limits CAN traffic.
- **CAN sends never run inside the serial drain loop.** The newest complete frame is latched into
  `pendingDemand` / `pendingGoto` and applied once per `loop()` after the drain; blocking CAN sends
  inside the RX loop stall it long enough to overflow the serial buffer. A goto parsed in the same
  drain pass **supersedes** a demand — it is the later, more specific intent.

`checkMaxAgeResync()` resends cached per-node state on a max-age timer so actors recover after a
dropped frame or a restart.

TX error handling: a successful send clears **all** latched TX errors, and errors auto-expire after
1 s. Before that, a single latched `0x110` TX error could never clear and deadlocked forwarding.

## Arm switch

A switch between `kArmPin` (26) and GND (`INPUT_PULLUP`) is reported to the plugin in a USB
heartbeat every `kUsbHeartbeatIntervalMs` (500 ms). The plugin arms only while that heartbeat is
fresh, so a dead gateway or unplugged cable disarms the platform by itself.

## Instrumentation

`GatewayStats` prints one CSV line every 5 s on the debug serial:

```
GS,frames,dtMin,dtMean,dtMax,b<10,b10-20,b20-35,b35-50,b>50,resync,crMiss,discard,sendMaxUs,txFail
```

`dt` is the interarrival of complete demand frames in ms. `resync` counts bytes skipped hunting for
`'B'`, `crMiss` bytes eaten waiting for the terminating CR (framing loss), `discard` bytes dropped
in `mode0`. A healthy production link shows dt ≈ 16 ms, zero resync/crMiss/discard.

## BenchDebug mode (`-D BENCHDEBUG=1`)

Serial console on USB replacing the plugin link. `?` lists the commands:

| Command | Effect |
|---|---|
| `ho<0-3>` / `st<0-3>` | home / stop actor node (0 = all) |
| `p<node><ch><0-100>` | position, e.g. `p11100` |
| `c<node><ch><0-100>` | calibration move |
| `mi<node><ch>` / `ma<node><ch>` | save current position as logical min / max |
| `gt <node\|0> <a1%> <a2%> <ms>` | profiled goto — exercises the arm/disarm path |
| `gs <node> <wave> <rateHz> <amp%> <durS>` | synthetic `0x110` demand stream (`gs 0` stops) |
| `tr` / `ta` / `td` | test-bench start / abort / dump (needs actor testbench firmware) |

## Tools

- `tools/analyze_smoothness.py` — PASS/FAIL on test-bench dumps; gate metric is `residual/step ≤ 0.25`
  from a per-leg quadratic fit. The earlier velocity-ripple metric was quantisation-noise sensitive
  and was replaced. Note the TS CSV field order: `TS,node,state,cycles,maxCmdUs10,missed`.
- `tools/bff_stream.py` — streams synthetic BFF frames at the gateway for timing tests.
