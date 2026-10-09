# TurnCoordinatorCAN

Turn coordinator (turn rate needle + slip ball) as a CAN instrument: Arduino Nano, MCP2515,
one AX1201728SG driving two X27.168. Design: `docs/superpowers/specs/2026-10-09-turn-coordinator-can-design.md`.

## Build / test

    pio run -e nano            # firmware (BENCHDEBUG 0 in src/Configuration.h)
    pio test -e native         # TurnCoordinatorCalibration.h unit tests
    pio device monitor -b 115200

## Bench checklist (BENCHDEBUG 1)

1. Power-up: both needles sweep to their home stop, LED blinks three times.
2. `ho` twice — both needles return to the stop, no skipped steps.
3. `cw`, reset the board, `?` — both tables show 0 points, needles sit on the stop.
4. Calibrate turn rate: `ct<deg>` until the aircraft symbol sits on the left mark → `pt-90`
   (or whatever the dial says), centre → `pt0`, right mark → `pt90`. `?` lists the points.
   Send `pt`/`ps` on its own line after the needle has stopped: a point requested while the
   needle is still travelling is refused ("Needle still moving"), never stored wrong.
5. Calibrate slip: `cs<deg>` onto the left mark → `ps-8`, centre → `ps0`, right → `ps8`.
6. `tr0 sl0` — both needles on centre. `tr90`, `tr-90`, `sl8`, `sl-8` on their marks.
   `tr200` parks on the outermost taught point, no stop is hit.
7. `co` — both needles follow smoothly, no stutter; `cx` stops.
8. `br0` / `br255`.

Direction is not configured in code: if a needle runs the wrong way, teach the points the
other way round.

## Rig checklist (BENCHDEBUG 0)

1. Gateway heartbeat: LED on; both needles on their taught centre.
2. Right turn in X-Plane: aircraft symbol banks right; slip in a skid moves the ball the
   right way. Wrong direction → re-teach points, no code change.
3. From the DCU bench console, `tc45,-4` moves both needles; `tc` alone keeps the slip.
4. From `CANDebugNode`, send `0x103` with len 2: the board must ignore it (no move).
5. Pull the gateway: LED off; reconnect: LED on, needles follow the next frame.
