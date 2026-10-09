# AttitudeCAN

CAN instrument board for the attitude indicator: two nested 28BYJ-48 steppers (pitch motor on
the roll frame), one Hall sensor per axis, one panel LED. Design:
`docs/superpowers/specs/2026-10-09-attitude-can-design.md`.

## Build / test

    pio run -e nano            # firmware (BENCHDEBUG 0 in src/Configuration.h)
    pio test -e native         # AttitudeGeometry.h unit tests
    pio device monitor -b 115200

## Bench checklist (BENCHDEBUG 1)

1. `hm` — both Hall sensors toggle when the axes are turned by hand. Enter stops.
2. `ho` — roll finds its window, then pitch finds its sensor; no stop is touched. `st` shows
   `homed: yes`. A failure blinks the LED 1s/1s; the DebugLog (DEBUGLOG_ENABLE 1) names the
   phase that ran out of travel.
3. `ro90`, `ro-90`, `ro180` — the pitch display stays put while roll turns.
4. `pi30`, `pi-30`, `pi40` (clamped to 30) — no stop is reached.
5. `at10,-20` — both axes arrive together.
6. `li0`, `li255`.
7. `zr` / `zp` after jogging onto the optical zero; `st` shows the accumulated offsets.
   Note: a bootloader upload keeps EEPROM; only an ISP chip-erase without EESAVE wipes it. If the offsets look reset after flashing, that is why.

## Rig checklist (BENCHDEBUG 0)

1. Gateway heartbeat starts homing; LED comes up at the last panel brightness.
2. Roll right in X-Plane: horizon banks the right way, else flip `kRollInversed`.
3. Nose up: horizon moves the right way, else flip `kPitchInversed`. Never use `kStepperInversed[]` for this: both coil flags must match the physical gearing, which bench item 3 establishes (the pitch display stays put under `ro90`); a mismatch there shows as the display drifting with roll.
4. From `CANDebugNode`, send `0x101` with len 2: the board must ignore it (no move, no reset).
5. Pull the gateway: LED off; reconnect: LED back to the last brightness, no re-home.
