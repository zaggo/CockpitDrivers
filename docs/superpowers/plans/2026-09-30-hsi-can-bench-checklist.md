# HSICAN bench checklist

Work top to bottom. Steps 1–6 run with `BENCHDEBUG = 1` and the MCP2515
disconnected from the bus; step 7 onwards needs the DCU and X-Plane.

## Before flashing

- [ ] **Blocking: build `DCUProviderPlugin` on a machine with a working Xcode.**
      The sandbox this branch was developed in had no usable developer tools —
      `./build-macos.sh` dies in CMake's compiler detection — so the plugin's
      three commits were written and reviewed but **never compiled**. Everything
      else on the branch was built. Do this before anything is flashed, because
      a plugin that will not compile makes the whole chain untestable.
- [ ] Photograph and write down the existing wiring. After the rewiring
      `HSIDriver` can no longer be flashed as a fallback — its `Configuration.h`
      expects servos on D10..D12 and halls on D2/D8/A0.
- [ ] Reflashing this commit over an earlier build of this branch resets stored
      calibration to defaults (the EEPROM magic now folds in the config layout).
      The only sign is `HSI: No valid EEPROM config, writing defaults` at boot,
      easy to mistake for routine first-boot chatter — if it appears after a
      mid-checklist reflash, redo steps 9 and 11 (backlash, zero calibration).
- [ ] Rewire: `gs1Servo` D10→D7, `fromToServo` D11→D8, `gs2Servo` D12→A0;
      OBS hall D2→GPB4, compass hall A0→GPB5, HDG hall D8→GPB6; new panel-light
      wire to D6.
- [ ] Confirm the MCP2515 sits on D10 `/CS`, D11/D12/D13 SPI, D2 `/INT`.

## Bench, no bus

1. [ ] Build once with `DEBUGLOG_ENABLE 1` and read the `Free RAM:` line at boot
       (it is compiled away at the default `0`). Note the figure — anything under
       400 bytes is the stack/heap collision zone that froze RPMGaugeCAN. Set the
       flag back to `0` and rebuild before item 2 — otherwise the homing/state
       log chatter interleaves with the bench console's own replies for the rest
       of this section.
2. [ ] `hm`, then home **ten times**. Watch specifically for a run that lands
       **~180° off**, not merely for scatter. Any such run means the window
       guard let something through.
3. [ ] All three axes at zero — one command per line, the console does not
       split a line on spaces:
       - `hd0`
       - `ob0`
       - `bg0`

       Card rotation direction: `hd90` should turn the card so that 090 comes
       up under the lubber line.
4. [ ] Course pointer at 090, bug at 270 relative to the card — again one
       command per line:
       - `hd0`
       - `ob90`
       - `bg270`
5. [ ] `cd2.5` / `cd-2.5` and `gs2.5` / `gs-2.5` — full needle travel, both
       directions, no buzzing at the stops.
6. [ ] `fl1` TO, `fl2` FROM, `fl0` noNav, `fl4` GS valid with `fl0` parking the
       GS needles. **FROM has never been seen on this rig** — confirm it appears.
7. [ ] Backlash: send `ob90`, then `ob91`, then back to `ob90`. The pointer must
       return to where it started. Repeat approaching from the other side.
8. [ ] Deadband: hold a steady heading (send no further `hd` command), confirm
       the pointers are silent. Then send `hd1` — a 1° change must still move
       them.
9. [ ] `blo`, `blc`, `blh` — no space before the axis letter, or the axis
       argument silently falls through to `obs`. Measured slack should land
       near 28 / 15 / 75. Send `blsave` after each one, power-cycle, confirm
       they survived. A measurement that aborts prints a message and stores
       nothing; treat a silent 0 as a sensor problem, not a result.
10. [ ] **Known quirk, use one input at a time.** The physical OBS and HDG
        knobs drive their axis directly, while the serial commands work
        through a mirrored copy of the wire state. Turn a knob, then send any
        command, and the axis snaps back to the mirror's value. Drive the
        instrument either by knob or by console during a session, not both.
11. [ ] `czo`, `czc`, `czh` — one per axis, no space before the axis letter,
        after jogging that axis onto its true mark; power-cycle and confirm
        the zero held.
12. [ ] `li0`, `li128`, `li255` — panel light dims smoothly.
13. [ ] Coils stay energised at rest by design. After an hour idle, confirm no
        pointer has drifted.

## On the bus

14. [ ] `BENCHDEBUG = 0`, flash, connect to the bus. Gateway heartbeat brings the
        light up and starts homing.
15. [ ] Pull the CAN cable: light goes out, card returns to north, deviation
        needles to zero, flag to noNav, pointers stay put.
16. [ ] Unplug one hall sensor and re-home: light blinks, and a retry follows
        within 10 s rather than the instrument sitting dead.
17. [ ] DCU bench console `hd`/`ob`/`bg`/`cd`/`gs`/`hf` drive the instrument.
18. [ ] DCU `hw` watch, then turn both knobs: deltas appear in both directions.
        100 clicks each way and back must return to the starting value.

## In the sim

19. [ ] All six datarefs track. Fly a VOR radial and confirm TO/FROM flips.
20. [ ] Fly an ILS: GS needle live, and the needles park when tuned away.
21. [ ] OBS knob turns the course pointer; HDG knob turns the bug.
22. [ ] HDG knob short press syncs the bug onto the current heading.
23. [ ] HDG knob long press (>1.5 s) re-homes. A short press must **not** home.
