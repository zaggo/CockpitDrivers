# HSICAN — bench mode

HSICAN has a serial console for driving and calibrating the instrument on the
bench, without the CAN bus, DCU or X-Plane. This page explains how to use it,
with most of it on the calibration commands.

The full commissioning sequence is in
[`docs/superpowers/plans/2026-09-30-hsi-can-bench-checklist.md`](../docs/superpowers/plans/2026-09-30-hsi-can-bench-checklist.md).

## Getting into bench mode

1. In `src/Configuration.h` set `#define BENCHDEBUG 1`.
2. Optionally set `DEBUGLOG_ENABLE 1` in `src/DebugLog.h`. It adds the homing
   and error log (`*** Timeout`, `no edge found`, `Hall window steps`, …).
   Several failures are **only** visible with it on. The price is that the log
   interleaves with the console's replies.
3. `pio run -t upload`, then `pio device monitor -b 115200`.
4. The board answers `BenchDebug ready - send ? for commands`.

Send one command per line. The line is only executed on Enter (`\n`), and it
is not split on spaces, so `hd0 ob0` is one (bad) command, not two. Send `?`
for the short help.

In bench mode the CAN controller is not used at all. Nothing homes by itself,
so **send `hm` first**: `hd`/`ob`/`bg` do nothing until all three steppers are
homed.

## Axes and servos

| Axis (stepper) | What it moves     | `hm` index | letter (`cz`/`bl`) |
|----------------|-------------------|:----------:|:------------------:|
| `obs`          | course pointer    | `hm1`      | `o`                |
| `compass`      | compass card      | `hm2`      | `c`                |
| `hdg`          | heading bug       | `hm3`      | `h`                |

| Servo index (`sa`) | Servo         | Driven by | Calibrated with |
|:------------------:|---------------|-----------|-----------------|
| `0`                | `cdiServo`    | `cd`      | `sa0`, `dpc`    |
| `1`                | `fromToServo` | `fl`      | `sa1`           |
| `2`                | `gs1Servo`    | `gs` (only while `fl` bit 2 is set) | `ga r`/`gp r` table (right needle) |
| `3`                | `gs2Servo`    | `gs` (only while `fl` bit 2 is set) | `ga l`/`gp l` table (left needle) |

## Command reference

### Driving the instrument

| Command | Meaning |
|---|---|
| `hd<deg>` | Heading. Turns the card; pointer and bug ride along with it. `hd90` puts 090 under the lubber line. Decimals and negatives allowed (`hd-2` = `hd358`). |
| `ob<deg>` | OBS course, relative to north on the card. |
| `bg<deg>` | Heading bug, relative to north on the card. |
| `cd<dots>` | CDI deviation in dots, e.g. `cd2.5`, `cd-1`. |
| `gs<dots>` | Glideslope deviation in dots. Only moves the needles while `fl` has bit 2 set. |
| `fl<bits>` | Flags: bit 0 = TO, bit 1 = FROM, bit 2 = GS valid. `fl1` TO, `fl2` FROM, `fl0` no nav + GS parked, `fl5` TO + GS valid. |
| `li<0-255>` | Panel light PWM. |

The console keeps its own copy of heading/OBS/bug/deviation/flags, the same
picture a CAN `0x104`/`0x105` frame would carry. Every one of these commands
changes its own field and then sends the **whole** picture to the instrument
again. For example, `ob90` after `hd30` still leaves the card at 30. Every `hm…`
resets heading, OBS and bug in that copy to 0, so the picture after homing is
all zero. Deviation and flags are kept.

### Homing and diagnostics

| Command | Meaning |
|---|---|
| `hm` / `hm0` | Home all three steppers. |
| `hm1` / `hm2` / `hm3` | Home only OBS / compass / HDG. The instrument counts as homed once all three have homed successfully. After a power cycle, run `hm` once before using a single-axis home. |
| `hs` | Toggle the hall monitor. On switch-on it prints the MCP23017 port B registers (`IODIR`, `GPPU`, `IPOL` must all read `0x70`). After that it prints `Hall obs:x compass:x hdg:x flips:n` at most every 250 ms, and only when something changed. `1` = sensor sees the magnet. |
| `st` | Stop all steppers and switch their coils off. A pointer can be pushed by hand while the coils are off, so re-home afterwards if you touched it. |

What the hall monitor tells you:

- **An input never changes, even when you turn the axis by hand over its
  magnet:** the signal does not reach the MCP23017 (wiring, dead sensor, wrong
  GPB pin). A line that is permanently `1` means the pin is held low.
- **`flips` climbs while a motor runs but nothing passes a magnet:** the line
  picks up noise. An external 4.7–10 kΩ pull-up is the usual fix.
- A healthy input goes `0 → 1 → 0` once per magnet pass, with a small `flips`
  count.

### Calibration

| Command | Meaning |
|---|---|
| `cz<o\|c\|h>` | Store the current jog as the zero offset for that axis. |
| `bl<o\|c\|h>` | Measure gear backlash (slack) on that axis. Blocking. |
| `blsave` | Store the last `bl` measurement. |
| `bl<o\|c\|h> <steps>` | Set backlash directly, e.g. `blh 90`. |
| `sa<servo> <deg100> <ratio1000>` | Servo mounting offset and linkage ratio (CDI, TO/FROM). |
| `dpc <deg100PerDot>` | Servo input degrees per CDI deviation dot. |
| `gm` | Both GS servos to their absolute 50 % centre. |
| `ga<l\|r> <-100..100>` | Deflect one GS servo, percent of full travel from the centre. |
| `gp<l\|r> <dots>` | Store (dots, current `ga` percent) as an interpolation point for that servo. |
| `gpc<l\|r>` | Clear that servo's table. |
| `cw` | Wipe the whole calibration back to the factory defaults. |

**Every calibration command writes to EEPROM immediately.** There is no
"apply without saving" and no undo, apart from `cw` (which resets
*everything*).

Put the axis letter directly after the command: `czo`, not `cz o`. The parser
checks only the first letter after the command: `c…` = compass, `h…` = hdg,
**anything else, a space included, = obs**. So `bl c` silently measures OBS.

## Calibrating

Order that works:

1. `hm` (and `hs` if homing misbehaves)
2. Backlash, one axis at a time: `blc` `blsave`, `blo` `blsave`, `blh` `blsave`
3. `hm` again: homing uses the backlash value to take up the slack before it
   measures
4. Zero offsets: compass first (`czc`), then `czo`, `czh`
5. CDI and flag servos: offsets and ratios with `sa`, then scaling with `dpc`
6. GS servos: teach each needle's table with `gm` / `ga` / `gp`
7. Power-cycle, `hm`, and check everything held

### Zero offset — `cz`

Homing finds the centre of the axis's hall window. The magnet is not at the
pointer's true zero, so after that the axis turns on by a stored per-axis offset
(`zeroAdjustDegree`, defaults `obs −90`, `compass 85`, `hdg 11`). The homing log
shows it as `Move to adjusted zero position, adjustment degree: …`.

`cz` corrects that offset. It **adds** the distance you have jogged since homing
to the stored value. So you can repeat it: a second pass refines the first
instead of replacing it.

1. `hm` and wait for it to finish. The axis now sits at what the firmware
   *thinks* is zero.
2. Send `hd0`, `ob0`, `bg0`. Nothing should move.
3. Jog the axis onto its true mark **with the console, in whole degrees**:
   - compass: `hd1`, `hd-2`, … until N is exactly under the lubber line
   - OBS: `ob1`, `ob-1`, … until the course pointer points straight up
   - bug: `bg1`, `bg-1`, … until the bug sits straight up
4. Send `czc` / `czo` / `czh` for that axis.
5. **Immediately send `hd0` / `ob0` / `bg0` for that same axis.** `cz` sets that
   axis's position to zero, but the console's copy still holds your jog. The
   next command would drive the axis off the mark by exactly the jog again.
   `hd0` after `czc` (or `ob0` after `czo`, `bg0` after `czh`) brings the copy
   back in line without moving anything.
6. Check with `hm`, then `hd0` `ob0` `bg0`.

Notes:

- **`h` is the heading *bug*, not the card.** The card is `c`. `hd` jogs the card
  *and* carries the pointer and the bug along with it, so `hd2` followed by
  `czh` stores the bug's ride-along as a bug offset and leaves the card
  uncalibrated.
- The offset is stored in **whole degrees**, rounded to the nearest one. Jog in
  whole degrees and you lose nothing.
- The offset only takes effect on the next homing. Until then the axis just
  stays where you put it.
- Don't use the OBS/HDG knobs for jogging on the bench. They drive their axis
  directly and bypass the console's copy, so the next command snaps the
  pointer back.
- Calibrate the compass first. The console computes the pointer and bug
  positions relative to the card, and a jog on `hd` moves them as well.

### Backlash — `bl` / `blsave`

The gear trains have slack: after a reversal the motor turns a few steps before
the pointer follows. The firmware adds `stepsBacklash` extra steps on every
reversal, and homing uses the same value to take up the slack before it
measures. Defaults: `obs 28`, `compass 15`, `hdg 75` steps (4096 steps per turn,
so 1 step ≈ 0.09°).

`bl<axis>` measures it with the hall sensor, the only feedback from the
mechanics. The axis creeps clockwise at slow speed until the hall switches.
The gear teeth now press against their forward flanks and the magnet sits at
the switching point. Then it reverses and counts motor steps until the hall
switches back. During those steps the motor only crosses the gap between the
teeth, and the pointer side does not move yet. Those steps are the slack.

The forward search covers more than a full turn, so the axis can start
anywhere. After homing it stands `zeroAdjustDegree` away from the magnet, and
that is fine. Which window edge it finds doesn't matter.

The measurement only sees the gears between the motor and the magnet. On this
instrument:

- **OBS:** the hall sits on the final axis, so the measurement covers the whole
  train.
- **Compass and HDG bug:** the hall sits in the **middle** of the gear train.
  The slack of the stages after it is invisible, so the measured value is too
  low. Use it as a lower bound and top it up by hand with `blc <steps>` /
  `blh <steps>`, checking with the reversal test below.

A digital hall switch also has a little hysteresis (it switches back at a
slightly different point), which adds a few steps.

Reversal test: `hd0` (card at north) or `bg0` (bug up), then `hd10` / `bg10`
and back to `hd0` / `bg0`. Do the same from the other side (`hd350` / `bg350`
→ `hd0` / `bg0`). If the pointer stops short when coming back, raise the value.
If it overshoots, lower it. One step ≈ 0.09°. The default HDG value of 75
steps is about 6.6°.

1. `hm` first.
2. `blc` (or `blo`, `blh`). Keep your hands off the panel: the measurement
   blocks for up to about 7 s (one slow turn), and the console does not react
   during that time.
3. It prints `Measured slack steps: N`. Plausible values are near the defaults.
4. If N looks right: `blsave`. It prints `Stored slack steps: N`.
5. Next axis. `blsave` always stores the **last** measurement, for the axis that
   measurement was taken on. So always measure → save → next axis.

Pitfalls:

- **`bl` sets the stored backlash of that axis to 0 in EEPROM before it
  measures.** It has to, otherwise the compensation would hide the slack it is
  measuring. If you skip `blsave`, that axis runs with **no** compensation from
  then on, including after a reboot. Either `blsave` or re-run `bl` until
  you get a good value.
- **`0` means failure, not "no slack".** With `DEBUGLOG_ENABLE 1` a failure
  prints `HSI: no edge found` (no hall edge within a full turn) or
  `edge did not return` (no edge back within a quarter turn in reverse).
  Without the log you only see `0`, so **never `blsave` a 0**. Both point to
  the sensor, not the gears. Check it with `hs`.
- Run `hm` after all three axes are measured, so homing takes up the slack
  with the new values.

### Servo linkage — `sa`

```
sa<servo> <deg100> <ratio1000>
```

Each servo angle is computed as

```
servo angle = clamp(input, min, max) × ratio + offset
```

- `deg100` = offset in hundredths of a degree (`8000` = 80.00°). This is the
  servo angle at input 0, i.e. needle centred, GS parked, flag hidden.
- `ratio1000` = linkage ratio × 1000 (`2000` = 2.0). **Negative reverses** the
  direction (the `gs2Servo` default `-1429` is mounted mirrored).
- The clamp is on the *input*, before the ratio: ±40° for CDI, effectively
  none (±999) for the flag.

Always send **both** values, even if you only change one. Current defaults:

| Servo | `sa` | Default |
|---|---|---|
| CDI | `sa0` | `sa0 8000 2000` |
| TO/FROM flag | `sa1` | `sa1 0 1000` |

`sa2` / `sa3` are accepted and stored but **ignored**: the GS servos run from
their taught tables (next section).

`sa` stores the value but **does not move the servo**. Resend the command that
drives it to see the effect:

- **CDI:** `cd0`, then tune the `sa0` offset until the needle is centred. Then
  `cd2` / `cd-2` and check the span.
- **TO/FROM flag:** the input is a raw servo angle: `0` no nav, `90` TO,
  `182` FROM, with ratio `1000` and offset `0`. If all three positions are off
  by the same amount, move them with the offset (`sa1 <deg100> 1000`). Check
  with `fl0`, `fl1`, `fl2`.

Rejected input prints `usage: …` or `bad servo index`. A successful one prints
`servo adjust stored`.

### CDI scale — `dpc`

```
dpc <deg100PerDot>
```

Converts CDI deviation dots into the servo *input* (before the ratio):
`input = dots × deg100PerDot / 100`. Default `1000` = 10° per dot, the value
the old AirManager script used.

Use it to make the needle sit exactly on the dot marks of the face: `cd2`, and
if the needle stops short of the second dot, raise `dpc`; if it overshoots,
lower it.

**Watch the clamp.** The ±40° clamp applies to the input, so it caps the
reachable deflection at `40 / (deg100PerDot / 100)` dots: 4 dots at the
default. With `dpc 1600` the needle stops at 2.5 dots. If full scale runs out
early, lower `dpc` and raise the ratio instead.

`dpc` only takes effect with the next `cd`. Reply: `deg per dot stored`.

### GS needles — `gm` / `ga` / `gp`

The two GS servos have no offset, ratio or deg-per-dot. Each has its own
**taught table** of up to 9 `(dots → percent)` points, and `gs<dots>`
interpolates linearly between them. `percent` is the absolute servo
deflection around its 50 % centre: `-100` = 0°, `0` = 90°, `100` = 180°.
Sign, linkage ratio and any non-linearity of the mechanism end up in the
points, so there is nothing to compute by hand.

Outside the table the **outermost point holds**: with `-2.5` and `2.5` dots
taught, `gs9` stays at the `2.5` position. The taught extremes are the soft
endstops. An empty table centres the servo (0 %).

```
gm                    both GS servos to 50 % (90°)
ga<l|r> <-100..100>   deflect one servo, absolute percent from the centre
gp<l|r> <dots>        store (dots, current ga percent) for that servo
gpc<l|r>              clear that servo's table
```

`l` is the left needle (`gs2Servo`, pin A0), `r` the right one (`gs1Servo`,
D7). `ga` and `gp` take a space: `gal 30`, `gpl 1.5`. `gm` resets the
remembered `ga` percent to 0 for both servos.

Teaching, one needle at a time:

1. `gm`, then `gal <pct>` until the left needle is centred, `gpl 0`.
2. `gal <pct>` until the needle sits on the top (fly-down) mark, `gpl -2`
   (or whatever dots that mark is; sign as X-Plane reports it).
3. Same for the bottom mark, `gpl 2`. Add intermediate marks (`gpl 1`,
   `gpl -1`) if the linkage is visibly non-linear.
4. `gpr …` for the right needle.
5. `fl4` `gs0`, `gs2`, `gs-2`, `gs9`: both needles should agree on every mark,
   and `gs9` must not push past the taught end.

Every `gp` prints the servo's table, e.g. `GS l (gs2): -2.00->-62% 0.00->3% 2.00->70%`.
Storing a point at a dots value already in the table replaces it; a full
table answers `GS table full - gpc<l|r> clears it`.

### Reset — `cw`

`cw` resets **all** calibration to the defaults in `src/Configuration.h`: zero
offsets, backlash, servo offsets/ratios, CDI deg-per-dot and both GS tables
(emptied). It cannot be undone.
Reboot afterwards. Until then the steppers keep the backlash compensation they
already had.

The same reset happens automatically when a firmware update changes the layout
of the stored calibration. The only sign is
`HSI: No valid EEPROM config, writing defaults` at boot, visible with
`DEBUGLOG_ENABLE 1`. If you see that after a reflash, redo backlash and zero
offsets.

### Not adjustable from the console

`moveDeadband` and `reversalDeadband` (per axis, in steps) live in EEPROM
too, but can only be changed through the defaults in `Configuration.h`. They
take effect after a `cw` or on a fresh board.
