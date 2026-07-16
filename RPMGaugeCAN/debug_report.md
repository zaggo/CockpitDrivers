# RPMGaugeCAN CAN Lockup — Debug Session Report

## Initial symptom (as reported)

- Setup: DCU running in BENCHDEBUG mode, RPMGaugeCAN connected via CAN bus, both boards' own serial monitors available for logging.
- Action: enter `rp1000` in DCU's BenchDebug console.
- Observed: RPMGauge needle moves to 1000 RPM. Shortly after, DCU's CAN alarm LED turns on. RPMGauge stops responding to any further commands. Power-cycling/resetting the RPMGauge board restores normal operation until the next command is sent.
- Clarification obtained: DCU's BenchDebug console sends the CAN message for a typed command exactly once (not repeatedly / not at 50 Hz).

## Baseline log (before any diagnostic code was added)

Serial output from RPMGaugeCAN, from power-up through the hang, with the stock firmware (no diagnostics added yet):

```
RPMGauge initializing...
RPMGauge: EEPROM config loaded
RPMGauge: minStep=504 maxStep=3732
CAN initialized
Entering Configuration Mode Successful!
Setting Baudrate Successful!
Starting to Set Mask!
Setting Mask Successful!
Starting to Set Mask!
Setting Mask Successful!
Starting to Set Filter!
Setting Filter Successful!
(...6 filter blocks total...)
RPMGauge started up
Gateway heartbeat OK


Gateway heartbeat TIMEOUT
```

Additional facts established via user Q&A at this stage:
- The hang occurs right when the needle reaches its target position.
- The panel backlight (LED) goes dark when the hang occurs.
- Reproducible with other RPM values too (tested rp500, rp3500 in addition to rp1000/rp3000).
- The board's EEPROM calibration was valid (`minStep=504`, `maxStep=3732`), not factory-default.

## Session code changes and tests

All changes below were made during this session and have since been reverted (see "Final state").

### Change 1 — timeout register dump

Added to `RPMGaugeCAN/src/CAN.cpp`, inside `onGatewayHeartbeatTimeout()`: prints `digitalRead(kCanIntPin)`, and the MCP2515's `EFLG`, `TEC` (`errorCountTX()`), `REC` (`errorCountRX()`) via the `mcp_can` library's public accessors.

**Test:** DCU sends an LED brightness command twice (both succeed, brightness changes as expected), then `rp3000` is sent. Needle moves to 3000 smoothly, backlight goes dark, board stops responding.

**Result (log excerpt):**
```
RPMGauge started up
Gateway heartbeat OK




Gateway heartbeat TIMEOUT
  /INT pin (should idle HIGH): LOW
  EFLG: 0
  TEC: 0
  REC: 0
```

### Change 2 — remove `String` concatenation from debug prints

`RPMGaugeCAN/src/CAN.cpp` (`handleFrame()`) and `RPMGaugeCAN/src/RPMGauge.cpp` (`moveNeedle()`, both branches): replaced `String(...) + String(...)` concatenation-based `DEBUGLOG_PRINTLN` calls with sequential `DEBUGLOG_PRINT`/`DEBUGLOG_PRINTLN` calls on primitive values (no heap allocation).

### Change 3 — 1 Hz tick diagnostic + raw MCP2515 register reads

Added a `CAN::loop()` override in `RPMGaugeCAN/src/CAN.cpp` that calls `InstrumentCAN::loop()` then, once per second, prints: `gatewayAlive` state, `/INT` pin level, `EFLG`, `TEC`, `REC`. Also added a raw SPI register-read helper (bypassing the `mcp_can` library's private accessors) and used it to print `CANINTF` (register `0x2C`), `CANINTE` (`0x2B`), `RXB0CTRL` (`0x60`), `RXB1CTRL` (`0x70`) in both the 1 Hz tick and the timeout diagnostic block.

Build: RAM 683/2048 bytes (33.3%), Flash 16940/30720 bytes (55.1%).

**Test A:** fresh reset, send `rp1000` alone.

**Result (log excerpt):**
```
Move RPM needle to 0 adjusted to step 504
RPMGauge started up
[tick] alive=no /INT=HIGH EFLG=0 TEC=0 REC=0 CANINTF=0 CANINTE=3 RXB0CTRL=6 RXB1CTRL=0
Gateway heartbeat OK
[tick] alive=yes /INT=HIGH EFLG=0 TEC=0 REC=0 CANINTF=4 CANINTE=3 RXB0CTRL=6 RXB1CTRL=3
(...13 more identical ticks...)
CAN Message received: ID 262
Move RPM needle to 1000 adjusted to step 1426
[tick] alive=yes /INT=LOW EFLG=0 TEC=0 REC=0 CANINTF=6 CANINTE=3 RXB0CTRL=6 RXB1CTRL=3
Gateway heartbeat TIMEOUT
  /INT pin (should idle HIGH): LOW
  EFLG: 0
  TEC: 0
  REC: 0
  CANINTF: 6
  CANINTE: 3
  RXB0CTRL: 6
  RXB1CTRL: 3
[tick] alive=no /INT=LOW EFLG=0 TEC=0 REC=0 CANINTF=6 CANINTE=3 RXB0CTRL=6 RXB1CTRL=3
(...repeats unchanged indefinitely...)
```

**Test B:** same build, fresh reset. Sequence: LED brightness command, then OLED/odometer display command, then RPM move command (`rp3000`). LED and OLED commands completed without any issue. The RPM move command produced the hang.

**Result (log excerpt):**
```
[tick] alive=yes /INT=HIGH EFLG=0 TEC=0 REC=0 CANINTF=4 CANINTE=3 RXB0CTRL=6 RXB1CTRL=3
CAN Message received: ID 515
(...ticks continue normally, RXB0CTRL=6...)
CAN Message received: ID 496
(...ticks continue normally, RXB0CTRL now reads 7...)
CAN Message received: ID 262
Move RPM needle to 3000 adjusted to step 3270
[tick] alive=yes /INT=LOW EFLG=0 TEC=0 REC=0 CANINTF=6 CANINTE=3 RXB0CTRL=6 RXB1CTRL=3
Gateway heartbeat TIMEOUT
  /INT pin (should idle HIGH): LOW
  EFLG: 0
  TEC: 0
  REC: 0
  CANINTF: 38
  CANINTE: 3
  RXB0CTRL: 6
  RXB1CTRL: 3
[tick] alive=no /INT=LOW EFLG=0 TEC=0 REC=0 CANINTF=38 CANINTE=3 RXB0CTRL=6 RXB1CTRL=3
(...repeats unchanged for 20+ ticks shown...)
```

### Change 4 — clear MCP2515 `EFLG`/`ERRIF`

Added `clearMCPErrorLatch()` to `RPMGaugeCAN/src/CAN.cpp`: reads `EFLG` (`0x2D`) via raw SPI and writes `0x00` if nonzero; reads `CANINTF` (`0x2C`) and clears just the `ERRIF` bit (`0x20`) if set. Called on every `CAN::loop()` iteration (previously the diagnostics only read these, never wrote them).

**Test:** fresh reset, send `rp1000` alone.

**Result (log excerpt):**
```
[tick] alive=yes /INT=HIGH EFLG=0 TEC=0 REC=0 CANINTF=4 CANINTE=3 RXB0CTRL=6 RXB1CTRL=3
(...16 more identical ticks...)
CAN Message received: ID 262
Move RPM needle to 1000 adjusted to step 1426
[tick] alive=yes /INT=LOW EFLG=0 TEC=0 REC=0 CANINTF=6 CANINTE=3 RXB0CTRL=6 RXB1CTRL=3
Gateway heartbeat TIMEOUT
  /INT pin (should idle HIGH): LOW
  EFLG: 0
  TEC: 0
  REC: 0
  CANINTF: 6
  CANINTE: 3
  RXB0CTRL: 6
  RXB1CTRL: 3
[tick] alive=no /INT=LOW EFLG=0 TEC=0 REC=0 CANINTF=6 CANINTE=3 RXB0CTRL=6 RXB1CTRL=3
[tick] alive=no /INT=LOW EFLG=255 TEC=255 REC=255 CANINTF=255 CANINTE=255 RXB0CTRL=255 RXB1CTRL=255
CAN Message received: ID 0
CAN Message received: ID 0
(...repeated many times, IDs seen include 0, 65280, 65535, 2040...)
[tick] alive=no /INT=LOW EFLG=0 TEC=0 REC=0 CANINTF=0 CANINTE=0 RXB0CTRL=0 RXB1CTRL=0
(...repeats unchanged for many ticks shown...)
```

### Change 5 — bound the RX drain loop

`shared/CANBase/src/InstrumentCAN.cpp` / `include/InstrumentCAN.h`: changed the frame-drain `while` loop to a `for` loop capped at a new constant `kMaxFramesPerLoop = 32` iterations per `loop()` call, and added a guard to skip (via `continue`) any received frame reporting `len > 8` before passing it to `handleFrame()`/`updateGatewayHeartbeat()`.

Build succeeded for both `RPMGaugeCAN` (`nano` env) and `DCU` (`megaatmega2560` env). No test result was recorded for this change in isolation before the next change was made.

### Additional user-reported observation (not tied to a new code change)

In a separate attempt (exact code state at the time not pinned down), entering `rp0` — which does not move the needle, since the target position already equals the current position (504, set at boot) — produced the same hang (CAN alarm on DCU, RPMGauge unresponsive).

### Change 6 — swap executed effect for `rpm` and `odometer` messages

In `RPMGaugeCAN/src/CAN.cpp` `handleFrame()`: the `rpm` case (CAN ID `0x106` / 262) was changed to call `odometer->displayNumber()` with a hardcoded digit array (representing "123") instead of `rpmGauge->moveNeedle()`. The `odometer` case (CAN ID `0x1F0` / 496) was changed to call `rpmGauge->moveNeedle(1000)` (hardcoded) instead of `odometer->displayNumber()`.

**Test:** fresh reset, send `rp1000` alone.

**Result:** OLED display showed "123" (confirming the swapped handler ran). CAN lockup occurred, same signature as previous tests (`CAN Message received: ID 262`, then hang; timeout diagnostic showed `/INT` LOW, `CANINTF: 6`, `CANINTE: 3`, `RXB0CTRL: 6`, `RXB1CTRL: 3`, `EFLG`/`TEC`/`REC` all 0).

### Change 7 — pad DCU's `rpm` message payload

`DCU/src/BenchDebug.cpp`, `sendRpm()`: payload buffer changed from 2 bytes to 8 bytes (still only the first 2 bytes carry data via `packBE16`, remaining bytes zero), and the `sendMessage` length argument changed from `2` to `8`. CAN ID (`CanMessageId::rpm`) unchanged.

**Test:** DCU reflashed with this change, RPMGauge restarted (unchanged firmware from Change 6), `rp1000` sent.

**Result:** hang, same signature (`CAN Message received: ID 262`, then `/INT` LOW, `CANINTF: 6`, `CANINTE: 3`, `RXB0CTRL: 6`, `RXB1CTRL: 3`, `EFLG`/`TEC`/`REC` all 0).

### Change 8 — swap which RXB0 filter slot holds which ID

`RPMGaugeCAN/src/CAN.cpp` `instrumentBegin()`: filter slot 0 changed to `CanMessageId::odometer` (was `rpm`), filter slot 1 changed to `CanMessageId::rpm` (was `odometer`). RXB1 filters (`lights`, `gatewayHeartbeat`) unchanged.

**Test A:** fresh reset, send `rp1000` alone.

**Result:** hang. Log showed `RXB0CTRL` reading `7` (rather than `6`) at the point of `CAN Message received: ID 262` and thereafter, consistent with the ID now being matched by filter slot 1 instead of slot 0. Timeout diagnostic: `/INT` LOW, `CANINTF: 6`, `CANINTE: 3`, `RXB0CTRL: 7`, `RXB1CTRL: 3`, `EFLG`/`TEC`/`REC` all 0.

**Test B:** same build, board restarted, send `oh123` (odometer) alone.

**Result:** motor moved (into the minimum end-stop, wrong direction per user observation — expected, since the odometer handler now hardcodes `moveNeedle(1000)` from Change 6) and the board hung. Log:
```
CAN Message received: ID 496
Move RPM needle to 1000 adjusted to step 1426
Gateway heartbeat TIMEOUT
  /INT pin (should idle HIGH): HIGH
  EFLG: 0
  TEC: 0
  REC: 0
  CANINTF: 4
  CANINTE: 3
  RXB0CTRL: 6
  RXB1CTRL: 3
[tick] alive=no /INT=LOW EFLG=0 TEC=0 REC=0 CANINTF=6 CANINTE=3 RXB0CTRL=6 RXB1CTRL=3
(...repeats unchanged...)
```
(Note: in this instance the timeout diagnostic's own `/INT` read was `HIGH`; the very next tick showed `/INT=LOW`, `CANINTF=6`, unchanged thereafter.)

### Change 9 — remove the remaining debug print from `handleFrame()`

`RPMGaugeCAN/src/CAN.cpp` `handleFrame()`: removed the `DEBUGLOG_PRINT`/`DEBUGLOG_PRINTLN` pair that printed `"CAN Message received: ID ..."` (no replacement print added).

**Test:** fresh reset. Multiple `cl` (lights) commands sent in succession — all completed without issue (no freeze). Then `rp1000` sent.

**Result:** OLED changed (per Change 6, the `rpm` case still drives the OLED), backlight LED went off, board hung. Log showed roughly 44 ticks of normal heartbeat operation (no per-message print lines visible, as expected since the print was removed), then one tick with `/INT=LOW`, `CANINTF=6`, `RXB0CTRL=7`, followed by the timeout diagnostic (`/INT` LOW, `CANINTF: 6`, `CANINTE: 3`, `RXB0CTRL: 7`, `RXB1CTRL: 3`, `EFLG`/`TEC`/`REC` all 0), then unchanged ticks indefinitely.

### Time-delay test (same build as Change 9, before Change 9 — see note)

**Note on ordering:** this test was actually run against the Change 8 build (filter-slot swap, debug print still present), before Change 9 (print removal) was made. It is listed here out of strict chronological order for clarity since it directly follows the "why does only rpm/odometer fail" line of questioning.

**Test:** fresh reset, no commands sent for approximately 60 seconds (60 heartbeat ticks), then send a single `cl` (lights) command — nothing else sent before or after.

**Result:** hang. Log showed ~60 ticks of `alive=yes /INT=HIGH CANINTF=4 CANINTE=3 RXB0CTRL=6 RXB1CTRL=3`, then `CAN Message received: ID 515`, then one tick `alive=yes /INT=LOW CANINTF=6 ...`, then `Gateway heartbeat TIMEOUT` (`/INT` LOW, `CANINTF: 6`, `CANINTE: 3`, `RXB0CTRL: 6`, `RXB1CTRL: 3`, `EFLG`/`TEC`/`REC` all 0), then ~17 ticks unchanged at `CANINTF=6`, then one single tick showing `CANINTF=38`, then reverting to `CANINTF=6` for subsequent ticks, continuing indefinitely.

### Change 10 — swap which physical RX buffer (RXB0 vs RXB1) handles which messages

`RPMGaugeCAN/src/CAN.cpp` `instrumentBegin()`: filter slots 0–1 (RXB0) changed to `lights`, `gatewayHeartbeat` (previously on RXB1). Filter slots 2–5 (RXB1) changed to `odometer`, `rpm`, `odometer`, `rpm` (previously `rpm`/`odometer` were on RXB0).

**Test:** fresh reset. Multiple `cl` (lights) commands sent in succession (now filtered via RXB0) — all completed without issue. Then `rp0` sent (now filtered via RXB1).

**Result:** hang. Log:
```
[tick] alive=yes /INT=HIGH EFLG=0 TEC=0 REC=0 CANINTF=4 CANINTE=3 RXB0CTRL=7 RXB1CTRL=0
(...many identical ticks...)
[tick] alive=yes /INT=LOW EFLG=0 TEC=0 REC=0 CANINTF=7 CANINTE=3 RXB0CTRL=7 RXB1CTRL=1
Gateway heartbeat TIMEOUT
  /INT pin (should idle HIGH): LOW
  EFLG: 0
  TEC: 0
  REC: 0
  CANINTF: 7
  CANINTE: 3
  RXB0CTRL: 7
  RXB1CTRL: 1
[tick] alive=no /INT=LOW EFLG=0 TEC=0 REC=0 CANINTF=7 CANINTE=3 RXB0CTRL=7 RXB1CTRL=1
(...repeats unchanged...)
```

## Not yet tested

Proposed but not carried out before the session ended:
- Checking DCU's `Serial1` debug output (separate from the BenchDebug console's USB `Serial` link) for TX-error log entries (`DCU/src/CAN.cpp`'s `sendMessage()` logs `"Error Sending Message to id 0x..."` and records a `TX_ERROR` in `canIdErrors[]` on send failure) during a failing `rp`/`oh` send.

## Final state

All code changes listed above have been reverted to match the repository's `HEAD` commit, with one exception: the `String`-concatenation removal from Change 2 (`RPMGaugeCAN/src/RPMGauge.cpp`, `moveNeedle()`) is no longer an uncommitted experimental change — it was committed separately (commit `e4b5be9`, `"feat: improve debug logging in moveNeedle function for better clarity"`) during this session and is therefore part of the current `HEAD`, not something to roll back.

`git diff` against `HEAD` is clean for every file touched this session (`CAN.cpp`, `CAN.h`, `RPMGauge.cpp`, `InstrumentCAN.cpp`, `InstrumentCAN.h`, `DCU/BenchDebug.cpp`). Both `RPMGaugeCAN` (`nano` env) and `DCU` (`megaatmega2560` env) build successfully at this state.
