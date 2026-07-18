# MotionProviderPlugin — Design

**Date:** 2026-07-18
**Status:** Approved (design), pending implementation plan

## 1. Overview

A second X-Plane 12 plugin, `MotionProviderPlugin`, living in its own subfolder beside
`DCUProviderPlugin` and built with the same CMake + shell-script pipeline (not PlatformIO). It:

1. Listens to X-Plane flight-model datarefs (motion cues).
2. Runs a **classical washout filter** plus an **additive effects layer** to produce a commanded
   6-DOF platform pose.
3. Solves the **inverse kinematics of a rotary (6-RSS) Stewart platform** — six servo arms with
   push-rods — to six servo angles.
4. Scales each servo angle to a **16-bit setpoint** (`0..65280`, `32640` = home), applies **safety
   limiting**, and streams the six setpoints over USB serial to the `MotionGateway` board.

The plugin is the X-Plane-side motion-cueing counterpart to the `MotionGateway`/`MotionActor` boards,
analogous to how `DCUProviderPlugin` is the counterpart to the `DCU` board.

### Key architectural decisions

- **Wire format: emit the existing BFF frame first.** `MotionGateway` already decodes two serial
  motion protocols today — BFF Motion Driver (`mode1`) and SimTools (`mode2`) — both carrying six
  16-bit actuator demands. The plugin therefore emits the **BFF frame the gateway already parses**
  (`"BC"` + reserved byte + 6×MSB + 6×LSB + `0x0D`), so the first real-world motion test runs against
  the **unmodified gateway**. A richer, CRC-protected native wire format (plus the matching
  `MotionGateway` receiver change) is deferred to a later, optional phase.
- **Safety before first motion.** The core safety limiter (range clamp + velocity/acceleration
  limiting) lands *with* the serial-output phase, so the very first time real actuators move, runaway
  and over-acceleration protection already exists.
- **Setpoint semantics: linear servo angle.** The downstream `MotionActor` board maps the 16-bit
  demand linearly onto actuator travel (`map(demand, 0, 0xffff, logicalMin, logicalMax)` via the
  Kangaroo controller). The IK produces a servo angle per leg; the plugin normalizes that angle's
  travel to the BFF `0..65280` range (`32640` = home).
- **Config format: TOML via `toml++` (single header).** The build must cross-compile for macOS and
  Windows (MSVC + MinGW) with a zero-install pipeline, so config parsing uses the single-header,
  dependency-free `toml++` library rather than a linked dependency. Hierarchical keys
  (`washout.surge.gain`) map cleanly and parsing is stricter/less error-prone than hand-rolled YAML.

## 2. Module breakdown

Each module has one clear purpose and a well-defined interface.

### Reused from DCUProviderPlugin (near-verbatim)

- **`Plugin.cpp`** — the only file touching the X-Plane plugin ABI
  (`XPluginStart/Stop/Enable/Disable/ReceiveMessage`, flight-loop registration). Holds the single
  `MotionProvider` instance.
- **`ConnectionManager` + `SerialPort` + `MessageQueue` + `TransportLayer`** — the serial I/O
  subsystem. `ConnectionManager` owns a **dedicated I/O thread** (`ioThreadLoop`) with atomic stats and
  reconnect-if-down bookkeeping; `MessageQueue` is the thread-safe handoff; the flight-loop thread
  never touches the serial port. Reused as-is; `TransportLayer` is adapted to also encode the BFF frame
  (see §5).
- **`StatusWindow`** — raw XPLM window (hand-drawn text, mouse hit-testing, menu item). Extended per
  phase for cue display, setpoint display, manual-DOF keyboard control, a port chooser, and a
  "Reload config" button.
- **`ConfigUtils`** — flat `key=value` persistence of last-used serial port + window visibility at
  `~/.motionprovider.cfg` (distinct from the tuning config in §6). Kept as-is.

### New modules

- **`MotionProvider`** (analog to `DCUProvider`) — orchestrator. Once per output tick it runs the
  pipeline: sample → washout → effects → (manual override) → IK → safety → enqueue frame. Owns the
  fixed-rate accumulator that decouples output rate (~60 Hz) from X-Plane frame rate.
- **`DataRefManager`** — the only file calling `XPLMFindDataRef`/`XPLMGetData*`. Resolves datarefs in
  `onAircraftLoaded()` (init + `XPLM_MSG_PLANE_LOADED`), samples cue inputs per frame, falls back to 0
  for absent/aircraft-specific datarefs rather than crashing.
- **`WashoutFilter`** — classical washout. Input: specific forces (surge/sway/heave) + angular rates
  (roll/pitch/yaw) + orientation. Output: commanded 6-DOF pose. High-pass filter on translational
  accel → transient translational cue; low-pass tilt-coordination → sustained-accel gravity alignment;
  high-pass on angular rates → rotational cue. All gains/cutoffs/limits tunable via config. **Pure
  math, no X-Plane dependency** (unit-testable off-device).
- **`EffectsLayer`** — additive discrete cues layered onto the washout pose: gear touchdown bump,
  ground-roll rumble, engine-RPM vibration, aerodynamic buffet. Each effect independently gain-tunable.
- **`StewartKinematics`** — rotary 6-RSS inverse kinematics. Input: 6-DOF pose + platform geometry
  (config constants). Output: six servo angles, then scaled to six 16-bit setpoints. **Pure math,
  unit-testable off-device.**
- **`SafetyLimiter`** — per-setpoint safety: range clamp, plus velocity and acceleration limiting
  (rate-of-change caps), all dynamically settable via config. Later hardened (§4, Phase 5) with runaway
  detection, watchdog/heartbeat, home-on-fault, and soft-start.
- **`MotionConfig`** — loads the TOML tuning config (§6) into typed structs; hot-reloadable.

## 3. Data flow

One output tick (fixed rate ~60 Hz, accumulator-decoupled from frame rate), on the flight-loop thread:

```
DataRefManager.sample()
  → WashoutFilter        → commanded pose (6 DOF)
  → + EffectsLayer       → pose offset
  → + manual override    → (Phase 2a; replaces washout when manual mode active)
  → StewartKinematics    → 6 servo angles → 6 × 16-bit setpoints
  → SafetyLimiter        → clamped setpoints
  → TransportLayer.encodeBFF → frame
  → MessageQueue.enqueueTx
        ┄┄┄ thread boundary ┄┄┄
  → I/O thread (ConnectionManager) → SerialPort.write → MotionGateway
```

All cueing/IK math is cheap and runs on the flight-loop thread; only the serial write crosses to the
I/O thread, exactly as in DCUProviderPlugin.

## 4. Phase plan

Each phase ends with an executable plugin that is real-world testable in X-Plane. Reordered from the
original proposal so the safety core lands with the first serial output.

### Phase 0 — Scaffold
Copy and adapt the DCUProviderPlugin build pipeline (`CMakeLists.txt`, `build-macos.sh`,
`build-windows.sh`, `build-xc-windows.sh`, `build-all.sh`, `toolchain-mingw64.cmake`, `xplm.def`,
`SDK/`, `.vscode/`) into a new `MotionProviderPlugin/` folder. Rename the plugin, signature, menu item,
and config path. Include the reused I/O + window skeleton, not yet wired to output.
**Deliverable:** plugin loads in X-Plane 12, shows an empty status window via its menu item, logs to
`Log.txt`. Does nothing else.

### Phase 1 — Dataref listening
Implement `DataRefManager` with the cue input struct; resolve on aircraft load; sample per frame.
Display key cue values live in the status window for debugging.
**Deliverable:** live specific-force / rate / orientation values visible in the window while flying.

### Phase 2 — Inverse kinematics
Implement `StewartKinematics` with the known platform geometry (hardcoded constants initially). Feed it
a placeholder pose (zero, or raw orientation) and display the six resulting 16-bit setpoints in the
window. No motion.
**Deliverable:** six setpoints displayed and updating; validated against hand-computed values via the
native test target (§7).

### Phase 2a — Manual DOF input
Keyboard control in the status window: Tab selects an axis, ↑/↓ (or +/−) nudges it, R resets to home;
the manual pose feeds the IK. Introduce the TOML tuning config (§6) + "Reload config" button here, with
platform geometry moved into it.
**Deliverable:** manually setting each DOF produces the expected setpoints; geometry editable via config
hot-reload.

### Phase 3 — Washout + effects
Implement `WashoutFilter` (classical, tunable) and `EffectsLayer` (touchdown bump, ground-roll rumble,
engine-RPM vibration, buffet). All tuned via TOML hot-reload.
**Deliverable:** realistic flight-driven pose, viewable as the six setpoints; still no serial output.

### Phase 4 — Serial output + safety core
Wire the BFF frame emitter through `TransportLayer` → `MessageQueue` → `ConnectionManager` I/O thread.
Add the serial port chooser to the status window (reused from DCUProviderPlugin). Implement
`SafetyLimiter` (range clamp + velocity/acceleration limiting, config-tunable) placed immediately before
frame emit.
**Deliverable:** **first real motion**, driving the actuators through the **unmodified** `MotionGateway`
(`mode1`/BFF), already safety-clamped.

### Phase 5 — Safety hardening + optional native protocol
Extend `SafetyLimiter`: runaway detection, connection watchdog/heartbeat, graceful home-on-fault,
startup soft-start ramp. Optionally design a richer CRC-protected native wire format and implement the
matching `MotionGateway` receiver (the deferred follow-up).
**Deliverable:** production-safe link ready for sustained real-actuator operation.

## 5. Wire protocol

**Phase 4 (BFF, against unmodified gateway):** frame `"BC"` (0x42 0x43) + 1 reserved byte + 6 MSB bytes
(act1..act6) + 6 LSB bytes (act1..act6) + `0x0D` (CR). Each actuator's 16-bit demand =
`(MSB << 8) | LSB`, range `0..65280`, `32640` = home. Matches `MotionGateway::handleBFFFrame` exactly.
`TransportLayer` gains a `encodeBFF(const uint16_t setpoints[6])` helper.

**Phase 5 (optional native):** a framed format with magic bytes, message type, length, six `uint16`
payload, and a CRC16 trailer. Requires a new `RxState` parser in `MotionGateway` and is out of scope for
first motion.

## 6. Tuning config

A TOML file (path e.g. `~/.motionprovider.toml`), parsed with the single-header `toml++`, holding all
tunable parameters, **hot-reloadable** via a "Reload config" button in the status window:

- **Geometry** — base joint coordinates, platform joint coordinates, servo horn length, push-rod
  length, servo home angle, per-servo angle travel range.
- **Washout** — per-axis high-pass/low-pass cutoff frequencies, sensitivity gains, tilt-coordination
  limits, translational/rotational output limits.
- **Effects** — per-effect intensity/frequency (touchdown, rumble, engine vibration, buffet).
- **Safety** — per-setpoint min/max, max velocity, max acceleration, soft-start ramp time.
- **Output** — target output rate (Hz).

The serial port and window visibility stay in the separate flat `~/.motionprovider.cfg` via
`ConfigUtils`, matching the DCUProviderPlugin split.

## 7. Testing

DCUProviderPlugin ships no automated test suite. The two pure-math modules — `StewartKinematics` and
`WashoutFilter` — are the highest-risk code and are testable off-device (no X-Plane dependency), so a
small native test target is added for them (test-driven where practical): known pose → known servo
angles for IK, and step/impulse response checks for the washout filters. X-Plane-coupled code
(`DataRefManager`, `StatusWindow`, serial I/O) is validated by on-sim testing per phase.

## 8. Input datarefs (Phase 1 — to confirm during that phase)

Proposed cue set (exact paths pinned in Phase 1, since some are aircraft-specific):

- **Specific forces (body frame):** `sim/flightmodel/forces/g_axil` (surge), `.../g_side` (sway),
  `.../g_nrml` (heave).
- **Angular rates:** `sim/flightmodel/position/P` (roll), `.../Q` (pitch), `.../R` (yaw).
- **Orientation (tilt coordination):** `sim/flightmodel/position/theta` (pitch), `.../phi` (roll).
- **Effects inputs:** on-ground + `.../groundspeed` (touchdown/rumble), engine RPM
  (`sim/cockpit2/engine/indicators/engine_speed_rpm` or `ENGN_N1_`) for vibration, angle-of-attack for
  buffet.
