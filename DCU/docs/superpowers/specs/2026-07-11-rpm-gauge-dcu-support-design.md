# RPM Gauge (RPMGaugeCAN) DCU + Plugin Support

## Context

A new instrument board, `RPMGaugeCAN`, exists on the CAN bus but the DCU (gateway) and
`DCUProviderPlugin` (X-Plane side) don't yet feed it data. RPMGaugeCAN is receive-only: it
consumes `rpm` (CAN ID `0x106`) and `odometer` (CAN ID `0x1F0`) frames plus the existing shared
`lights` (`0x203`) frame for backlight — it sends nothing to the DCU except its standard instrument
heartbeat (node ID `0x05`, already defined as `CanNodeId::rpmGaugeNodeId`).

Both CAN message IDs and the node ID are already defined in `shared/CANBase/include/CanMessageId.h`
/ `CanNodeId.h`. What's missing is entirely upstream of the CAN bus: the plugin doesn't read the
relevant X-Plane datarefs or send serial frames for them, and the DCU doesn't have a serial message
type to receive that data or logic to forward it onto CAN.

Protocol reference (provided by user):
- `0x106` RPM, 50Hz: bytes 0-1 = uint16 RPM, direct value. Dataref
  `sim/cockpit2/engine/indicators/engine_speed_rpm[0]`.
- `0x1F0` Odometer, 10Hz: byte 0 = Hrs×1000 digit, byte 1 = Hrs×100 digit, byte 2 = Hrs×10 digit,
  byte 3 = Hrs×1 digit, byte 4 = Hrs/10 digit (all uint8, direct digit 0-9), bytes 5-6 = Hrs/100
  as uint16, scaled ×1000. Datarefs `VFLYTEAIR/tach/TachTimeHrs1000` / `Hrs100` / `Hrs10` / `Hrs1`
  / `TachTimeTenths` / `TachTimeHundredths` — aircraft-specific (custom plane add-on), used as-is
  per the protocol doc. (Corrected from an initial `HrsTenths`/`HrsHundredths` guess once the
  aircraft's actual dataref reference was checked — the real names have no `Hrs` infix for those
  two. Also: `TachTimeHrs1000/100/10/1/Tenths` are `INT`-typed datarefs; only `TachTimeHundredths`
  is `FLOAT` — see the Plugin side section below.)

## Scope

Both DCU firmware and `DCUProviderPlugin` get updated in this pass — the plugin needs to actually
source live rpm/odometer datarefs and send them over serial for the DCU-side work to be
observable end-to-end (rather than only testable via BenchDebug).

## Shared header changes

`shared/CANBase/include/SerialMessageId.h` — add two new downlink message types to the existing
`MessageType` enum:

```cpp
SerialMessageRPM      = 0x05,
SerialMessageOdometer = 0x06,
```

No changes needed to `CanMessageId.h` / `CanNodeId.h` — `rpm`, `odometer`, `rpmGaugeNodeId` already
exist there.

## Serial payload layout (plugin → DCU)

Following the existing convention (there's no shared struct for downlink payloads today — each
side agrees on a byte layout informally; `DCUReceiver::handleFrame` validates `len` and reads by
fixed offset):

- **RPM** (`SerialMessageRPM`, 4 bytes): one native-endian `float rpm`.
- **Odometer** (`SerialMessageOdometer`, 24 bytes): six native-endian `float`s, in order —
  `hrs1000, hrs100, hrs10, hrs1, hrsTenths, hrsHundredths`.

Design note: the existing codebase already has one struct (in `DCUProvider.cpp`'s inline downlink
structs) that would suffer compiler padding if fields were re-ordered as bytes-then-word. To avoid
this entirely, both the odometer downlink payload and its DCU-side CAN payload are built as
explicit byte/float arrays rather than as a mixed-width `struct`, sidestepping padding rather than
fighting it with `__attribute__((packed))`.

## Plugin side (`DCUProviderPlugin`)

`DataRefManager`:
- New members: `dr_rpm`, `dr_tachHrs1000`, `dr_tachHrs100`, `dr_tachHrs10`, `dr_tachHrs1`,
  `dr_tachHrsTenths`, `dr_tachHrsHundredths`. Looked up in `onAircraftLoaded()` alongside existing
  datarefs. The `VFLYTEAIR/...` tach datarefs use the same lazy/null-tolerant lookup pattern
  already used for `dr_TransponderModeW` (third-party dataref that may not exist on load or on
  other aircraft).
- New getters: `getRpm()`, `getTachHours1000()`, `getTachHours100()`, `getTachHours10()`,
  `getTachHours1()`, `getTachHoursTenths()`, `getTachHoursHundredths()`. `TachTimeHrs1000/100/10/1`
  and `TachTimeTenths` are `INT`-typed datarefs on this aircraft (confirmed from its dataref
  reference) — their getters use a new `readInt()` helper (mirrors `readFloat()`, wraps
  `XPLMGetDatai`, returns a safe default of 0 when the dataref is null), since reading an INT-only
  dataref with the float accessor silently returns 0 regardless of its real value rather than
  autoconverting. `TachTimeHundredths` is genuinely `FLOAT` and keeps using `readFloat()`. An
  unsupported aircraft (no `VFLYTEAIR` datarefs at all) reports 0 hours rather than crashing; RPM
  (a stock dataref) is unaffected either way.

`DCUProvider::updateDownlink()`:
- New accumulators `rpmAccumulator_`, `odometerAccumulator_`; rate constants `RPM_RATE = 50.0f`,
  `ODOMETER_RATE = 10.0f` Hz, matching the protocol doc rates and the existing
  accumulator-throttle pattern (`FUEL_RATE`, `LIGHTS_RATE`, etc.).
- Two new blocks, each building its payload and calling
  `msgQueue_->enqueueTx(MessageType::SerialMessageRPM, ...)` /
  `MessageType::SerialMessageOdometer`. No change-detection on the plugin side (matches existing
  behavior for fuel/lights/transponder — dedupe happens downstream in `DCUReceiver`).

## DCU firmware side

`DCUReceiver`:
- New cached state: `rpmValue` (uint16_t), `tachHrs1000/100/10/1/Tenths` (uint8_t each),
  `tachHrsHundredths100` (uint16_t, already ×1000-scaled) + two new `MessageMeta` instances
  (`rpmMeta`, `odometerMeta`), both with `maxAgeMs = 5000`, matching the fuel/lights pattern.
- `handleFrame()`: new cases —
  - `SerialMessageRPM`: validate `len == 4`, decode float, round to `uint16_t`, compare against
    cached value, call `sendRpm()` if changed.
  - `SerialMessageOdometer`: validate `len == 24`, decode 6 floats, round/scale each per the
    protocol doc (digits direct, hundredths ×1000), compare against cached values, call
    `sendOdometer()` if any changed.
- `checkMaxAgeResync()`: add the two new resend calls alongside the existing ones, so instruments
  recover after a dropped frame even without new plugin input.

`DCUReceiver::sendRpm()` / `sendOdometer()` (DCU): private methods on `DCUReceiver`, not `CAN` —
matching the existing pattern where `sendFuelLevel()`/`sendCockpitLightLevel()`/`sendTransponder()`
already live on `DCUReceiver` rather than `CAN` (the `CAN` class only exposes the generic
`sendMessage(CanMessageId, len, data)`).
- `sendRpm()`: 2-byte buffer, `packBE16`, `canBus->sendMessage(CanMessageId::rpm, 2, data)`.
- `sendOdometer()`: 7-byte buffer — 5 raw digit bytes + `packBE16` for the hundredths word —
  `canBus->sendMessage(CanMessageId::odometer, 7, data)`.
- No new inbound `CAN::handleFrame` case needed — RPMGaugeCAN sends only its instrument heartbeat,
  no application data back to the DCU.

`CAN.h`:
- `kMaxCanIdErrors`: bump `8` → `12`. Current steady-state distinct trackable CAN IDs (4 outbound +
  3 inbound = 7) plus the 2 new outbound IDs (rpm, odometer) reach 9, leaving headroom for
  per-node heartbeat-timeout entries (`instrumentHeartbeatId + nodeId`) without silently dropping
  error/alarm tracking once the array fills.

## BenchDebug additions (DCU)

For bench testing without X-Plane attached, mirroring the existing `lt`/`rt`/`cl` command style:

- `rp<uint16>`: set RPM value and send immediately via `sendRpm()`.
- `oh<float hours>`: set total odometer hours as one float (e.g. `oh1234.56`); BenchDebug
  decomposes it into the 5 digit bytes + hundredths word (inverse of RPMGaugeCAN's own parsing)
  and sends via `sendOdometer()`. A single float input matches how `lt`/`rt` take one float for
  tank level, rather than requiring 6 separate digit arguments.
- New members: `uint16_t rpmValue`, `float odometerHours`.
- `?` help text gets two new lines describing `rp` and `oh`.

## Edge cases

- Non-VFLYTEAIR aircraft loaded in X-Plane: tach datarefs are null → getters return 0 → odometer
  frame sends all-zero digits. No crash; RPM (stock dataref) is unaffected by aircraft choice.
- RPM gauge is single-engine (`engine_speed_rpm[0]` only), matching the protocol doc.
- `kMaxCanIdErrors` bump is a pure constant change — the array is statically sized and
  zero-initialized, no migration needed.

## Testing

- No unit tests exist yet for CAN/DCUReceiver logic (per repo convention, `test/` is largely an
  empty scaffold) — this change follows that existing convention rather than introducing new test
  infrastructure.
- Manual verification: `pio run` (DCU), `./build-macos.sh` (plugin) must both build clean.
- Bench verification: use the new `rp`/`oh` BenchDebug commands to drive RPMGaugeCAN directly over
  CAN without X-Plane attached, confirming the gauge needle and odometer digits respond correctly.
- End-to-end verification (with X-Plane + VFLYTEAIR aircraft loaded): confirm RPM gauge tracks
  engine RPM and odometer increments plausibly during a flight session.
