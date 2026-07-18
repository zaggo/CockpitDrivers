# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

See also the top-level `/Users/zaggo/Developer/CockpitDrivers/CLAUDE.md` for the whole-rig context (this
plugin is the X-Plane-side counterpart to the `DCU` board project, talking over USB serial).

## Commands

Not PlatformIO — plain CMake + shell scripts, run from this directory:

```bash
./build-macos.sh          # Release build, installs to X-Plane plugins dir
./build-macos.sh debug    # Debug build with symbols (-g -O0), needed for breakpoints
./build-windows.sh        # Native build on Windows (MSVC, needs VS Build Tools)
./build-xc-windows.sh     # Cross-compile for Windows from macOS (MinGW, needs toolchain-mingw64.cmake)
./build-all.sh            # All platforms
```

Requires the X-Plane SDK. `CMakeLists.txt` looks for it in this order: `-DXPLANE_SDK=...`,
`$XPLANE_SDK` env var, `./SDK` (present in this repo), then a couple of hardcoded macOS paths.

Debugging (macOS): VS Code + CodeLLDB, via `.vscode/launch.json` — "Debug X-Plane Plugin" (launches
X-Plane with debugger attached) or "Attach to X-Plane" (already-running instance). Must open a `.cpp`
file before pressing F5 or the debug config picker won't show. Plugin logs go through `XPLMDebugString`
into X-Plane's `Log.txt` (path is machine-specific, see README.md for the configured location).

No automated test suite currently exists (`CMakeLists.txt` conditionally adds a `tests/` subdirectory
if present, but it isn't).

## Architecture

### Data flow

`Plugin.cpp` holds the single `DCUProvider` instance and is the only file touching the X-Plane plugin
ABI (`XPluginStart/Stop/Enable/Disable/ReceiveMessage`, flight loop callback registration). Everything
else lives behind `DCUProvider`, which owns and drives four components once per flight-loop tick
(`onFlightLoopTick`, called every X-Plane frame):

1. `ConnectionManager::update()` — reconnect-if-down bookkeeping (retries every 2s when disconnected).
2. `ConnectionManager::processIO()` — pumps `SerialPort` non-blocking read/write, reassembles framed
   messages from the raw byte stream via `TransportLayer::decodeFrame`, and pushes/pulls them through
   `MessageQueue`.
3. `DCUProvider::updateDownlink()` — X-Plane → gateway. Reads datarefs via `DataRefManager` and
   `msgQueue_->enqueueTx(...)`, one block per data category, each independently rate-limited via its own
   accumulator (fuel 5 Hz, lights/transponder/odometer 10 Hz, RPM 50 Hz — constants in `DCUProvider.h`).
4. `DCUProvider::updateUplink()` — gateway → X-Plane. Drains `MessageQueue` RX, `switch`es on
   `MessageType`, writes into `DataRefManager` setters.

`StatusWindow` is updated once per second (accumulator in `onFlightLoopTick`) and separately owns the
X-Plane menu item / ImGui-ish window for serial port selection; port changes go through
`DCUProvider::changePort()`, which tears down and recreates `ConnectionManager`, clears `MessageQueue`
stats/queues, and sleeps 1s after reopening the port (Arduino reset settling time).

### Wire protocol

Frame format `AA 55 | type(u8) | len(u8) | payload[len]` — encode/decode lives in `TransportLayer`,
`MessageType` enum and payload structs are defined once in the **shared** header
`shared/CANBase/include/SerialMessageId.h` (not duplicated here — this is the same header the `DCU`
board project's serial gateway code uses). Adding a new message type means updating that shared header
plus both sides: `DCUReceiver`/`DCUSender` in `DCU`, and the downlink/uplink switch blocks in
`DCUProvider.cpp` here.

`ConnectionManager::processIO` does the byte-stream reassembly: it buffers raw reads in `rxBuffer_`,
resyncs on `0xAA 0x55` (dropping one byte at a time on mismatch), and only decodes once a full
`4 + payloadLen` frame is available. `MAX_RX_BUFFER` (2048) guards against unbounded growth if framing
gets stuck.

### Config persistence

`ConfigUtils` reads/writes a flat `key=value` file at `~/.dcuprovider.cfg` — currently just last-used
serial port and status window visibility. Not related to the X-Plane dataref/aircraft config.

### Datarefs

`DataRefManager` is the only place that calls `XPLMFindDataRef`/`XPLMGetData*`/`XPLMSetData*`. Lookups
happen in `onAircraftLoaded()` (called both at plugin init and on `XPLM_MSG_PLANE_LOADED`), so datarefs
are re-resolved whenever the aircraft changes — some (the `VFLYTEAIR/...` tach/transponder-mode
datarefs) are third-party and aircraft-specific; getters fall back to 0/default when the dataref is
absent rather than crash. `setTransponderMode` re-resolves `dr_TransponderModeW` lazily since that
third-party dataref may not exist yet at plugin init.
