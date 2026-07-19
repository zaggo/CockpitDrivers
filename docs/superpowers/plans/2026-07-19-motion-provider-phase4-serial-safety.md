# MotionProviderPlugin — Phase 4 (Serial output + safety core) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stream the six setpoints to the MotionGateway over USB serial as the BFF frame it already decodes, on a dedicated I/O thread with auto-reconnect and a serial-port chooser — gated behind an explicit ARM, with a safety core (envelope-aware pose clamp + per-setpoint velocity/acceleration limiting) so the first real actuator motion is bounded.

**Architecture:** A copied `SerialPort`/`SerialPortUtils` plus a new TX-only `SerialLink` (I/O thread streams the latest BFF frame at a fixed rate, reconnects if the port drops). `BffEncoder` builds the 16-byte frame. The safety core has two parts: `StewartKinematics::clampToReachable` scales a commanded pose toward home until every leg is reachable (before the IK), and `SafetyLimiter` velocity/acceleration-limits the six setpoints (after the IK). `MotionProvider` wires it end-to-end behind an ARM gate: disarmed streams HOME (32640); armed streams the limited live setpoints. The status window gains a port chooser, connection/TX status, and an ARM/DISARM button.

**Tech Stack:** C++17, POSIX termios / Win32 serial (copied), std::thread I/O, existing CMake + native tests, toml++ config.

## Global Constraints

- C++17. `BffEncoder`, `SafetyLimiter`, `SafetyConfig`, `SerialConfig`, and `StewartKinematics::clampToReachable` MUST NOT include any X-Plane SDK header (native-testable). `SerialLink`/`SerialPort` are OS-serial only (also no XPLM).
- Only `Plugin.cpp` calls the plugin ABI; `DataRefManager` stays the only XPLM dataref reader. Serial I/O runs on `SerialLink`'s dedicated thread — never the flight-loop thread.
- Wire format = the existing BFF frame `MotionGateway::handleBFFFrame` decodes: 16 bytes `'B' 'C' reserved(0) MSB[0..5] LSB[0..5] 0x0D`, where actuator i demand = `(MSB[i]<<8)|LSB[i]`, and setpoints are in **BFF actuator order** (the IK already emits `setpoints[]` in BFF order). Do NOT change MotionGateway.
- **ARM gate:** disarmed on startup and after any reconnect; disarmed = the stream target is HOME (all 32640). Only an explicit ARM makes the live setpoints the target. Disarm returns to home *through* the SafetyLimiter (smooth ramp, not a jump).
- SANDBOX: `cmake`/`clang`/`make` BLOCKED. Implementers WRITE FILES ONLY; the user builds, runs the native tests, and does all real-serial/X-Plane verification. Serial hardware behavior cannot be tested here.
- Bump the status window version string to `v0.6 (Phase 4)`.
- Work on branch `feature/motion-provider-plugin`.

## File structure

- Copy from `DCUProviderPlugin/src/`: `SerialPort.h`, `SerialPort.cpp`, `SerialPortUtils.h`, `SerialPortUtils.cpp` (verbatim, no changes).
- Create `src/BffEncoder.h/.cpp`, `src/SerialConfig.h`, `src/SerialLink.h/.cpp`.
- Create `src/SafetyConfig.h`, `src/SafetyLimiter.h/.cpp`.
- Modify `src/StewartKinematics.h/.cpp` (add `clampToReachable`), `src/MotionConfig.h/.cpp` (load `[serial]`/`[safety]`), `src/StatusData.h`, `src/MotionProvider.h/.cpp`, `src/StatusWindow.h/.cpp`.
- Create `tests/test_bff.cpp`, `tests/test_safety.cpp`; extend `tests/test_kinematics.cpp` (clampToReachable). Modify `CMakeLists.txt`, `tests/CMakeLists.txt`.

---

### Task 1: Serial subsystem — SerialPort (copy), BffEncoder, SerialLink

**Files:**
- Copy (verbatim, from DCUProviderPlugin/src/): `SerialPort.h`, `SerialPort.cpp`, `SerialPortUtils.h`, `SerialPortUtils.cpp` → MotionProviderPlugin/src/
- Create: `src/BffEncoder.h`, `src/BffEncoder.cpp`, `src/SerialConfig.h`, `src/SerialLink.h`, `src/SerialLink.cpp`, `tests/test_bff.cpp`
- Modify: `CMakeLists.txt`, `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `BffEncoder::encode(const uint16_t sp[6], uint8_t out[16])` + `BffEncoder::kFrameSize`; `struct SerialConfig{int baud; double rateHz;}` + defaults; `class SerialLink` (`configure`, `connect`, `stop`, `update(float dt)`, `setFrame(const uint8_t*, size_t)`, `isConnected`, `framesSent`, `port`).

- [ ] **Step 1: Copy the serial port files verbatim** (they are OS-generic, unchanged):

```bash
cp DCUProviderPlugin/src/SerialPort.h       MotionProviderPlugin/src/SerialPort.h
cp DCUProviderPlugin/src/SerialPort.cpp     MotionProviderPlugin/src/SerialPort.cpp
cp DCUProviderPlugin/src/SerialPortUtils.h  MotionProviderPlugin/src/SerialPortUtils.h
cp DCUProviderPlugin/src/SerialPortUtils.cpp MotionProviderPlugin/src/SerialPortUtils.cpp
```

- [ ] **Step 2: Write `src/BffEncoder.h`**

```cpp
#pragma once
#include <cstdint>
#include <cstddef>

// Encodes six 16-bit actuator setpoints (BFF actuator order) into the frame
// MotionGateway::handleBFFFrame decodes: "BC" reserved MSB[6] LSB[6] CR.
namespace BffEncoder {
    constexpr std::size_t kFrameSize = 16;
    void encode(const uint16_t setpoints[6], uint8_t out[kFrameSize]);
}
```

- [ ] **Step 3: Write `src/BffEncoder.cpp`**

```cpp
#include "BffEncoder.h"

void BffEncoder::encode(const uint16_t sp[6], uint8_t out[kFrameSize]) {
    out[0] = 'B';
    out[1] = 'C';
    out[2] = 0x00;                       // reserved
    for (int i = 0; i < 6; ++i) {
        out[3 + i]     = static_cast<uint8_t>((sp[i] >> 8) & 0xFF);  // MSB[i]
        out[3 + 6 + i] = static_cast<uint8_t>(sp[i] & 0xFF);         // LSB[i]
    }
    out[15] = 0x0D;                      // CR terminator
}
```

- [ ] **Step 4: Write `src/SerialConfig.h`**

```cpp
#pragma once

struct SerialConfig {
    int    baud   = 115200;
    double rateHz = 60.0;   // BFF frame stream rate
    static SerialConfig defaults() { return SerialConfig{}; }
};
```

- [ ] **Step 5: Write `src/SerialLink.h`**

```cpp
#pragma once
#include "SerialPort.h"
#include "BffEncoder.h"
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <cstdint>

// TX-only serial streamer. A dedicated I/O thread writes the latest frame at a
// fixed rate and never touches the flight-loop thread. update(dt) (called from
// the flight loop) reconnects if the port dropped. Models DCUProviderPlugin's
// ConnectionManager thread lifecycle but streams a single latest frame instead
// of a FIFO (setpoints are realtime: latest wins, no backlog).
class SerialLink {
public:
    SerialLink() = default;
    ~SerialLink();

    SerialLink(const SerialLink&) = delete;
    SerialLink& operator=(const SerialLink&) = delete;

    void configure(const std::string& port, int baud, double rateHz);
    bool connect();          // open + start I/O thread; false if open failed
    void stop();             // stop thread + close port
    void update(float dt);   // flight-loop: reconnect bookkeeping

    void setFrame(const uint8_t* data, std::size_t len);  // thread-safe latest frame

    bool isConnected() const { return connected_.load(); }
    uint64_t framesSent() const { return frames_.load(); }
    std::string port() const { return port_; }

private:
    void startIoThread();
    void stopIoThread();
    void ioThreadLoop();

    SerialPort serial_;
    std::string port_;
    int baud_ = 115200;
    double rateHz_ = 60.0;

    std::thread ioThread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::atomic<uint64_t> frames_{0};

    std::mutex frameMutex_;
    uint8_t frame_[BffEncoder::kFrameSize] = {0};
    bool haveFrame_ = false;

    float reconnectAccum_ = 0.0f;
    static constexpr float kReconnectInterval = 2.0f;  // s
};
```

- [ ] **Step 6: Write `src/SerialLink.cpp`**

```cpp
#include "SerialLink.h"
#include <chrono>
#include <cstring>

SerialLink::~SerialLink() {
    stop();
}

void SerialLink::configure(const std::string& port, int baud, double rateHz) {
    port_ = port;
    baud_ = baud;
    rateHz_ = rateHz > 0.0 ? rateHz : 60.0;
}

bool SerialLink::connect() {
    if (isConnected()) return true;
    if (port_.empty()) return false;
    if (!serial_.openPort(port_, baud_)) return false;
    reconnectAccum_ = 0.0f;
    startIoThread();
    return true;
}

void SerialLink::stop() {
    stopIoThread();
    serial_.closePort();
    connected_.store(false);
}

void SerialLink::startIoThread() {
    if (ioThread_.joinable()) ioThread_.join();  // reap a self-exited thread
    running_.store(true);
    connected_.store(true);
    ioThread_ = std::thread(&SerialLink::ioThreadLoop, this);
}

void SerialLink::stopIoThread() {
    if (ioThread_.joinable()) {
        running_.store(false);
        ioThread_.join();
    }
}

void SerialLink::ioThreadLoop() {
    const auto period = std::chrono::microseconds(
        static_cast<long long>(1'000'000.0 / (rateHz_ > 0.0 ? rateHz_ : 60.0)));
    while (running_.load(std::memory_order_relaxed) && serial_.isOpen()) {
        uint8_t local[BffEncoder::kFrameSize];
        bool send = false;
        {
            std::lock_guard<std::mutex> lk(frameMutex_);
            if (haveFrame_) { std::memcpy(local, frame_, sizeof(local)); send = true; }
        }
        if (send) {
            if (serial_.writeBestEffort(local, sizeof(local))) {
                frames_.fetch_add(1, std::memory_order_relaxed);
            }
            // writeBestEffort closes the port on hard error; loop guard exits.
        }
        std::this_thread::sleep_for(period);
    }
    connected_.store(false);  // port dropped or stopped; update() will reconnect
}

void SerialLink::update(float dt) {
    if (isConnected()) { reconnectAccum_ = 0.0f; return; }
    reconnectAccum_ += dt;
    if (reconnectAccum_ >= kReconnectInterval) {
        reconnectAccum_ = 0.0f;
        // Reap the exited thread before reopening.
        if (ioThread_.joinable()) ioThread_.join();
        connect();
    }
}

void SerialLink::setFrame(const uint8_t* data, std::size_t len) {
    if (len != BffEncoder::kFrameSize) return;
    std::lock_guard<std::mutex> lk(frameMutex_);
    std::memcpy(frame_, data, len);
    haveFrame_ = true;
}
```

- [ ] **Step 7: Write `tests/test_bff.cpp`**

```cpp
#include "BffEncoder.h"
#include <cstdio>
#include <cstdint>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }

int main() {
    uint16_t sp[6] = {0x1234, 0x00FF, 0xFF00, 32640, 0, 65280};
    uint8_t f[BffEncoder::kFrameSize];
    BffEncoder::encode(sp, f);

    check(BffEncoder::kFrameSize == 16, "frame is 16 bytes");
    check(f[0] == 'B' && f[1] == 'C', "starts with BC");
    check(f[2] == 0x00, "reserved byte 0");
    check(f[15] == 0x0D, "CR terminator");

    // Round-trip decode exactly as MotionGateway::handleBFFFrame does.
    for (int i = 0; i < 6; ++i) {
        uint16_t demand = static_cast<uint16_t>((f[3 + i] << 8) | f[3 + 6 + i]);
        check(demand == sp[i], "setpoint round-trips through BFF frame");
    }
    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
```

- [ ] **Step 8: Wire builds.** `CMakeLists.txt` SOURCES += `src/SerialPort.cpp src/SerialPortUtils.cpp src/BffEncoder.cpp src/SerialLink.cpp`; HEADERS += `src/SerialPort.h src/SerialPortUtils.h src/BffEncoder.h src/SerialConfig.h src/SerialLink.h`. `tests/CMakeLists.txt`:

```cmake
add_executable(test_bff test_bff.cpp ../src/BffEncoder.cpp)
target_include_directories(test_bff PRIVATE ../src)
add_test(NAME bff COMMAND test_bff)
```

- [ ] **Step 9: Commit**

```bash
git add MotionProviderPlugin/src/SerialPort.h MotionProviderPlugin/src/SerialPort.cpp MotionProviderPlugin/src/SerialPortUtils.h MotionProviderPlugin/src/SerialPortUtils.cpp MotionProviderPlugin/src/BffEncoder.h MotionProviderPlugin/src/BffEncoder.cpp MotionProviderPlugin/src/SerialConfig.h MotionProviderPlugin/src/SerialLink.h MotionProviderPlugin/src/SerialLink.cpp MotionProviderPlugin/tests/test_bff.cpp MotionProviderPlugin/CMakeLists.txt MotionProviderPlugin/tests/CMakeLists.txt
git commit -m "feat(motion): Phase 4 Task 1 - serial subsystem (SerialPort, BffEncoder, SerialLink)"
```

- [ ] **Step 10: Manual test (user)** — `cd MotionProviderPlugin/tests && cmake -B build && cmake --build build && ./build/test_bff` → `0 failures`.

---

### Task 2: Safety core — SafetyLimiter + envelope clamp

**Files:**
- Create: `src/SafetyConfig.h`, `src/SafetyLimiter.h`, `src/SafetyLimiter.cpp`, `tests/test_safety.cpp`
- Modify: `src/StewartKinematics.h`, `src/StewartKinematics.cpp` (add `clampToReachable`), `tests/test_kinematics.cpp`, `CMakeLists.txt`, `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `struct SafetyConfig{double maxVelocity; double maxAcceleration;}` + defaults; `class SafetyLimiter` (`limit(const uint16_t desired[6], double dt, uint16_t out[6])`, `reset(const uint16_t initial[6])`, `setConfig`); `Pose StewartKinematics::clampToReachable(const Pose&) const`.

- [ ] **Step 1: Write `src/SafetyConfig.h`**

```cpp
#pragma once

// Per-setpoint rate limits, in 16-bit demand counts (0..65280 full scale).
struct SafetyConfig {
    double maxVelocity     = 30000.0;   // counts / second
    double maxAcceleration = 120000.0;  // counts / second^2
    static SafetyConfig defaults() { return SafetyConfig{}; }
};
```

- [ ] **Step 2: Write `src/SafetyLimiter.h`**

```cpp
#pragma once
#include <cstdint>
#include "SafetyConfig.h"

// Velocity- and acceleration-limits each of the six setpoints so commanded
// motion never jumps. Stateful; dt-driven; no X-Plane deps.
class SafetyLimiter {
public:
    explicit SafetyLimiter(const SafetyConfig& cfg) : cfg_(cfg) {}

    // Rate-limit toward the desired setpoints; writes the limited result to out.
    void limit(const uint16_t desired[6], double dt, uint16_t out[6]);

    // Snap internal state to a known position (e.g. home) with zero velocity.
    void reset(const uint16_t initial[6]);

    void setConfig(const SafetyConfig& cfg) { cfg_ = cfg; }

private:
    SafetyConfig cfg_;
    double pos_[6] = {0,0,0,0,0,0};
    double vel_[6] = {0,0,0,0,0,0};
    bool   init_ = false;
};
```

- [ ] **Step 3: Write `src/SafetyLimiter.cpp`**

```cpp
#include "SafetyLimiter.h"
#include <cmath>

namespace {
double clampd(double v, double lo, double hi){ return v<lo?lo:(v>hi?hi:v); }
}

void SafetyLimiter::reset(const uint16_t initial[6]) {
    for (int i = 0; i < 6; ++i) { pos_[i] = initial[i]; vel_[i] = 0.0; }
    init_ = true;
}

void SafetyLimiter::limit(const uint16_t desired[6], double dt, uint16_t out[6]) {
    if (dt <= 0.0) dt = 1.0 / 60.0;
    if (!init_) { reset(desired); }  // first call: start at the desired position

    const double vMax = cfg_.maxVelocity;
    const double dvMax = cfg_.maxAcceleration * dt;

    for (int i = 0; i < 6; ++i) {
        const double target = static_cast<double>(desired[i]);
        // Velocity needed to reach target in one step, capped to vMax.
        double desiredVel = clampd((target - pos_[i]) / dt, -vMax, vMax);
        // Acceleration-limit the change in velocity.
        double dv = clampd(desiredVel - vel_[i], -dvMax, dvMax);
        vel_[i] += dv;
        pos_[i] += vel_[i] * dt;
        pos_[i] = clampd(pos_[i], 0.0, 65280.0);
        out[i] = static_cast<uint16_t>(std::lround(pos_[i]));
    }
}
```

- [ ] **Step 4: Add `clampToReachable` to `src/StewartKinematics.h`** — declare in the public section:

```cpp
    // Scale a commanded pose toward home (all-zero pose, always reachable) until
    // every leg is reachable, so the platform degrades gracefully instead of
    // saturating. Returns pose unchanged if already fully reachable.
    Pose clampToReachable(const Pose& pose) const;
```

- [ ] **Step 5: Implement it in `src/StewartKinematics.cpp`** (append at end of file):

```cpp
Pose StewartKinematics::clampToReachable(const Pose& pose) const {
    if (solve(pose).allReachable) return pose;

    auto scaled = [&](double s) {
        Pose p;
        p.surge = static_cast<float>(pose.surge * s);
        p.sway  = static_cast<float>(pose.sway  * s);
        p.heave = static_cast<float>(pose.heave * s);
        p.roll  = static_cast<float>(pose.roll  * s);
        p.pitch = static_cast<float>(pose.pitch * s);
        p.yaw   = static_cast<float>(pose.yaw   * s);
        return p;
    };

    double lo = 0.0, hi = 1.0;               // lo always reachable (home), hi not
    for (int i = 0; i < 14; ++i) {
        double mid = 0.5 * (lo + hi);
        if (solve(scaled(mid)).allReachable) lo = mid; else hi = mid;
    }
    return scaled(lo);
}
```

- [ ] **Step 6: Write `tests/test_safety.cpp`**

```cpp
#include "SafetyLimiter.h"
#include "SafetyConfig.h"
#include "StewartKinematics.h"
#include "StewartGeometry.h"
#include "Pose.h"
#include <cstdio>
#include <cmath>
#include <cstdint>

static int g_failures = 0, g_checks = 0;
static void check(bool c, const char* w){ ++g_checks; if(!c){++g_failures; std::printf("  FAIL: %s\n", w);} }

int main() {
    const double dt = 1.0/60.0;

    // SafetyLimiter: step 32640 -> 65280 ramps, never exceeds velocity/accel, converges.
    {
        SafetyConfig cfg = SafetyConfig::defaults();
        SafetyLimiter lim(cfg);
        uint16_t home[6]; for(int i=0;i<6;i++) home[i]=32640;
        lim.reset(home);
        uint16_t desired[6]; for(int i=0;i<6;i++) desired[i]=65280;
        uint16_t out[6], prev[6]; for(int i=0;i<6;i++) prev[i]=32640;
        double maxStep = 0.0;
        uint16_t last = 32640;
        for (int t=0;t<600;t++) {
            lim.limit(desired, dt, out);
            double step = std::fabs((double)out[0] - (double)prev[0]);
            if (t>0) maxStep = std::max(maxStep, step);
            for(int i=0;i<6;i++) prev[i]=out[i];
            last = out[0];
        }
        // per-step move <= vMax*dt (+1 rounding)
        check(maxStep <= cfg.maxVelocity*dt + 1.5, "step within velocity limit");
        check(std::abs((int)last - 65280) <= 2, "converges to target");
    }

    // SafetyLimiter: output never leaves [0,65280] under an out-of-range demand path.
    {
        SafetyLimiter lim(SafetyConfig::defaults());
        uint16_t z[6]={0,0,0,0,0,0}; lim.reset(z);
        uint16_t hi[6]; for(int i=0;i<6;i++) hi[i]=65280;
        uint16_t out[6];
        bool inRange = true;
        for (int t=0;t<2000;t++){ lim.limit(hi, dt, out); for(int i=0;i<6;i++) if(out[i]>65280) inRange=false; }
        check(inRange, "output stays within [0,65280]");
    }

    // clampToReachable: an over-range pose becomes reachable and is scaled down.
    {
        StewartKinematics k(StewartGeometry::defaults());
        Pose big; big.pitch = 30.0f; big.roll = 30.0f;   // well outside envelope
        check(!k.solve(big).allReachable, "test pose is unreachable pre-clamp");
        Pose c = k.clampToReachable(big);
        check(k.solve(c).allReachable, "clamped pose is reachable");
        check(std::fabs(c.pitch) < std::fabs(big.pitch), "clamped pose scaled toward home");
        // A reachable pose passes through unchanged.
        Pose small; small.pitch = 1.0f;
        Pose c2 = k.clampToReachable(small);
        check(std::fabs(c2.pitch - small.pitch) < 1e-6, "reachable pose unchanged");
    }

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
```

- [ ] **Step 7: Wire builds.** `CMakeLists.txt` SOURCES += `src/SafetyLimiter.cpp`; HEADERS += `src/SafetyConfig.h src/SafetyLimiter.h`. `tests/CMakeLists.txt`:

```cmake
add_executable(test_safety test_safety.cpp ../src/SafetyLimiter.cpp ../src/StewartKinematics.cpp)
target_include_directories(test_safety PRIVATE ../src)
add_test(NAME safety COMMAND test_safety)
```

- [ ] **Step 8: Commit**

```bash
git add MotionProviderPlugin/src/SafetyConfig.h MotionProviderPlugin/src/SafetyLimiter.h MotionProviderPlugin/src/SafetyLimiter.cpp MotionProviderPlugin/src/StewartKinematics.h MotionProviderPlugin/src/StewartKinematics.cpp MotionProviderPlugin/tests/test_safety.cpp MotionProviderPlugin/CMakeLists.txt MotionProviderPlugin/tests/CMakeLists.txt
git commit -m "feat(motion): Phase 4 Task 2 - safety core (SafetyLimiter + clampToReachable)"
```

- [ ] **Step 9: Manual test (user)** — `cmake --build build && ./build/test_safety && ./build/test_kinematics` → `0 failures`.

---

### Task 3: Config + MotionProvider integration (ARM gate, safety, serial)

**Files:**
- Modify: `src/MotionConfig.h`, `src/MotionConfig.cpp` (load `[serial]`/`[safety]`), `src/StatusData.h`, `src/MotionProvider.h`, `src/MotionProvider.cpp`

**Interfaces:**
- Produces: `MotionConfig::loadSerial(path)->SerialConfig`, `loadSafety(path)->SafetyConfig`; `MotionProvider` owns `SerialLink`, `SafetyLimiter`, `armed_`; UI actions `UI_ARM_TOGGLE`, `UI_RESCAN`, and port selection. StatusData gains `armed`, `serialConnected`, `framesSent`, `serialPort`, `sentSetpoints[6]`.

- [ ] **Step 1: `MotionConfig` loaders.** In `MotionConfig.h` add includes `"SerialConfig.h"`, `"SafetyConfig.h"` and declarations `SerialConfig loadSerial(const std::string&); SafetyConfig loadSafety(const std::string&);`. In `MotionConfig.cpp` add (reusing `getDouble`/`getInt`):

```cpp
SerialConfig MotionConfig::loadSerial(const std::string& path) {
    SerialConfig s = SerialConfig::defaults();
    toml::table tbl;
    try { tbl = toml::parse_file(path); } catch (const toml::parse_error&) { return s; }
    if (auto t = tbl["serial"].as_table()) {
        getInt(*t, "baud", s.baud);
        getDouble(*t, "output_rate_hz", s.rateHz);
    }
    return s;
}

SafetyConfig MotionConfig::loadSafety(const std::string& path) {
    SafetyConfig s = SafetyConfig::defaults();
    toml::table tbl;
    try { tbl = toml::parse_file(path); } catch (const toml::parse_error&) { return s; }
    if (auto t = tbl["safety"].as_table()) {
        getDouble(*t, "max_velocity_cps", s.maxVelocity);
        getDouble(*t, "max_acceleration_cps2", s.maxAcceleration);
    }
    return s;
}
```

- [ ] **Step 2: Extend `src/StatusData.h`** — add fields (after `commandedPose`):

```cpp
    uint16_t    sentSetpoints[6] = {32640,32640,32640,32640,32640,32640};
    bool        armed = false;
    bool        serialConnected = false;
    unsigned long long framesSent = 0;
    std::string serialPort;         // empty = none selected
```
Add `#include <string>` and `#include <cstdint>` to StatusData.h.

- [ ] **Step 3: Extend the UI action enum in `src/StatusData.h`**:

```cpp
enum UiAction {
    UI_RELOAD = 0,
    UI_TOGGLE_MODE,
    UI_NEXT_AXIS,
    UI_NUDGE_MINUS,
    UI_NUDGE_PLUS,
    UI_RESET,
    UI_ARM_TOGGLE,
    UI_RESCAN_PORTS
};
```

- [ ] **Step 4: `src/MotionProvider.h`** — add includes `"SerialLink.h"`, `"SafetyLimiter.h"`, `"BffEncoder.h"`, `<string>`; add members:

```cpp
    std::unique_ptr<SerialLink> serial_;
    std::unique_ptr<SafetyLimiter> safety_;
    bool armed_ = false;
    uint16_t sentSetpoints_[6] = {32640,32640,32640,32640,32640,32640};
```
and methods `void selectPort(const std::string& port); void rescanPorts();` (declare; the window calls back into these — see Task 4). Keep existing members.

- [ ] **Step 5: `src/MotionProvider.cpp`** — integrate. Add includes `"SerialLink.h"`, `"SafetyLimiter.h"`, `"BffEncoder.h"`, `"ConfigUtils.h"`.

In `initialize()` after the filters:

```cpp
    safety_ = std::make_unique<SafetyLimiter>(MotionConfig::loadSafety(MotionConfig::defaultPath()));
    uint16_t home[6]; for (int i=0;i<6;i++) home[i]=32640;
    safety_->reset(home);

    serial_ = std::make_unique<SerialLink>();
    SerialConfig sc = MotionConfig::loadSerial(MotionConfig::defaultPath());
    std::string lastPort = loadLastUsedPort();   // ConfigUtils (~/.motionprovider.cfg)
    if (!lastPort.empty()) {
        serial_->configure(lastPort, sc.baud, sc.rateHz);
        serial_->connect();   // opens DISARMED - streams home until armed
    }
```

In `shutdown()` add `if (serial_) serial_->stop(); serial_.reset(); safety_.reset();`.

Add the new methods:

```cpp
void MotionProvider::selectPort(const std::string& port) {
    if (!serial_) return;
    serial_->stop();
    SerialConfig sc = MotionConfig::loadSerial(MotionConfig::defaultPath());
    serial_->configure(port, sc.baud, sc.rateHz);
    serial_->connect();
    saveLastUsedPort(port);      // ConfigUtils persist
    armed_ = false;              // always re-arm deliberately after a port change
}

void MotionProvider::rescanPorts() { /* handled in the window via enumerateSerialPorts */ }
```

Extend `reloadConfig()` to also re-read serial/safety config:

```cpp
    if (safety_) safety_->setConfig(MotionConfig::loadSafety(path));
    // serial rate/baud change needs a reconnect to take effect:
    if (serial_) {
        SerialConfig sc = MotionConfig::loadSerial(path);
        serial_->configure(serial_->port(), sc.baud, sc.rateHz);
    }
```

In `onUiAction`, add cases:

```cpp
        case UI_ARM_TOGGLE:  armed_ = !armed_; break;
        case UI_RESCAN_PORTS: /* window rescans; nothing to do here */ break;
```

Replace the end of `onFlightLoopTick` (the solve + status section) so it applies the envelope clamp, arm gate, safety limiter, and streams the frame:

```cpp
    latestPose_ = pose;
    if (kin_) {
        Pose reachable = kin_->clampToReachable(pose);
        latestPose_ = reachable;
        latestSolve_ = kin_->solve(reachable);
    }

    // Arm gate: disarmed -> stream home; armed -> live setpoints. Either target
    // passes through the SafetyLimiter so disarm/home is a smooth ramp.
    uint16_t target[6];
    for (int i = 0; i < 6; ++i)
        target[i] = armed_ ? latestSolve_.setpoints[i] : static_cast<uint16_t>(32640);
    if (safety_) safety_->limit(target, static_cast<double>(elapsedSec), sentSetpoints_);
    else for (int i=0;i<6;i++) sentSetpoints_[i] = target[i];

    if (serial_) {
        uint8_t frame[BffEncoder::kFrameSize];
        BffEncoder::encode(sentSetpoints_, frame);
        serial_->setFrame(frame, sizeof(frame));
        serial_->update(elapsedSec);
    }

    statusAccumSec_ += elapsedSec;
    if (statusAccumSec_ >= 1.0f) { statusAccumSec_ = 0.0f; pushStatus(); }
```

Note: the `clampToReachable` now feeds `latestPose_`/display, so the shown commanded pose is the reachable one. In `onUiAction`'s immediate manual re-solve, also route through the clamp: `if (kin_ && manualMode_) { Pose r = kin_->clampToReachable(manualPose_); latestPose_=r; latestSolve_=kin_->solve(r); }`.

Extend `pushStatus()`:

```cpp
    for (int i=0;i<6;i++) sd.sentSetpoints[i] = sentSetpoints_[i];
    sd.armed = armed_;
    sd.serialConnected = serial_ && serial_->isConnected();
    sd.framesSent = serial_ ? serial_->framesSent() : 0;
    sd.serialPort = serial_ ? serial_->port() : std::string();
```

- [ ] **Step 6: Commit**

```bash
git add MotionProviderPlugin/src/MotionConfig.h MotionProviderPlugin/src/MotionConfig.cpp MotionProviderPlugin/src/StatusData.h MotionProviderPlugin/src/MotionProvider.h MotionProviderPlugin/src/MotionProvider.cpp
git commit -m "feat(motion): Phase 4 Task 3 - envelope clamp + arm gate + safety + serial streaming"
```

- [ ] **Step 7: Manual test (user)** — build; with no port selected nothing streams; the commanded pose should never show "unreachable" now (clamp). Full serial test needs Task 4's UI.

---

### Task 4: Status window — port chooser, ARM/DISARM, connection/TX status

**Files:**
- Modify: `src/StatusWindow.h`, `src/StatusWindow.cpp`, `src/MotionProvider.cpp` (wire window callbacks)

**Interfaces:**
- Produces: window shows serial status (port, connected, frames), an ARM/DISARM button, a clickable port list + Rescan, and the sent setpoints. `StatusWindow::setPortSelectedCallback(std::function<void(const std::string&)>)`; ports enumerated via `enumerateSerialPorts()`.

- [ ] **Step 1: `src/StatusWindow.h`** — add include `"SerialPortUtils.h"` and `#include <vector>`; give `Button` an optional port field and add a port-select callback + cached port list:

Change the `Button` struct to `struct Button { int left, top, right, bottom, action; std::string port; };`
Add members:
```cpp
    std::function<void(const std::string&)> portSelectedCallback_;
    std::vector<std::string> ports_;
```
Add methods:
```cpp
    void setPortSelectedCallback(std::function<void(const std::string&)> cb);
    void rescanPorts();   // refresh ports_ via enumerateSerialPorts()
```

- [ ] **Step 2: `src/StatusWindow.cpp`** — implement the additions and extend `draw()`/`mouseCallback`/`button`.

Add a `portButton` helper (like `button` but tags the rect with a port) and update `mouseCallback` to dispatch port buttons:

```cpp
void StatusWindow::setPortSelectedCallback(std::function<void(const std::string&)> cb) {
    portSelectedCallback_ = std::move(cb);
}
void StatusWindow::rescanPorts() { ports_ = enumerateSerialPorts(); }
```
In `initialize()`, call `rescanPorts();` once after creating the window.
In `mouseCallback`, when a hit button has a non-empty `port`, invoke `portSelectedCallback_(b.port)` instead of `commandCallback_`:
```cpp
        if (x >= b.left && x <= b.right && y >= b.bottom && y <= b.top) {
            if (!b.port.empty()) { if (portSelectedCallback_) portSelectedCallback_(b.port); }
            else if (commandCallback_) commandCallback_(b.action);
            return;
        }
```
Have `button(...)` push `""` for the port field; add a `portButton(x,y,label,port,...)` that pushes the port. Extend `draw()` (after the reload button block) with a serial section:

```cpp
    y -= 22;
    // ARM state + serial status
    button(x, y, data_.armed ? "[ DISARM ]" : "[ ARM ]", UI_ARM_TOGGLE,
           data_.armed ? 1.0f : 0.5f, data_.armed ? 0.4f : 1.0f, 0.4f);
    {
        char sb[128];
        std::snprintf(sb, sizeof(sb), "   %s  %s  frames %llu",
                      data_.armed ? "ARMED" : "disarmed",
                      data_.serialConnected ? "CONNECTED" : "no link",
                      (unsigned long long)data_.framesSent);
        drawString(x + 90, y, sb, data_.serialConnected ? 0.6f : 0.8f,
                   data_.serialConnected ? 1.0f : 0.7f, 0.6f);
    }
    y -= 18;
    std::snprintf(buf, sizeof(buf), "port: %s",
                  data_.serialPort.empty() ? "(none - pick below)" : data_.serialPort.c_str());
    drawString(x, y, buf, 0.8f, 0.8f, 0.9f);
    y -= 16;
    int px = x;
    px += button(px, y, "[ Rescan ]", UI_RESCAN_PORTS, 0.7f, 0.8f, 0.9f) + gap;
    // (Rescan handled below in mouse dispatch too - see note)
    y -= 16;
    for (const auto& p : ports_) {
        bool sel = (p == data_.serialPort);
        std::snprintf(buf, sizeof(buf), "%s %s", sel ? ">" : " ", p.c_str());
        portButton(x, y, buf, p, sel ? 1.0f : 0.7f, sel ? 1.0f : 0.7f, sel ? 0.4f : 0.7f);
        y -= 16;
    }
```
Make `UI_RESCAN_PORTS` also refresh the window's own list: in `mouseCallback`/command dispatch, when `b.action == UI_RESCAN_PORTS` call `rescanPorts()` before forwarding (or handle locally). Simplest: in `mouseCallback`, if `b.action == UI_RESCAN_PORTS` call `this->rescanPorts();` and still forward to `commandCallback_`. Widen/enlarge the window: set `params.bottom = 40;` in `initialize()` to fit the serial section (top stays 500).

Bump the title string to `"Motion Provider v0.6 (Phase 4)"`.

- [ ] **Step 3: Wire the window callbacks in `src/MotionProvider.cpp` `initialize()`:**

```cpp
    statusWindow_->setPortSelectedCallback([this](const std::string& p){ selectPort(p); });
```
(The existing `setCommandCallback` already routes `UI_ARM_TOGGLE`/`UI_RESCAN_PORTS` into `onUiAction`.)

- [ ] **Step 4: Commit**

```bash
git add MotionProviderPlugin/src/StatusWindow.h MotionProviderPlugin/src/StatusWindow.cpp MotionProviderPlugin/src/MotionProvider.cpp
git commit -m "feat(motion): Phase 4 Task 4 - port chooser, ARM/DISARM, serial status in window"
```

- [ ] **Step 5: Manual verification (user)** — build + load with the MotionGateway connected (in BFF/mode1). Window shows a port list; click your gateway's `/dev/cu.*`; status → CONNECTED, frames counting; while **disarmed** the gateway receives home (32640). Click **ARM** → live setpoints stream; fly and confirm the platform tracks (through the gateway → actors). **DISARM** → smooth return to home. Unplug/replug → auto-reconnect within ~2 s. Confirm the commanded pose never flags unreachable (envelope clamp). Tune `[serial]`/`[safety]` in the TOML + Reload.

**SAFETY for first real-actuator run:** start DISARMED, verify home holds, keep the e-stop/power within reach, and ARM only once the gateway shows the actors at home. Velocity/accel limits default conservative; raise them in `[safety]` only after the motion looks sane.

---

## Self-Review

- **Spec coverage:** spec §4 Phase 4 (serial output, port chooser in settings, I/O on a secondary thread, transport format) — Tasks 1/3/4; the reordered safety core (spec §1: safety lands with first serial output) — Task 2 (envelope clamp + rate limiting) + the ARM gate (Task 3). BFF frame per spec §5 = the gateway's existing decode (no gateway change).
- **Placeholder scan:** no TBD/TODO. `rescanPorts()` on MotionProvider is intentionally a no-op (the window owns port enumeration); documented. Engine/buffet effects remain the only reserved items (Phase 3), untouched here.
- **Type consistency:** `SerialLink::setFrame` takes the 16-byte `BffEncoder::kFrameSize` frame; `MotionProvider` encodes `sentSetpoints_` (BFF order) via `BffEncoder::encode`. `SafetyLimiter::limit(const uint16_t[6], double, uint16_t[6])` and `reset(const uint16_t[6])` used consistently. `clampToReachable(const Pose&) const` added to the same `StewartKinematics` used everywhere. StatusData's new fields are populated in `pushStatus` and read in `draw`. UI actions `UI_ARM_TOGGLE`/`UI_RESCAN_PORTS` handled in `onUiAction`; port buttons dispatch via the separate `portSelectedCallback_`.
- **Constraint check:** `BffEncoder`, `SafetyLimiter`, configs, and `clampToReachable` are XPLM-free (native tests link them). Serial I/O is on `SerialLink`'s thread only; the flight loop calls `setFrame` (mutex) + `update` (reconnect) — never blocks on serial. Only `Plugin.cpp` touches the ABI; `DataRefManager` remains the only dataref reader.
- **Safety:** disarmed default + home stream + smooth ramped disarm via the limiter + envelope clamp before IK + conservative default rate limits = first actuator motion is bounded and operator-gated. Runaway detection / watchdog / soft-start refinements remain Phase 5.
- **Test note:** BFF encode is round-trip tested against the gateway's exact decode; SafetyLimiter tested for velocity-bounded ramp, range clamp, and convergence; clampToReachable tested for unreachable→reachable scaling and reachable-passthrough. Serial/thread I/O and X-Plane are manual (sandbox can't run them).
