// Headless SerialLink test harness: drives SerialLink::setFrame() at a
// synthetic flight-loop cadence against a real port or a socat pty, so the TX
// scheduler can be measured and regression-tested without X-Plane.
//
// Build (from MotionProviderPlugin/tools):
//   clang++ -std=c++17 -O2 -I../src bff_sender_test.cpp \
//     ../src/SerialLink.cpp ../src/SerialPort.cpp ../src/BffEncoder.cpp \
//     ../src/HeartbeatDecoder.cpp -o bff_sender_test -pthread
//
// Test setup (two terminals):
//   socat -d -d pty,raw,echo=0 pty,raw,echo=0     # prints two /dev/ttysNNN
//   python3 measure_bff_timing.py /dev/ttysAAA     # measurer on one end
//   ./bff_sender_test /dev/ttysBBB 60 30           # sender on the other
//
// The sender feeds a 60 Hz sine, pauses for 1 s halfway through (the measurer
// must show ~10 Hz keepalive duplicates there, not silence), then resumes.
//
// Usage: bff_sender_test <port> [rateHz=60] [durationSec=30]

#include "SerialLink.h"
#include "BffEncoder.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <thread>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <port> [rateHz=60] [durationSec=30]\n", argv[0]);
        return 2;
    }
    const std::string port = argv[1];
    const double rateHz = argc > 2 ? std::atof(argv[2]) : 60.0;
    const double durationSec = argc > 3 ? std::atof(argv[3]) : 30.0;

    SerialLink link;
    link.configure(port, 115200, rateHz);
    if (!link.connect()) {
        std::fprintf(stderr, "failed to open %s\n", port.c_str());
        return 1;
    }
    std::printf("connected to %s, feeding %g Hz for %g s (1 s pause at t=%g s)\n",
                port.c_str(), rateHz, durationSec, durationSec / 2.0);

    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::microseconds(
        static_cast<long long>(1'000'000.0 / rateHz));
    const auto start = clock::now();
    const auto pauseAt = start + std::chrono::milliseconds(
        static_cast<long long>(durationSec * 500.0));
    const auto pauseEnd = pauseAt + std::chrono::seconds(1);
    auto next = start;
    bool pausedOnce = false;
    uint64_t fed = 0;

    while (true) {
        const auto now = clock::now();
        const double t = std::chrono::duration<double>(now - start).count();
        if (t >= durationSec) break;

        if (!pausedOnce && now >= pauseAt) {
            // Simulate a flight-loop stall: no setFrame for 1 s. The link must
            // keep the gateway fed with ~10 Hz keepalives of the last frame.
            std::this_thread::sleep_until(pauseEnd);
            next = clock::now();
            pausedOnce = true;
            continue;
        }
        if (now < next) {
            std::this_thread::sleep_until(next);
            continue;
        }
        next += period;  // absolute deadline, no drift

        const double phase = 2.0 * M_PI * t / 6.0;  // 6 s wave like the benches
        const uint16_t value = static_cast<uint16_t>(
            32640.0 + 0.15 * 65280.0 * std::sin(phase));
        uint16_t setpoints[6] = {value, value, value, value, value, value};
        uint8_t frame[BffEncoder::kFrameSize];
        BffEncoder::encode(setpoints, frame);
        link.setFrame(frame, sizeof(frame));
        ++fed;
    }

    // Give the TX thread a moment to flush the last frame, then report.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::printf("fed %llu frames, link sent %llu\n",
                static_cast<unsigned long long>(fed),
                static_cast<unsigned long long>(link.framesSent()));
    link.stop();
    return 0;
}
