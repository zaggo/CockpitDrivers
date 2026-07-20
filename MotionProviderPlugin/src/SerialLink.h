#pragma once
#include "SerialPort.h"
#include "BffEncoder.h"
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <cstdint>
#include "HeartbeatDecoder.h"

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
    bool heartbeatArmed() const { return hbArmed_.load(); }
    bool heartbeatFresh(double maxAgeSec) const;

private:
    void startIoThread();
    void stopIoThread();
    void ioThreadLoop();
    void rxThreadLoop();

    SerialPort serial_;
    std::string port_;
    int baud_ = 115200;
    double rateHz_ = 60.0;

    std::thread ioThread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::atomic<uint64_t> frames_{0};

    std::thread rxThread_;
    HeartbeatDecoder rxDecoder_;                 // touched only by the RX thread
    std::atomic<bool> hbArmed_{false};
    std::atomic<long long> hbLastMicros_{0};     // steady_clock micros of last valid frame; 0 = never

    std::mutex frameMutex_;
    uint8_t frame_[BffEncoder::kFrameSize] = {0};
    bool haveFrame_ = false;

    float reconnectAccum_ = 0.0f;
    static constexpr float kReconnectInterval = 2.0f;  // s

    // Opening the port pulses DTR, which resets the MotionGateway AVR (same
    // issue DCUProviderPlugin works around). Hold off streaming for this long
    // after connect so the gateway's boot/CAN-reinit window doesn't overlap
    // with the actor heartbeat timeout on the other side. Lives in the
    // dedicated I/O thread (not flight-loop update()), so it never blocks X-Plane.
    static constexpr int kPostConnectSettleMs = 1200;
};
