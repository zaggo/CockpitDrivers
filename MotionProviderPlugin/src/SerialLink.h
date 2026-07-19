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
