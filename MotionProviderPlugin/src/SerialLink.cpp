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
