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
    if (rxThread_.joinable()) rxThread_.join();  // reap a self-exited RX thread
    rxDecoder_.reset();
    rxThread_ = std::thread(&SerialLink::rxThreadLoop, this);
}

void SerialLink::stopIoThread() {
    if (ioThread_.joinable()) {
        running_.store(false);
        frameCv_.notify_all();  // wake the TX thread out of its wait
        ioThread_.join();
    }
    if (rxThread_.joinable()) {
        running_.store(false);
        rxThread_.join();
    }
}

void SerialLink::ioThreadLoop() {
    // Let the gateway's AVR finish its reset/boot before we start writing to
    // it (see kPostConnectSettleMs). Chunked so stop() stays responsive.
    for (int waited = 0; waited < kPostConnectSettleMs && running_.load(std::memory_order_relaxed); waited += 50) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Event-driven TX: send on setFrame() notification (rate-capped at rateHz_),
    // re-send the last frame as keepalive when the flight loop goes quiet.
    const auto minGap = std::chrono::microseconds(
        static_cast<long long>(1'000'000.0 / (rateHz_ > 0.0 ? rateHz_ : 60.0)));
    const auto keepalive = std::chrono::milliseconds(kKeepaliveMs);

    auto lastSend = std::chrono::steady_clock::now() - minGap;
    std::unique_lock<std::mutex> lk(frameMutex_);
    while (running_.load(std::memory_order_relaxed) && serial_.isOpen()) {
        frameCv_.wait_until(lk, lastSend + keepalive, [&] {
            return frameDirty_ || !running_.load(std::memory_order_relaxed);
        });
        if (!running_.load(std::memory_order_relaxed)) break;
        if (!haveFrame_) { lastSend = std::chrono::steady_clock::now(); continue; }

        // Rate cap: a fresh frame earlier than minGap after the last write
        // waits out the remainder (latest-wins - the frame under the lock may
        // be replaced meanwhile, which is exactly what we want).
        auto now = std::chrono::steady_clock::now();
        if (frameDirty_ && now < lastSend + minGap) {
            frameCv_.wait_until(lk, lastSend + minGap, [&] {
                return !running_.load(std::memory_order_relaxed);
            });
            if (!running_.load(std::memory_order_relaxed)) break;
            now = std::chrono::steady_clock::now();
        }

        uint8_t local[BffEncoder::kFrameSize];
        std::memcpy(local, frame_, sizeof(local));
        frameDirty_ = false;
        lk.unlock();  // never hold the mutex across the write syscall
        if (serial_.writeBestEffort(local, sizeof(local))) {
            frames_.fetch_add(1, std::memory_order_relaxed);
        }
        // writeBestEffort closes the port on hard error; loop guard exits.
        lk.lock();
        lastSend = now;
    }
    lk.unlock();
    connected_.store(false);  // port dropped or stopped; update() will reconnect
}

void SerialLink::rxThreadLoop() {
    uint8_t buf[64];
    while (running_.load(std::memory_order_relaxed) && serial_.isOpen()) {
        // Blocks up to SerialPort's internal read timeout (~20ms), then returns
        // whatever arrived (0 on timeout). Low-CPU wait, not a busy-poll.
        const std::size_t n = serial_.readBlocking(buf, sizeof(buf));
        for (std::size_t i = 0; i < n; ++i) {
            if (rxDecoder_.feed(buf[i])) {
                hbArmed_.store(rxDecoder_.armed(), std::memory_order_relaxed);
                const auto now = std::chrono::steady_clock::now().time_since_epoch();
                const long long us =
                    std::chrono::duration_cast<std::chrono::microseconds>(now).count();
                hbLastMicros_.store(us, std::memory_order_relaxed);
            }
        }
    }
}

bool SerialLink::heartbeatFresh(double maxAgeSec) const {
    const long long last = hbLastMicros_.load(std::memory_order_relaxed);
    if (last == 0) return false;   // never received one
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const long long us =
        std::chrono::duration_cast<std::chrono::microseconds>(now).count();
    return (us - last) <= static_cast<long long>(maxAgeSec * 1e6);
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
    {
        std::lock_guard<std::mutex> lk(frameMutex_);
        std::memcpy(frame_, data, len);
        haveFrame_ = true;
        frameDirty_ = true;
    }
    frameCv_.notify_one();  // TX thread writes it immediately (rate-capped)
}
