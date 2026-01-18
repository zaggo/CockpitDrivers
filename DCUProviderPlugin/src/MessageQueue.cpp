#include "MessageQueue.h"

void MessageQueue::enqueueTx(MessageType type, const void* payload, uint8_t len) {
    // Encode frame
    auto frame = TransportLayer::encodeFrame(type, payload, len);
    
    if (frame.empty()) {
        return;  // Encoding failed (payload too large)
    }
    
    // Enqueue
    {
        std::lock_guard<std::mutex> lock(txMutex_);
        txQueue_.push(std::move(frame));
    }
}

std::vector<uint8_t> MessageQueue::dequeueTxFrame() {
    std::lock_guard<std::mutex> lock(txMutex_);
    
    if (txQueue_.empty()) {
        return {};
    }
    
    auto frame = std::move(txQueue_.front());
    txQueue_.pop();
    
    txBytesSent_ += frame.size();
    
    return frame;
}

bool MessageQueue::hasTxPending() const {
    std::lock_guard<std::mutex> lock(txMutex_);
    return !txQueue_.empty();
}

void MessageQueue::enqueueRx(const Message& msg) {
    {
        std::lock_guard<std::mutex> lock(rxMutex_);
        rxQueue_.push(msg);
    }
    
    rxBytesReceived_ += (4 + msg.payload.size());  // Frame overhead
}

std::optional<Message> MessageQueue::dequeueRx() {
    std::lock_guard<std::mutex> lock(rxMutex_);
    
    if (rxQueue_.empty()) {
        return std::nullopt;
    }
    
    auto msg = std::move(rxQueue_.front());
    rxQueue_.pop();
    
    return msg;
}

bool MessageQueue::hasRxPending() const {
    std::lock_guard<std::mutex> lock(rxMutex_);
    return !rxQueue_.empty();
}