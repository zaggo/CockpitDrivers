
#include "MessageQueue.h"

void MessageQueue::clearTxQueue() {
    std::lock_guard<std::mutex> lock(txMutex_);
    while (!txQueue_.empty()) {
        txQueue_.pop();
    }
}

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


std::vector<uint8_t> MessageQueue::peekTxFrame() const {
    std::lock_guard<std::mutex> lock(txMutex_);
    if (txQueue_.empty()) {
        return {};
    }
    return txQueue_.front();
}

void MessageQueue::popTxFrame() {
    std::lock_guard<std::mutex> lock(txMutex_);
    if (txQueue_.empty()) return;
    txBytesSent_ += txQueue_.front().size();
    txQueue_.pop();
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