#pragma once

#include "TransportLayer.h"
#include <queue>
#include <mutex>
#include <atomic>
#include <optional>
#include <vector>
#include <cstdint>

class MessageQueue {
public:
    MessageQueue() = default;
    
    // Prevent copying
    MessageQueue(const MessageQueue&) = delete;
    MessageQueue& operator=(const MessageQueue&) = delete;
    
    // ============ TX Queue (Plugin → Gateway) ============
    
    /// Enqueues a message to send to the gateway.
    /// Internally encodes the message into a framed format.
    /// 
    /// @param type Message type
    /// @param payload Pointer to payload data (can be nullptr if len == 0)
    /// @param len Payload length (must be <= 255)
    void enqueueTx(MessageType type, const void* payload, uint8_t len);
    
    /// Returns the next TX frame without removing it (empty vector if none).
    std::vector<uint8_t> peekTxFrame() const;

    /// Removes the next TX frame and increments counter (call NUR bei Erfolg!)
    void popTxFrame();

    /// Entfernt alle TX-Frames aus der Queue (z.B. beim Portwechsel)
    void clearTxQueue();
    
    /// Returns true if there are pending TX frames.
    bool hasTxPending() const;
    
    // ============ RX Queue (Gateway → Plugin) ============
    
    /// Enqueues a received message from the gateway.
    /// Automatically increments rxBytesReceived_ counter.
    /// 
    /// @param msg Decoded message
    void enqueueRx(const Message& msg);
    
    /// Dequeues the next RX message.
    /// Returns std::nullopt if queue is empty.
    std::optional<Message> dequeueRx();
    
    /// Returns true if there are pending RX messages.
    bool hasRxPending() const;
    
    // ============ Statistics ============
    
    /// Total bytes sent (includes frame overhead: AA 55 type len)
    uint64_t getTxBytesSent() const { return txBytesSent_.load(); }

    /// Total bytes received (includes frame overhead)
    uint64_t getRxBytesReceived() const { return rxBytesReceived_.load(); }

    /// Reset statistics (optional)
    void resetStats() {
        txBytesSent_.store(0);
        rxBytesReceived_.store(0);
    }

private:
    // TX queue and synchronization. Producer: main/flight-loop thread.
    // Consumer: ConnectionManager's I/O thread.
    std::queue<std::vector<uint8_t>> txQueue_;
    mutable std::mutex txMutex_;
    std::atomic<uint64_t> txBytesSent_{0};

    // RX queue and synchronization. Producer: ConnectionManager's I/O thread.
    // Consumer: main/flight-loop thread.
    std::queue<Message> rxQueue_;
    mutable std::mutex rxMutex_;
    std::atomic<uint64_t> rxBytesReceived_{0};
};