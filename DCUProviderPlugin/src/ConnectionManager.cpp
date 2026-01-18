#include "ConnectionManager.h"
#include <cstdio>
#include <ctime>

ConnectionManager::ConnectionManager(const std::string& devicePath, int baudRate)
    : devicePath_(devicePath), baudRate_(baudRate) {
}

ConnectionManager::~ConnectionManager() {
    disconnect();
}

bool ConnectionManager::connect() {
    if (isConnected()) {
        return true;
    }
    
    bool success = serial_.openPort(devicePath_, baudRate_);
    lastOpenOk_ = success;
    
    if (success) {
        reconnectAccumulator_ = 0.0f;
    }
    
    return success;
}

void ConnectionManager::disconnect() {
    serial_.closePort();
    reconnectAccumulator_ = 0.0f;
}

bool ConnectionManager::isConnected() const {
    return serial_.isOpen();
}

void ConnectionManager::processIO(MessageQueue& queue) {
    if (!isConnected()) {
        return;
    }
    
    // ============ TX: Send queued messages ============
    while (queue.hasTxPending()) {
        auto frame = queue.dequeueTxFrame();
        if (frame.empty()) {
            continue;
        }
        
        bool writeOk = serial_.writeBestEffort(frame.data(), frame.size());
        lastWriteOk_ = writeOk;
        
        if (writeOk) {
            lastTxTime_ = static_cast<float>(std::time(nullptr));
        }
    }
    
    // ============ RX: Read incoming data ============
    uint8_t rxBuffer[256];
    size_t rxLen = serial_.readNonBlocking(rxBuffer, sizeof(rxBuffer));
    
    if (rxLen > 0) {
        lastRxTime_ = static_cast<float>(std::time(nullptr));
        
        // Try to decode frame(s)
        // Note: For now, assume one complete frame per read
        // TODO: Implement frame reassembly for partial frames
        auto msg = TransportLayer::decodeFrame(rxBuffer, rxLen);
        if (msg) {
            queue.enqueueRx(*msg);
        }
    }
}

void ConnectionManager::update(float dt) {
    // Accumulate time
    reconnectAccumulator_ += dt;
    
    // If connected, reset accumulator
    if (isConnected()) {
        reconnectAccumulator_ = 0.0f;
        return;
    }
    
    // Not connected: try to reconnect periodically
    if (reconnectAccumulator_ >= RECONNECT_INTERVAL) {
        attemptReconnect();
        reconnectAccumulator_ = 0.0f;
    }
}

void ConnectionManager::attemptReconnect() {
    // Try to connect
    bool success = connect();
    
    // Log attempt (optional)
    if (success) {
        XPLMDebugString("DCUProvider: Reconnection successful\n");
    } else {
        XPLMDebugString("DCUProvider: Reconnection failed\n");
    }
}