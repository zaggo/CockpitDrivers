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
    rxBuffer_.clear();  // Clear any partial frames
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
        std::vector<uint8_t> frame = queue.peekTxFrame();
        if (frame.empty()) {
            break;
        }
        bool writeOk = serial_.writeBestEffort(frame.data(), frame.size());
        lastWriteOk_ = writeOk;
        if (writeOk) {
            lastTxTime_ = static_cast<float>(std::time(nullptr));
            queue.popTxFrame();
        } else {
            // Bei Fehlschlag Frame nicht entfernen, später erneut versuchen
            break;
        }
    }
    
    // ============ RX: Read incoming data ============
    uint8_t tempBuffer[256];
    size_t rxLen = serial_.readNonBlocking(tempBuffer, sizeof(tempBuffer));
    
    if (rxLen > 0) {
        lastRxTime_ = static_cast<float>(std::time(nullptr));
        
        // Append new data to reassembly buffer
        rxBuffer_.insert(rxBuffer_.end(), tempBuffer, tempBuffer + rxLen);
        
        // Try to decode all complete frames from buffer
        while (rxBuffer_.size() >= 4) {  // Minimum frame size
            // Check for sync bytes
            if (rxBuffer_[0] != 0xAA || rxBuffer_[1] != 0x55) {
                // Invalid sync - discard first byte and try again
                rxBuffer_.erase(rxBuffer_.begin());
                continue;
            }
            
            // We have valid sync bytes, check if we have complete frame
            uint8_t payloadLen = rxBuffer_[3];
            size_t frameSize = 4 + payloadLen;
            
            if (rxBuffer_.size() < frameSize) {
                // Incomplete frame - wait for more data
                break;
            }
            
            // Try to decode the frame
            auto msg = TransportLayer::decodeFrame(rxBuffer_.data(), frameSize);
            if (msg) {
                queue.enqueueRx(*msg);
                // Remove processed frame from buffer
                rxBuffer_.erase(rxBuffer_.begin(), rxBuffer_.begin() + frameSize);
            } else {
                // Decode failed despite having enough bytes - discard first byte
                rxBuffer_.erase(rxBuffer_.begin());
            }
        }
        
        // Prevent buffer from growing indefinitely
        if (rxBuffer_.size() > MAX_RX_BUFFER) {
            XPLMDebugString("DCUProvider: RX buffer overflow, clearing\n");
            rxBuffer_.clear();
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