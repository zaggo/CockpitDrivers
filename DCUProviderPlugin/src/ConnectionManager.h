#pragma once

#include "SerialPort.h"
#include "TransportLayer.h"
#include "MessageQueue.h"
#include "XPLMUtilities.h"
#include <string>
#include <ctime>

class ConnectionManager {
public:
    /// Constructor
    /// @param devicePath Serial device path (e.g., "/dev/cu.usbserial-1440")
    /// @param baudRate Baud rate (e.g., 115200)
    explicit ConnectionManager(const std::string& devicePath, int baudRate);
    
    ~ConnectionManager();
    
    // Prevent copying
    ConnectionManager(const ConnectionManager&) = delete;
    ConnectionManager& operator=(const ConnectionManager&) = delete;
    
    // ============ Connection Lifecycle ============
    
    /// Attempts to open the serial port.
    /// Returns true on success, false on failure.
    /// Updates lastOpenOk_ flag.
    bool connect();
    
    /// Closes the serial port.
    void disconnect();
    
    /// Returns true if the port is currently open.
    bool isConnected() const;
    
    // ============ I/O Processing ============
    
    /// Processes both TX and RX.
    /// - Sends all queued TX frames to serial port
    /// - Reads incoming data and decodes frames into RX queue
    /// 
    /// Updates lastTxTime_ and lastRxTime_ on successful I/O.
    /// 
    /// @param queue Message queue to send/receive from
    void processIO(MessageQueue& queue);
    
    // ============ Connection Management ============
    
    /// Call this regularly from flight loop (e.g., every frame or 1Hz).
    /// Handles automatic reconnection attempts.
    /// 
    /// @param dt Delta time since last call (seconds)
    void update(float dt);
    
    // ============ Statistics / Status ============
    
    /// Timestamp of last successful TX
    float getLastTxTime() const { return lastTxTime_; }
    
    /// Timestamp of last successful RX
    float getLastRxTime() const { return lastRxTime_; }
    
    /// True if last write attempt succeeded
    bool getLastWriteOk() const { return lastWriteOk_; }
    
    /// True if last open attempt succeeded
    bool getLastOpenOk() const { return lastOpenOk_; }
    
private:
    /// Internal reconnection attempt with logging
    void attemptReconnect();
    
    SerialPort serial_;
    std::string devicePath_;
    int baudRate_;
    
    // Reconnection logic
    float reconnectAccumulator_ = 0.0f;
    static constexpr float RECONNECT_INTERVAL = 2.0f;
    
    // RX Frame reassembly buffer
    std::vector<uint8_t> rxBuffer_;
    static constexpr size_t MAX_RX_BUFFER = 2048;
    
    // Statistics
    float lastTxTime_ = 0.0f;
    float lastRxTime_ = 0.0f;
    bool lastWriteOk_ = false;
    bool lastOpenOk_ = false;
};