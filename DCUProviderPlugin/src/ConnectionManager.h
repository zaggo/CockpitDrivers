#pragma once

#include "SerialPort.h"
#include "TransportLayer.h"
#include "MessageQueue.h"
#include "XPLMUtilities.h"
#include <string>
#include <ctime>
#include <thread>
#include <atomic>

class ConnectionManager {
public:
    /// Constructor
    /// @param devicePath Serial device path (e.g., "/dev/cu.usbserial-1440")
    /// @param baudRate Baud rate (e.g., 115200)
    /// @param queue Message queue to send/receive from. Must outlive this
    ///        ConnectionManager - the I/O thread reads/writes it directly.
    ConnectionManager(const std::string& devicePath, int baudRate, MessageQueue& queue);

    ~ConnectionManager();

    // Prevent copying
    ConnectionManager(const ConnectionManager&) = delete;
    ConnectionManager& operator=(const ConnectionManager&) = delete;

    // ============ Connection Lifecycle ============

    /// Attempts to open the serial port.
    /// Returns true on success, false on failure.
    /// Updates lastOpenOk_ flag.
    /// On success, starts the dedicated I/O thread.
    bool connect();

    /// Stops the I/O thread (if running) and closes the serial port.
    void disconnect();

    /// Returns true if the port is currently open.
    bool isConnected() const;

    // ============ Connection Management ============

    /// Call this regularly from flight loop (e.g., every frame or 1Hz).
    /// Handles automatic reconnection attempts.
    ///
    /// @param dt Delta time since last call (seconds)
    void update(float dt);

    // ============ Statistics / Status ============

    /// Timestamp of last successful TX
    float getLastTxTime() const { return lastTxTime_.load(); }

    /// Timestamp of last successful RX
    float getLastRxTime() const { return lastRxTime_.load(); }

    /// True if last write attempt succeeded
    bool getLastWriteOk() const { return lastWriteOk_.load(); }

    /// True if last open attempt succeeded
    bool getLastOpenOk() const { return lastOpenOk_.load(); }

private:
    /// Internal reconnection attempt with logging
    void attemptReconnect();

    /// Starts ioThread_ running ioThreadLoop().
    void startIoThread();

    /// Signals ioThreadLoop() to stop and joins the thread. Safe to call
    /// even if no thread is running.
    void stopIoThread();

    /// Runs on a dedicated thread while connected: sends queued TX frames and
    /// does a bounded blocking read (SerialPort::readBlocking) to reassemble
    /// incoming frames. Kept off the flight-loop thread so USB/serial driver
    /// overhead never eats into X-Plane's frame budget.
    void ioThreadLoop();

    SerialPort serial_;
    std::string devicePath_;
    int baudRate_;
    MessageQueue& queue_;

    std::thread ioThread_;
    std::atomic<bool> ioThreadRunning_{false};

    // Reconnection logic
    float reconnectAccumulator_ = 0.0f;
    static constexpr float RECONNECT_INTERVAL = 2.0f;

    // RX Frame reassembly buffer (only touched by ioThread_)
    std::vector<uint8_t> rxBuffer_;
    static constexpr size_t MAX_RX_BUFFER = 2048;

    // Statistics - written from ioThread_/connect(), read from the main
    // thread via the getters above, hence atomic.
    std::atomic<float> lastTxTime_{0.0f};
    std::atomic<float> lastRxTime_{0.0f};
    std::atomic<bool> lastWriteOk_{false};
    std::atomic<bool> lastOpenOk_{false};
};