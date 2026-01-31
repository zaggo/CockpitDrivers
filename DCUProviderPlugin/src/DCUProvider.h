
#pragma once
#include "SerialPortUtils.h"
#include "SerialPort.h"
#include "ConnectionManager.h"
#include "DataRefManager.h"
#include "MessageQueue.h"
#include "StatusWindow.h"
#include <memory>

class DCUProvider {
public:
    DCUProvider();
    ~DCUProvider();
    
    // Prevent copying
    DCUProvider(const DCUProvider&) = delete;
    DCUProvider& operator=(const DCUProvider&) = delete;
    
    // ============ Lifecycle ============
    
    /// Initialize all components.
    /// Must be called once during plugin startup.
    /// Returns true on success (even if connection fails).
    bool initialize();
    
    /// Shutdown all components.
    /// Called automatically in destructor.
    void shutdown();
    
    // ============ Main Loop ============
    
    /// Called every frame from X-Plane flight loop.
    /// Handles all I/O, downlink, uplink, and status updates.
    /// 
    /// @param elapsedTime Time since last frame (seconds)
    void onFlightLoopTick(float elapsedTime);
    
    // ============ Status ============
    
    /// Returns true if serial connection to gateway is active.
    bool isConnected() const;
    
private:
    // Current serial port path
    std::string currentPort_;

    // Helper to change port and reconnect
    void changePort(const std::string& newPort);
    // ============ Internal Updates ============
    
    /// Downlink: Read data from X-Plane datarefs and queue messages for gateway.
    /// Handles rate limiting (e.g., 5 Hz for fuel, 2 Hz for lights).
    /// 
    /// @param dt Delta time since last call (seconds)
    void updateDownlink(float dt);
    
    /// Uplink: Process received messages from gateway and write to X-Plane datarefs.
    void updateUplink();
    
    /// Update status window display (every 1 second).
    void updateStatusWindow();
    
    // ============ Components ============
    
    std::unique_ptr<DataRefManager> dataRefMgr_;
    std::unique_ptr<MessageQueue> msgQueue_;
    std::unique_ptr<ConnectionManager> connMgr_;
    std::unique_ptr<StatusWindow> statusWin_;
    
    // ============ Rate Limiting Accumulators ============
    
    float fuelAccumulator_ = 0.0f;
    float lightsAccumulator_ = 0.0f;
    float transponderAccumulator_ = 0.0f;
    
    static constexpr float FUEL_RATE = 5.0f;    // Hz
    static constexpr float LIGHTS_RATE = 10.0f;  // Hz
    static constexpr float TRANSPONDER_RATE = 10.0f; // Hz
};