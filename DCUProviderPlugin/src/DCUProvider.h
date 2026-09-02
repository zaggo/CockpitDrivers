
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
    
    /// Called when a new aircraft is loaded.
    /// Reinitializes datarefs to match the new aircraft.
    void onAircraftLoaded();
    
    // ============ Status ============
    
    /// Returns true if serial connection to gateway is active.
    bool isConnected() const;
    
private:
    // Current serial port path
    std::string currentPort_;

    // Helper to change port and reconnect
    void changePort(const std::string& newPort);
    // Rescan available serial ports and refresh the status window's list.
    // Called whenever the status window is (re-)shown or Rescan is clicked.
    void refreshPorts();
    // Deliberate manual disconnect from the status window: tears down the
    // connection and clears currentPort_, so clicking any port (including the
    // same one) reconnects. The saved last-used port is left untouched.
    void disconnectPort();
    // Dispatch a status-window button action (UI_RESCAN_PORTS / UI_DISCONNECT).
    void onUiAction(int action);
    // ============ Internal Updates ============
    
    /// Downlink: Read data from X-Plane datarefs and queue messages for gateway.
    /// Handles rate limiting (e.g., 5 Hz for fuel, 2 Hz for lights).
    /// 
    /// @param dt Delta time since last call (seconds)
    void updateDownlink(float dt);
    
    /// Uplink: Process received messages from gateway and write to X-Plane datarefs.
    void updateUplink();
    
    /// Claim or release X-Plane's rudder/toe-brake axes depending on whether the
    /// RudderCAN node is still reporting over a live link.
    ///
    /// @param dt Delta time since last call (seconds)
    void updateRudderOverride(float dt);

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
    float rpmAccumulator_ = 0.0f;
    float odometerAccumulator_ = 0.0f;
    float airspeedAccumulator_ = 0.0f;
    float altimeterVsiAccumulator_ = 0.0f;

    static constexpr float FUEL_RATE = 5.0f;    // Hz
    static constexpr float LIGHTS_RATE = 10.0f;  // Hz
    static constexpr float TRANSPONDER_RATE = 10.0f; // Hz
    static constexpr float RPM_RATE = 50.0f;      // Hz
    static constexpr float ODOMETER_RATE = 10.0f; // Hz
    static constexpr float AIRSPEED_RATE = 50.0f; // Hz
    static constexpr float ALTIMETER_VSI_RATE = 50.0f; // Hz

    // ============ Rudder Override Watchdog ============

    // Time without a rudder frame after which the joystick override is released.
    // Without this, unplugging the DCU would leave the sim with dead rudder pedals
    // and no way to fly. Generous next to the node's 2s periodic refresh.
    static constexpr float RUDDER_SIGNAL_TIMEOUT = 3.0f; // seconds

    // Starts timed out, so the override is only ever claimed after real data arrives.
    float rudderSilenceAccumulator_ = RUDDER_SIGNAL_TIMEOUT;

    // Seconds left to show the "Ports rescanned" confirmation in the status window.
    float rescanFlashRemaining_ = 0.0f;
};