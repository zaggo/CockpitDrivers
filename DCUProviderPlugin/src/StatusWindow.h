
#pragma once
#include "XPLMDisplay.h"
#include "XPLMMenus.h"
#include "XPLMGraphics.h"
#include <string>
#include <cstdint>
#include <functional>
#include <vector>

struct StatusData {
    // Connection
    bool isConnected = false;
    std::string devicePath;
    int baudRate = 115200;
    
    // Statistics
    uint64_t txBytesSent = 0;
    uint64_t rxBytesReceived = 0;
    
    // Timing
    float lastTxTime = 0.0f;
    float lastRxTime = 0.0f;
    
    // Flags
    bool lastWriteOk = false;
    bool lastOpenOk = false;
};

class StatusWindow {
public:
    // Index des aktuell selektierten Ports (für Vorwahl beim Start)
    int selectedPortIdx_ = 0;
    StatusWindow();
    ~StatusWindow();

    // Prevent copying
    StatusWindow(const StatusWindow&) = delete;
    StatusWindow& operator=(const StatusWindow&) = delete;

    // Set available serial ports for selection in the UI
    void setAvailablePorts(const std::vector<std::string>& ports);

    // Set callback to notify when user selects a new port
    void setPortChangedCallback(std::function<void(const std::string&)> cb);

    /// Initialize window and menu item.
    /// Must be called during plugin startup.
    void initialize();

    /// Destroy window and menu item.
    /// Called automatically in destructor.
    void destroy();

    /// Show or hide the window.
    void setVisible(bool visible);

    /// Returns true if window is currently visible.
    bool isVisible() const;

    /// Update displayed status data.
    /// Call this regularly (e.g., every frame or 1Hz) to refresh display.
    /// @param data Current status data
    void update(const StatusData& data);

private:
    // X-Plane callback signatures (must return void, not int/XPLMCursorStatus)
    static void drawCallback(XPLMWindowID inWindowID, void* inRefcon);
    static void keyCallback(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags,
                            char inVirtualKey, void* inRefcon, int losingFocus);
    static void mouseCallback(XPLMWindowID inWindowID, int x, int y,
                              XPLMMouseStatus inMouse, void* inRefcon);
    static void menuCallback(void* inMenuRef, void* inItemRef);

    // Internal drawing
    void draw();
    void drawText(int x, int y);
    void drawString(int x, int y, const std::string& text, float r, float g, float b);

    // Member variables
    std::vector<std::string> availablePorts_;
    std::function<void(const std::string&)> portChangedCallback_;
    XPLMWindowID windowId_;
    int menuItemIdx_;
    XPLMMenuID pluginMenuId_;
    StatusData statusData_;
};