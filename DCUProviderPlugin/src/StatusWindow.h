
#pragma once
#include "XPLMDisplay.h"
#include "XPLMMenus.h"
#include "XPLMGraphics.h"
#include <string>
#include <cstdint>
#include <functional>
#include <vector>

// A single pre-formatted display line, built once when StatusData changes
// (~1 Hz) instead of every render frame in the draw callback.
struct StatusLine {
    std::string text;
    float r = 0.7f, g = 0.7f, b = 0.7f;
};

// UI button actions, dispatched to the owner via the command callback.
enum {
    UI_RESCAN_PORTS = 1,  // re-enumerate serial ports
    UI_DISCONNECT         // close the current serial connection
};

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

    // Show transient "Ports rescanned" confirmation next to the Rescan button
    bool rescanFlash = false;
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

    // Set callback invoked whenever the window transitions from hidden to visible,
    // so the caller can rescan for serial ports that appeared in the meantime.
    void setWindowShownCallback(std::function<void()> cb);

    // Set callback for UI button actions (UI_RESCAN_PORTS, UI_DISCONNECT).
    void setCommandCallback(std::function<void(int)> cb);

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

    // Clickable region, rebuilt every draw() frame. A non-empty port means
    // "select this port"; otherwise action is dispatched via commandCallback_.
    struct Button {
        int left, top, right, bottom;
        int action;
        std::string port;
    };

    // Internal drawing
    void draw();
    void drawText(int x, int y);
    void drawString(int x, int y, const std::string& text, float r, float g, float b);

    // Draw a clickable label and register its hitbox. Returns the label width
    // in pixels so callers can lay out buttons side by side.
    int button(int x, int y, const std::string& label, int action,
               float r, float g, float b);
    int portButton(int x, int y, const std::string& label, const std::string& port,
                   float r, float g, float b);

    // Rebuild cachedLines_ from statusData_. Called from update() (~1 Hz),
    // NOT from draw() (every render frame) - formatting is comparatively
    // expensive and the underlying data only changes this often anyway.
    void rebuildCachedLines();

    // Member variables
    std::vector<std::string> availablePorts_;
    std::vector<Button> buttons_;
    std::function<void(const std::string&)> portChangedCallback_;
    std::function<void()> windowShownCallback_;
    std::function<void(int)> commandCallback_;
    XPLMWindowID windowId_;
    int menuItemIdx_;
    XPLMMenuID pluginMenuId_;
    StatusData statusData_;
    StatusLine connStatusLine_;
    std::vector<StatusLine> cachedLines_;

    // Last visibility state we know about; used to detect changes made by
    // X-Plane's native close button, which has no callback of its own.
    bool lastKnownVisible_ = false;
};