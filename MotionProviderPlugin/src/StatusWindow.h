#pragma once
#include "XPLMDisplay.h"
#include "XPLMMenus.h"
#include "XPLMGraphics.h"
#include "StatusData.h"
#include "SerialPortUtils.h"
#include <string>
#include <functional>
#include <vector>

class StatusWindow {
public:
    StatusWindow();
    ~StatusWindow();

    StatusWindow(const StatusWindow&) = delete;
    StatusWindow& operator=(const StatusWindow&) = delete;

    void initialize();
    void destroy();
    void setVisible(bool visible);
    bool isVisible() const;

    // Refresh the displayed snapshot (called ~1 Hz and on every UI action).
    void update(const StatusData& data);

    // Invoked when the user clicks an on-screen button; arg is a UiAction code.
    void setCommandCallback(std::function<void(int)> cb);

    // Invoked when the user selects a serial port.
    void setPortSelectedCallback(std::function<void(const std::string&)> cb);

    // Refresh the cached port list.
    void rescanPorts();

private:
    static void drawCallback(XPLMWindowID inWindowID, void* inRefcon);
    static void keyCallback(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags,
                            char inVirtualKey, void* inRefcon, int losingFocus);
    static void menuCallback(void* inMenuRef, void* inItemRef);

    void draw();
    void drawString(int x, int y, const std::string& text, float r, float g, float b);
    void mouseCallback(XPLMWindowID, int x, int y, XPLMMouseStatus, void*);

    // Draw a clickable button at (x, baseline y), record its hitbox + action,
    // return its pixel width so the caller can place the next one after it.
    int button(int x, int y, const std::string& label, int action,
               float r, float g, float b);

    // Draw a clickable port button at (x, baseline y).
    int portButton(int x, int y, const std::string& label, const std::string& port,
                   float r, float g, float b);

    struct Button { int left, top, right, bottom, action; std::string port; };

    XPLMWindowID windowId_;
    int menuItemIdx_;
    XPLMMenuID pluginMenuId_;
    bool lastKnownVisible_;

    StatusData data_;
    std::function<void(int)> commandCallback_;
    std::function<void(const std::string&)> portSelectedCallback_;
    std::vector<Button> buttons_;   // rebuilt every draw()
    std::vector<std::string> ports_;  // cached serial ports
};
