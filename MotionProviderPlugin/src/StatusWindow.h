#pragma once
#include "XPLMDisplay.h"
#include "XPLMMenus.h"
#include "XPLMGraphics.h"
#include <string>

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

    // Phase 0: no payload. Later phases pass a data struct.
    void update();

private:
    static void drawCallback(XPLMWindowID inWindowID, void* inRefcon);
    static void keyCallback(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags,
                            char inVirtualKey, void* inRefcon, int losingFocus);
    static void menuCallback(void* inMenuRef, void* inItemRef);

    void draw();
    void drawString(int x, int y, const std::string& text, float r, float g, float b);

    XPLMWindowID windowId_;
    int menuItemIdx_;
    XPLMMenuID pluginMenuId_;
    bool lastKnownVisible_;
};
