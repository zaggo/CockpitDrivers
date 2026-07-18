#pragma once
#include "XPLMDisplay.h"
#include "XPLMMenus.h"
#include "XPLMGraphics.h"
#include "StatusData.h"
#include <string>
#include <functional>

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

    // Refresh the displayed cue snapshot (called ~1 Hz).
    void update(const StatusData& data);
    void setReloadCallback(std::function<void()> cb);
    void setKeyCommandCallback(std::function<void(char)> cb);

private:
    static void drawCallback(XPLMWindowID inWindowID, void* inRefcon);
    static void keyCallback(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags,
                            char inVirtualKey, void* inRefcon, int losingFocus);
    static void menuCallback(void* inMenuRef, void* inItemRef);

    void draw();
    void drawString(int x, int y, const std::string& text, float r, float g, float b);
    void mouseCallback(XPLMWindowID, int x, int y, XPLMMouseStatus, void*);

    XPLMWindowID windowId_;
    int menuItemIdx_;
    XPLMMenuID pluginMenuId_;
    bool lastKnownVisible_;

    StatusData data_;
    std::function<void()> reloadCallback_;
    std::function<void(char)> keyCommandCallback_;
    int btnLeft_ = 0, btnTop_ = 0, btnRight_ = 0, btnBottom_ = 0; // reload button hitbox
};
