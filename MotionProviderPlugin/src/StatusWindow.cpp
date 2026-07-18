#include "StatusWindow.h"
#include "ConfigUtils.h"
#include "XPLMUtilities.h"
#include <cstdio>

StatusWindow::StatusWindow()
    : windowId_(nullptr), menuItemIdx_(-1), pluginMenuId_(nullptr), lastKnownVisible_(false) {}

StatusWindow::~StatusWindow() {
    destroy();
}

void StatusWindow::initialize() {
    bool shouldBeVisible = loadStatusWindowVisible();

    XPLMCreateWindow_t params = {};
    params.structSize = sizeof(XPLMCreateWindow_t);
    params.left = 100;
    params.top = 500;
    params.right = 500;
    params.bottom = 170;
    params.visible = shouldBeVisible ? 1 : 0;
    params.drawWindowFunc = drawCallback;
    params.handleKeyFunc = keyCallback;
    params.handleMouseClickFunc = [](XPLMWindowID w, int x, int y, XPLMMouseStatus s, void* ref) -> int {
        StatusWindow* self = static_cast<StatusWindow*>(ref);
        if (self) self->mouseCallback(w, x, y, s, ref);
        return 1;
    };
    params.handleCursorFunc = nullptr;
    params.handleMouseWheelFunc = nullptr;
    params.handleRightClickFunc = nullptr;
    params.refcon = this;
    params.layer = xplm_WindowLayerFloatingWindows;
    params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;

    windowId_ = XPLMCreateWindowEx(&params);
    if (!windowId_) {
        XPLMDebugString("MotionProvider: Failed to create status window\n");
        return;
    }
    lastKnownVisible_ = shouldBeVisible;

    pluginMenuId_ = XPLMFindPluginsMenu();
    if (pluginMenuId_) {
        int subMenuIdx = XPLMAppendMenuItem(pluginMenuId_, "Motion Provider", nullptr, 1);
        XPLMMenuID ourMenuId = XPLMCreateMenu("Motion Provider", pluginMenuId_, subMenuIdx,
                                              menuCallback, this);
        XPLMAppendMenuItem(ourMenuId, "Show Status Window", this, 1);
        menuItemIdx_ = subMenuIdx;
        pluginMenuId_ = ourMenuId;
    }
    XPLMDebugString("MotionProvider: Status window initialized\n");
}

void StatusWindow::destroy() {
    if (windowId_) { XPLMDestroyWindow(windowId_); windowId_ = nullptr; }
    if (pluginMenuId_) { XPLMDestroyMenu(pluginMenuId_); pluginMenuId_ = nullptr; }
    // Remove our parent item from the Plugins menu. initialize() reassigned
    // pluginMenuId_ to our submenu, so re-fetch the Plugins menu (the item's
    // owner) to remove the entry at menuItemIdx_.
    if (menuItemIdx_ >= 0) {
        XPLMRemoveMenuItem(XPLMFindPluginsMenu(), menuItemIdx_);
        menuItemIdx_ = -1;
    }
}

void StatusWindow::setVisible(bool visible) {
    if (!windowId_) return;
    XPLMSetWindowIsVisible(windowId_, visible ? 1 : 0);
    lastKnownVisible_ = visible;
    saveStatusWindowVisible(visible);
}

bool StatusWindow::isVisible() const {
    if (!windowId_) return false;
    return XPLMGetWindowIsVisible(windowId_) != 0;
}

void StatusWindow::update(const StatusData& data) {
    data_ = data;
    bool nowVisible = isVisible();
    if (nowVisible != lastKnownVisible_) {
        lastKnownVisible_ = nowVisible;
        saveStatusWindowVisible(nowVisible);
    }
}

void StatusWindow::setReloadCallback(std::function<void()> cb) {
    reloadCallback_ = std::move(cb);
}

void StatusWindow::drawCallback(XPLMWindowID inWindowID, void* inRefcon) {
    (void)inWindowID;
    StatusWindow* self = static_cast<StatusWindow*>(inRefcon);
    if (self) self->draw();
}

void StatusWindow::keyCallback(XPLMWindowID, char inKey, XPLMKeyFlags, char, void* inRefcon, int) {
    StatusWindow* self = static_cast<StatusWindow*>(inRefcon);
    if (!self) return;
    if (inKey == 27) self->setVisible(false); // ESC hides
}

void StatusWindow::menuCallback(void* inMenuRef, void* inItemRef) {
    (void)inMenuRef;
    StatusWindow* self = static_cast<StatusWindow*>(inItemRef);
    if (self) self->setVisible(!self->isVisible());
}

void StatusWindow::mouseCallback(XPLMWindowID, int x, int y, XPLMMouseStatus s, void*) {
    if (s != xplm_MouseDown) return;
    if (x >= btnLeft_ && x <= btnRight_ && y <= btnTop_ && y >= btnBottom_) {
        if (reloadCallback_) reloadCallback_();
    }
}

void StatusWindow::draw() {
    if (!windowId_) return;
    int left, top, right, bottom;
    XPLMGetWindowGeometry(windowId_, &left, &top, &right, &bottom);
    XPLMDrawTranslucentDarkBox(left, top, right, bottom);

    int x = left + 10;
    int y = top - 20;
    char buf[160];

    drawString(x, y, "Motion Provider v0.4 (Phase 2a)", 0.8f, 1.0f, 0.8f);
    y -= 20;

    static const char* kAxis[6] = { "surge","sway","heave","roll","pitch","yaw" };
    if (data_.manualMode) {
        std::snprintf(buf, sizeof(buf),
            "MANUAL  axis=%s  [%.1f %.1f %.1f | %.1f %.1f %.1f]",
            kAxis[data_.manualAxis],
            data_.manualPose.surge, data_.manualPose.sway, data_.manualPose.heave,
            data_.manualPose.roll, data_.manualPose.pitch, data_.manualPose.yaw);
        drawString(x, y, buf, 1.0f, 0.9f, 0.5f);
    } else {
        drawString(x, y, "AUTO (attitude placeholder)   [M] manual", 0.7f, 0.8f, 0.9f);
    }
    y -= 16;
    drawString(x, y, "[M] mode  [Tab] axis  [Up/Dn] nudge  [R] reset",
               0.6f, 0.6f, 0.65f);
    y -= 18;

    drawString(x, y, data_.solve.allReachable ? "IK: all legs reachable"
                                               : "IK: POSE UNREACHABLE",
               data_.solve.allReachable ? 0.6f : 1.0f,
               data_.solve.allReachable ? 1.0f : 0.4f, 0.4f);
    y -= 16;

    for (int i = 0; i < 6; ++i) {
        std::snprintf(buf, sizeof(buf), "P%d  %+7.2f deg  ->  %5u%s",
                      i + 1, data_.solve.legs[i].angleDeg, data_.solve.setpoints[i],
                      data_.solve.legs[i].reachable ? "" : "  (unreachable)");
        drawString(x, y, buf, 0.85f, 0.85f, 0.9f);
        y -= 16;
    }

    // Reload button
    y -= 4;
    btnLeft_ = x; btnTop_ = y + 12; btnRight_ = x + 150; btnBottom_ = y - 4;
    drawString(x, y, data_.lastReloadOk ? "[ Reload config ]" : "[ Reload FAILED ]",
               0.7f, data_.lastReloadOk ? 0.9f : 0.4f, 0.9f);
}

void StatusWindow::drawString(int x, int y, const std::string& text, float r, float g, float b) {
    float color[3] = { r, g, b };
    XPLMDrawString(color, x, y, const_cast<char*>(text.c_str()), nullptr, xplmFont_Basic);
}
