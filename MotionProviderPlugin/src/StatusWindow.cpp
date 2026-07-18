#include "StatusWindow.h"
#include "ConfigUtils.h"
#include "XPLMUtilities.h"

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
    params.bottom = 380;
    params.visible = shouldBeVisible ? 1 : 0;
    params.drawWindowFunc = drawCallback;
    params.handleKeyFunc = keyCallback;
    params.handleMouseClickFunc = [](XPLMWindowID, int, int, XPLMMouseStatus, void*) -> int { return 1; };
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

void StatusWindow::update() {
    // Keep persisted visibility in sync with the native close button
    // (round-rectangle decoration has no callback of its own).
    bool nowVisible = isVisible();
    if (nowVisible != lastKnownVisible_) {
        lastKnownVisible_ = nowVisible;
        saveStatusWindowVisible(nowVisible);
    }
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

void StatusWindow::draw() {
    if (!windowId_) return;
    int left, top, right, bottom;
    XPLMGetWindowGeometry(windowId_, &left, &top, &right, &bottom);
    XPLMDrawTranslucentDarkBox(left, top, right, bottom);
    drawString(left + 10, top - 20, "Motion Provider v0.1 (Phase 0)", 0.8f, 1.0f, 0.8f);
}

void StatusWindow::drawString(int x, int y, const std::string& text, float r, float g, float b) {
    float color[3] = { r, g, b };
    XPLMDrawString(color, x, y, const_cast<char*>(text.c_str()), nullptr, xplmFont_Basic);
}
