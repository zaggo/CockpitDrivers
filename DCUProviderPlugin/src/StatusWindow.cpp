#include "StatusWindow.h"
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <sstream>

StatusWindow::StatusWindow()
    : windowId_(nullptr), menuItemIdx_(-1), pluginMenuId_(nullptr) {
}

StatusWindow::~StatusWindow() {
    destroy();
}

void StatusWindow::initialize() {
    // Create window using new X-Plane API
    XPLMCreateWindow_t params = {};
    params.structSize = sizeof(XPLMCreateWindow_t);
    params.left = 100;
    params.top = 500;
    params.right = 500;
    params.bottom = 200;
    params.visible = 1;
    params.drawWindowFunc = drawCallback;
    params.handleKeyFunc = keyCallback;
    // Mouse callback must return int, so use a static adapter
    params.handleMouseClickFunc = [](XPLMWindowID inWindowID, int x, int y, XPLMMouseStatus inMouse, void* inRefcon) -> int {
        StatusWindow* self = static_cast<StatusWindow*>(inRefcon);
        if (self) self->mouseCallback(inWindowID, x, y, inMouse, inRefcon);
        return 1; // consume click
    };
    params.handleCursorFunc = nullptr;
    params.handleMouseWheelFunc = nullptr;
    params.handleRightClickFunc = nullptr;
    params.refcon = this;
    params.layer = xplm_WindowLayerFloatingWindows;
    params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
    
    windowId_ = XPLMCreateWindowEx(&params);
    
    if (!windowId_) {
        XPLMDebugString("DCUProvider: Failed to create status window\n");
        return;
    }
    
    // Create menu item
    pluginMenuId_ = XPLMFindPluginsMenu();
    
    if (pluginMenuId_) {
        menuItemIdx_ = XPLMAppendMenuItem(pluginMenuId_, "DCU Provider", this, 1);
    }
    
    XPLMDebugString("DCUProvider: Status window initialized\n");
}

void StatusWindow::destroy() {
    if (windowId_) {
        XPLMDestroyWindow(windowId_);
        windowId_ = nullptr;
    }
    
    if (menuItemIdx_ >= 0 && pluginMenuId_) {
        XPLMRemoveMenuItem(pluginMenuId_, menuItemIdx_);
        menuItemIdx_ = -1;
    }
}

void StatusWindow::setVisible(bool visible) {
    if (windowId_) {
        XPLMSetWindowIsVisible(windowId_, visible ? 1 : 0);
    }
}

bool StatusWindow::isVisible() const {
    if (!windowId_) return false;
    return XPLMGetWindowIsVisible(windowId_) != 0;
}

void StatusWindow::update(const StatusData& data) {
    statusData_ = data;
    
    // Request redraw
    // In practice, X-Plane redraws every frame anyway
    // If you need geometry, use int variables:
    // int left, top, right, bottom;
    // XPLMGetWindowGeometry(windowId_, &left, &top, &right, &bottom);
}

void StatusWindow::drawCallback(XPLMWindowID inWindowID, void* inRefcon) {
    (void)inWindowID;
    StatusWindow* self = static_cast<StatusWindow*>(inRefcon);
    if (self) {
        self->draw();
    }
}

void StatusWindow::keyCallback(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags,
                                char inVirtualKey, void* inRefcon, int losingFocus) {
    (void)inWindowID;
    (void)inFlags;
    (void)inVirtualKey;
    (void)losingFocus;
    StatusWindow* self = static_cast<StatusWindow*>(inRefcon);
    if (!self) return;
    // ESC to hide window
    if (inKey == 27) {
        self->setVisible(false);
    }
}

void StatusWindow::mouseCallback(XPLMWindowID inWindowID, int x, int y,
                                  XPLMMouseStatus inMouse, void* inRefcon) {
    (void)inWindowID;
    (void)x;
    (void)y;
    (void)inMouse;
    (void)inRefcon;
    // No special mouse handling needed
}

void StatusWindow::menuCallback(void* inMenuRef, void* inItemRef) {
    (void)inMenuRef;
    StatusWindow* self = static_cast<StatusWindow*>(inItemRef);
    if (self) {
        self->setVisible(!self->isVisible());
    }
}

void StatusWindow::draw() {
    if (!windowId_) return;
    
    int left, top, right, bottom;
    XPLMGetWindowGeometry(windowId_, &left, &top, &right, &bottom);
    // Draw background using XPLMDrawTranslucentDarkBox
    XPLMDrawTranslucentDarkBox(left, top, right, bottom);
    // Optionally, draw a simple border using XPLMDrawString (simulate border with text or skip)
    // Draw text
    drawText(left + 10, top - 20);
}

void StatusWindow::drawText(int x, int y) {
    // Title
    drawString(x, y, "DCU Provider Status v0.1", 0.8f, 1.0f, 0.8f);
    y -= 20;
    
    // Connection status
    std::string connStatus = statusData_.isConnected ? "CONNECTED" : "DISCONNECTED";
    float r = statusData_.isConnected ? 0.2f : 1.0f;
    float g = statusData_.isConnected ? 1.0f : 0.2f;
    drawString(x, y, "Status: " + connStatus, r, g, 0.2f);
    y -= 16;
    
    // Device path
    drawString(x, y, "Device: " + statusData_.devicePath, 0.7f, 0.7f, 0.7f);
    y -= 16;
    
    // Baud rate
    std::stringstream ss;
    ss << "Baud: " << statusData_.baudRate;
    drawString(x, y, ss.str(), 0.7f, 0.7f, 0.7f);
    y -= 16;
    
    // TX/RX stats
    ss.str("");
    ss << "TX: " << statusData_.txBytesSent << " bytes | "
       << "RX: " << statusData_.rxBytesReceived << " bytes";
    drawString(x, y, ss.str(), 0.9f, 0.9f, 0.9f);
    y -= 16;
    
    // Last TX/RX times
    ss.str("");
    ss << "Last TX: " << std::fixed << std::setprecision(1) 
       << statusData_.lastTxTime << "s | "
       << "Last RX: " << statusData_.lastRxTime << "s";
    drawString(x, y, ss.str(), 0.8f, 0.8f, 0.8f);
    y -= 16;
    
    // Write status
    std::string writeStatus = statusData_.lastWriteOk ? "OK" : "FAIL";
    float wr = statusData_.lastWriteOk ? 0.2f : 1.0f;
    float wg = statusData_.lastWriteOk ? 1.0f : 0.2f;
    drawString(x, y, "Last Write: " + writeStatus, wr, wg, 0.2f);
    y -= 16;
    
    // Open status
    std::string openStatus = statusData_.lastOpenOk ? "OK" : "FAIL";
    float or_ = statusData_.lastOpenOk ? 0.2f : 1.0f;
    float og = statusData_.lastOpenOk ? 1.0f : 0.2f;
    drawString(x, y, "Last Open: " + openStatus, or_, og, 0.2f);
}

void StatusWindow::drawString(int x, int y, const std::string& text, 
                               float r, float g, float b) {
    float color[3] = { r, g, b };
    XPLMDrawString(color, x, y, const_cast<char*>(text.c_str()), nullptr, xplmFont_Basic);
}