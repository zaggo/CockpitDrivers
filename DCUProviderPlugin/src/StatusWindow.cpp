
#include "StatusWindow.h"
#include <algorithm>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <sstream>

StatusWindow::StatusWindow()
    : windowId_(nullptr), menuItemIdx_(-1), pluginMenuId_(nullptr) {
}

void StatusWindow::setAvailablePorts(const std::vector<std::string>& ports) {
    availablePorts_ = ports;
    if (!ports.empty()) {
        if (selectedPortIdx_ >= (int)ports.size())
            selectedPortIdx_ = 0;
    } else {
        selectedPortIdx_ = 0;
    }
}

void StatusWindow::setPortChangedCallback(std::function<void(const std::string&)> cb) {
    portChangedCallback_ = std::move(cb);
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
    
    // Create menu
    pluginMenuId_ = XPLMFindPluginsMenu();
    
    if (pluginMenuId_) {
        // Create a submenu for our plugin
        int subMenuIdx = XPLMAppendMenuItem(pluginMenuId_, "DCU Provider", nullptr, 1);
        XPLMMenuID ourMenuId = XPLMCreateMenu("DCU Provider", pluginMenuId_, subMenuIdx, 
                                               menuCallback, this);
        
        // Add "Show Status Window" item to our submenu
        XPLMAppendMenuItem(ourMenuId, "Show Status Window", this, 1);
        
        // Store reference for cleanup
        menuItemIdx_ = subMenuIdx;
        pluginMenuId_ = ourMenuId;
    }
    
    XPLMDebugString("DCUProvider: Status window initialized\n");
}

void StatusWindow::destroy() {
    if (windowId_) {
        XPLMDestroyWindow(windowId_);
        windowId_ = nullptr;
    }
    
    if (pluginMenuId_) {
        XPLMDestroyMenu(pluginMenuId_);
        pluginMenuId_ = nullptr;
    }
    
    menuItemIdx_ = -1;
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
        return;
    }

    // Up/Down arrow to change port selection
    if (!self->availablePorts_.empty()) {
        if (inKey == XPLM_VK_UP) {
            if (self->selectedPortIdx_ > 0) {
                self->selectedPortIdx_--;
                if (self->portChangedCallback_)
                    self->portChangedCallback_(self->availablePorts_[self->selectedPortIdx_]);
            }
        } else if (inKey == XPLM_VK_DOWN) {
            if (self->selectedPortIdx_ + 1 < (int)self->availablePorts_.size()) {
                self->selectedPortIdx_++;
                if (self->portChangedCallback_)
                    self->portChangedCallback_(self->availablePorts_[self->selectedPortIdx_]);
            }
        }
    }
}

void StatusWindow::mouseCallback(XPLMWindowID inWindowID, int x, int y,
                                  XPLMMouseStatus inMouse, void* inRefcon) {
    (void)inWindowID;
    (void)x;

    StatusWindow* self = static_cast<StatusWindow*>(inRefcon);
    if (!self) return;
    if (inMouse != xplm_MouseDown) return;

    // Get window geometry
    int left, top, right, bottom;
    XPLMGetWindowGeometry(self->windowId_, &left, &top, &right, &bottom);

    // Port-Liste beginnt bei drawText: y = top - 20 - 20 (Titel und Status) - 16 ("Device:")
    int yStart = top - 20 - 20 - 16;
    int lineHeight = 16;

    // Prüfe, ob Klick im Bereich der Port-Liste
    for (size_t i = 0; i < self->availablePorts_.size(); ++i) {
        int yLine = yStart - (int)i * lineHeight;
        // Einfache Hitbox: voller Fensterbereich in X, Zeile in Y
        if (y <= yLine && y > yLine - lineHeight) {
            self->selectedPortIdx_ = (int)i;
            if (self->portChangedCallback_)
                self->portChangedCallback_(self->availablePorts_[self->selectedPortIdx_]);
            break;
        }
    }
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
    
    // Serial port selection UI
    drawString(x, y, "Device:", 0.7f, 0.7f, 0.7f);
    y -= 16;
    if (!availablePorts_.empty()) {
        for (size_t i = 0; i < availablePorts_.size(); ++i) {
            std::string prefix = (i == (size_t)selectedPortIdx_) ? "> " : "  ";
            drawString(x + 10, y, prefix + availablePorts_[i],
                (i == (size_t)selectedPortIdx_) ? 1.0f : 0.7f,
                (i == (size_t)selectedPortIdx_) ? 1.0f : 0.7f,
                (i == (size_t)selectedPortIdx_) ? 0.2f : 0.7f);
            y -= 16;
        }
    } else {
        drawString(x + 10, y, "(No serial ports found)", 0.7f, 0.7f, 0.7f);
        y -= 16;
    }
    
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
    
    // Last TX/RX times (Sekunden seit letztem Ereignis)
    float now = static_cast<float>(std::time(nullptr));
    float lastTxAgo = (statusData_.lastTxTime > 0.0f) ? (now - statusData_.lastTxTime) : -1.0f;
    float lastRxAgo = (statusData_.lastRxTime > 0.0f) ? (now - statusData_.lastRxTime) : -1.0f;
    ss.str("");
    ss << "Last TX: ";
    if (lastTxAgo >= 0.0f)
        ss << std::fixed << std::setprecision(1) << lastTxAgo << "s ago | ";
    else
        ss << "n/a | ";
    ss << "Last RX: ";
    if (lastRxAgo >= 0.0f)
        ss << std::fixed << std::setprecision(1) << lastRxAgo << "s ago";
    else
        ss << "n/a";
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