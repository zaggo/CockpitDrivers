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
    params.right = 540;
    params.bottom = 40;
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

    rescanPorts();

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

void StatusWindow::setCommandCallback(std::function<void(int)> cb) {
    commandCallback_ = std::move(cb);
}

void StatusWindow::setPortSelectedCallback(std::function<void(const std::string&)> cb) {
    portSelectedCallback_ = std::move(cb);
}

void StatusWindow::rescanPorts() {
    ports_ = enumerateSerialPorts();
}

void StatusWindow::drawCallback(XPLMWindowID inWindowID, void* inRefcon) {
    (void)inWindowID;
    StatusWindow* self = static_cast<StatusWindow*>(inRefcon);
    if (self) self->draw();
}

void StatusWindow::keyCallback(XPLMWindowID, char inKey, XPLMKeyFlags, char,
                               void* inRefcon, int) {
    // Manual control is via on-screen buttons (X-Plane owns most keystrokes and
    // the window rarely has keyboard focus). Only ESC-to-hide is handled here.
    StatusWindow* self = static_cast<StatusWindow*>(inRefcon);
    if (self && inKey == 27) self->setVisible(false);
}

void StatusWindow::menuCallback(void* inMenuRef, void* inItemRef) {
    (void)inMenuRef;
    StatusWindow* self = static_cast<StatusWindow*>(inItemRef);
    if (self) self->setVisible(!self->isVisible());
}

void StatusWindow::mouseCallback(XPLMWindowID, int x, int y, XPLMMouseStatus s, void*) {
    if (s != xplm_MouseDown) return;
    for (const Button& b : buttons_) {
        if (x >= b.left && x <= b.right && y >= b.bottom && y <= b.top) {
            if (!b.port.empty()) {
                if (portSelectedCallback_) portSelectedCallback_(b.port);
            } else {
                if (b.action == UI_RESCAN_PORTS) rescanPorts();
                if (commandCallback_) commandCallback_(b.action);
            }
            return;
        }
    }
}

int StatusWindow::button(int x, int y, const std::string& label, int action,
                         float r, float g, float b) {
    int cw = 0, ch = 0;
    XPLMGetFontDimensions(xplmFont_Basic, &cw, &ch, nullptr);
    if (cw <= 0) cw = 7;
    if (ch <= 0) ch = 12;
    const int w = static_cast<int>(label.size()) * cw;
    buttons_.push_back({ x, y + ch, x + w, y - 3, action, "" });
    drawString(x, y, label, r, g, b);
    return w;
}

int StatusWindow::portButton(int x, int y, const std::string& label, const std::string& port,
                             float r, float g, float b) {
    int cw = 0, ch = 0;
    XPLMGetFontDimensions(xplmFont_Basic, &cw, &ch, nullptr);
    if (cw <= 0) cw = 7;
    if (ch <= 0) ch = 12;
    const int w = static_cast<int>(label.size()) * cw;
    // action is unused for port buttons: mouseCallback routes on non-empty port
    // before ever reading action. The placeholder must NOT be relied upon.
    buttons_.push_back({ x, y + ch, x + w, y - 3, UI_RESCAN_PORTS, port });
    drawString(x, y, label, r, g, b);
    return w;
}

void StatusWindow::draw() {
    if (!windowId_) return;
    buttons_.clear();

    int left, top, right, bottom;
    XPLMGetWindowGeometry(windowId_, &left, &top, &right, &bottom);
    XPLMDrawTranslucentDarkBox(left, top, right, bottom);

    int cw = 0, ch = 0;
    XPLMGetFontDimensions(xplmFont_Basic, &cw, &ch, nullptr);
    if (cw <= 0) cw = 7;
    const int gap = cw * 2;

    int x = left + 10;
    int y = top - 20;
    char buf[160];
    static const char* kAxis[6] = { "surge","sway","heave","roll","pitch","yaw" };

    drawString(x, y, "Motion Provider v0.7 (Phase 5)", 0.8f, 1.0f, 0.8f);
    y -= 20;

    // ---- Mode toggle (SIM/MANUAL, disarmed only) + DISARM e-stop ----
    // Arming is driven by the hardware switch; the UI only chooses the mode to
    // arm into and offers a software e-stop. Active/static = yellow; clickable =
    // cyan; disabled = grey.
    const int  st       = data_.armState;   // 0 Disarmed,1 Arming,2 Armed,3 Disarming
    const bool disarmed = (st == 0);
    const bool armedish = (st == 1 || st == 2);
    const bool manActive = armedish && data_.manualMode;   // used by manual DOF UI below
    {
        int bx = x;
        // Mode toggle: clickable only while disarmed.
        const char* modeLabel = data_.manualMode ? "[ MANUAL ]" : "[ SIM ]";
        if (disarmed) {
            bx += button(bx, y, modeLabel, UI_TOGGLE_MODE, 0.7f, 0.9f, 1.0f) + gap;
        } else {
            drawString(bx, y, modeLabel, 1.0f, 0.85f, 0.2f);   // yellow = locked-in mode
            bx += static_cast<int>(std::string(modeLabel).size()) * cw + gap;
        }
        // DISARM e-stop: clickable whenever not fully disarmed.
        if (!disarmed) {
            button(bx, y, "[ DISARM ]", UI_DISARM, 0.7f, 0.9f, 1.0f);
        } else {
            drawString(bx, y, "[ DISARM ]", 0.45f, 0.45f, 0.5f);  // grey = disabled
        }
    }
    y -= 18;

    // State + serial status line.
    {
        char stateStr[40];
        if (disarmed)              std::snprintf(stateStr, sizeof(stateStr), "disarmed (park)");
        else if (st == 3)          std::snprintf(stateStr, sizeof(stateStr), "DISARMING %.0f%%", data_.armBlend * 100.0f);
        else if (data_.manualMode) { if (st == 1) std::snprintf(stateStr, sizeof(stateStr), "MANUAL arming %.0f%%", data_.armBlend * 100.0f);
                                     else          std::snprintf(stateStr, sizeof(stateStr), "MANUAL"); }
        else                       { if (st == 1) std::snprintf(stateStr, sizeof(stateStr), "ARMING %.0f%%", data_.armBlend * 100.0f);
                                     else          std::snprintf(stateStr, sizeof(stateStr), "ARMED"); }
        char sb[128];
        std::snprintf(sb, sizeof(sb), "%s   %s   frames %llu", stateStr,
                      data_.serialConnected ? "CONNECTED" : "no link",
                      (unsigned long long)data_.framesSent);
        drawString(x, y, sb, data_.serialConnected ? 0.6f : 0.8f,
                   data_.serialConnected ? 1.0f : 0.7f, 0.6f);
    }
    y -= 16;

    if (data_.faultCode != 0) {
        drawString(x, y, data_.faultReason.c_str(), 1.0f, 0.3f, 0.3f);
        y -= 16;
    }

    // Manual DOF controls (only when armed in MANUAL). Axis 6 = Identify actor.
    if (manActive) {
        if (data_.manualAxis == 6)
            std::snprintf(buf, sizeof(buf), "[ Identify: %s ]", data_.identifyActor.c_str());
        else
            std::snprintf(buf, sizeof(buf), "[ Axis: %s ]", kAxis[data_.manualAxis]);
        int bx = x;
        bx += button(bx, y, buf, UI_NEXT_AXIS, 1.0f, 0.9f, 0.5f) + gap;
        bx += button(bx, y, "[ - ]", UI_NUDGE_MINUS, 0.9f, 0.9f, 0.6f) + gap;
        bx += button(bx, y, "[ + ]", UI_NUDGE_PLUS, 0.9f, 0.9f, 0.6f) + gap;
        button(bx, y, "[ Reset ]", UI_RESET, 0.9f, 0.7f, 0.6f);
        y -= 16;
    }

    // Commanded pose fed to the IK (what actually moves).
    const Pose& cp = data_.commandedPose;
    std::snprintf(buf, sizeof(buf),
        "cmd  x%+.1f y%+.1f z%+.1f mm | r%+.1f p%+.1f w%+.1f deg",
        cp.surge, cp.sway, cp.heave, cp.roll, cp.pitch, cp.yaw);
    drawString(x, y, buf, 1.0f, 0.9f, 0.6f);
    y -= 16;

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

    // Reload button + transient confirmation.
    y -= 4;
    int rx = x + button(x, y, "[ Reload config ]", UI_RELOAD, 0.7f, 0.9f, 0.9f) + gap;
    if (data_.reloadFlash) {
        if (data_.lastReloadOk) drawString(rx, y, "Config loaded", 0.4f, 1.0f, 0.5f);
        else                    drawString(rx, y, "No config file - using defaults", 1.0f, 0.8f, 0.3f);
    }
    y -= 18;

    // Serial port chooser.
    std::snprintf(buf, sizeof(buf), "port: %s",
                  data_.serialPort.empty() ? "(none - pick below)" : data_.serialPort.c_str());
    drawString(x, y, buf, 0.8f, 0.8f, 0.9f);
    y -= 16;
    button(x, y, "[ Rescan ]", UI_RESCAN_PORTS, 0.7f, 0.8f, 0.9f);
    y -= 16;
    for (const auto& p : ports_) {
        bool sel = (p == data_.serialPort);
        std::snprintf(buf, sizeof(buf), "%s %s", sel ? ">" : " ", p.c_str());
        portButton(x, y, buf, p, sel ? 1.0f : 0.7f, sel ? 1.0f : 0.7f, sel ? 0.4f : 0.7f);
        y -= 16;
    }
}

void StatusWindow::drawString(int x, int y, const std::string& text, float r, float g, float b) {
    float color[3] = { r, g, b };
    XPLMDrawString(color, x, y, const_cast<char*>(text.c_str()), nullptr, xplmFont_Basic);
}
