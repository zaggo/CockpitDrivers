#include "CurveWindow.h"
#include "ConfigUtils.h"
#include "XPLMGraphics.h"
#include "XPLMUtilities.h"
#include <algorithm>
#include <cstdio>

#if APL
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl.h>
#elif IBM
#define NOMINMAX   // windows.h defines min/max macros that break std::min/std::max
#include <windows.h>
#include <GL/gl.h>
#else
#include <GL/gl.h>
#endif

namespace {
constexpr int kWinW = 620, kWinH = 320;
constexpr int kPlotSize = 240;
constexpr int kTrackW = 150, kTrackH = 10;
constexpr int kRowStep = 30;
constexpr int kCurveSamples = 65;
}

const CurveWindow::Row CurveWindow::kRows[CurveWindow::PARAM_COUNT] = {
    { "Sensitivity +", -1.f, 1.f },
    { "Sensitivity -", -1.f, 1.f },
    { "Neutral",       -1.f, 1.f },
    { "Dead zone",      0.f, 0.5f },
    { "Extremity DZ",   0.f, 0.5f },
};

CurveWindow::CurveWindow(std::string axisLabel, AxisCurve* curve)
    : axisLabel_(std::move(axisLabel)), curve_(curve) {}

CurveWindow::~CurveWindow() { destroy(); }

void CurveWindow::setChangedCallback(std::function<void()> cb) { changedCallback_ = std::move(cb); }
void CurveWindow::setLiveInput(float raw) { liveInput_ = raw; }

float CurveWindow::getParam(int p) const {
    switch (p) {
        case SENS_POS:  return curve_->sensPos;
        case SENS_NEG:  return curve_->sensNeg;
        case NEUTRAL:   return curve_->neutral;
        case DEAD_ZONE: return curve_->deadZone;
        default:        return curve_->extremityDeadZone;
    }
}

void CurveWindow::setParam(int p, float v) {
    v = std::max(kRows[p].min, std::min(kRows[p].max, v));
    switch (p) {
        case SENS_POS:  curve_->sensPos = v; break;
        case SENS_NEG:  curve_->sensNeg = v; break;
        case NEUTRAL:   curve_->neutral = v; break;
        case DEAD_ZONE: curve_->deadZone = v; break;
        default:        curve_->extremityDeadZone = v; break;
    }
}

float CurveWindow::valueFromX(int x, int trackLeft, int trackWidth, float min, float max) {
    float t = static_cast<float>(x - trackLeft) / static_cast<float>(std::max(1, trackWidth));
    t = std::max(0.f, std::min(1.f, t));
    return min + t * (max - min);
}

void CurveWindow::initialize() {
    const bool shouldBeVisible = loadCurveWindowVisible();

    XPLMCreateWindow_t params = {};
    params.structSize = sizeof(XPLMCreateWindow_t);
    params.left = 150;
    params.top = 600;
    params.right = 150 + kWinW;
    params.bottom = 600 - kWinH;
    params.visible = shouldBeVisible ? 1 : 0;
    params.drawWindowFunc = drawCallback;
    params.handleKeyFunc = keyCallback;
    params.handleMouseClickFunc = mouseCallback;
    params.handleCursorFunc = nullptr;
    params.handleMouseWheelFunc = nullptr;
    params.handleRightClickFunc = nullptr;
    params.refcon = this;
    params.layer = xplm_WindowLayerFloatingWindows;
    params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;

    windowId_ = XPLMCreateWindowEx(&params);
    if (!windowId_) {
        XPLMDebugString("DCUProvider: Failed to create curve window\n");
        return;
    }
    XPLMSetWindowTitle(windowId_, (axisLabel_ + " Curve").c_str());
    lastKnownVisible_ = shouldBeVisible;
    XPLMDebugString("DCUProvider: Curve window initialized\n");
}

void CurveWindow::destroy() {
    if (windowId_) {
        XPLMDestroyWindow(windowId_);
        windowId_ = nullptr;
    }
}

void CurveWindow::setVisible(bool visible) {
    if (!windowId_) return;
    XPLMSetWindowIsVisible(windowId_, visible ? 1 : 0);
    lastKnownVisible_ = visible;
    saveCurveWindowVisible(visible);
}

bool CurveWindow::isVisible() const {
    return windowId_ && XPLMGetWindowIsVisible(windowId_) != 0;
}

void CurveWindow::update() {
    if (!windowId_) return;
    // The native close button has no callback; poll for it like StatusWindow does.
    const bool nowVisible = isVisible();
    if (nowVisible != lastKnownVisible_) {
        lastKnownVisible_ = nowVisible;
        saveCurveWindowVisible(nowVisible);
    }
}

// ---------------- X-Plane callbacks ----------------

void CurveWindow::drawCallback(XPLMWindowID, void* inRefcon) {
    if (auto* self = static_cast<CurveWindow*>(inRefcon)) self->draw();
}

void CurveWindow::keyCallback(XPLMWindowID, char inKey, XPLMKeyFlags, char, void* inRefcon, int) {
    auto* self = static_cast<CurveWindow*>(inRefcon);
    if (self && inKey == 27) self->setVisible(false);   // ESC hides
}

int CurveWindow::mouseCallback(XPLMWindowID, int x, int y, XPLMMouseStatus inMouse, void* inRefcon) {
    auto* self = static_cast<CurveWindow*>(inRefcon);
    if (!self) return 1;

    if (inMouse == xplm_MouseDown) {
        for (const Hit& h : self->hits_) {
            if (x < h.left || x > h.right || y < h.bottom || y > h.top) continue;
            if (h.reset) {
                self->setParam(h.param, 0.f);
                if (self->changedCallback_) self->changedCallback_();
            } else {
                self->grabbed_ = h.param;
                self->grabLeft_ = h.left;
                self->grabWidth_ = h.right - h.left;
                self->setParam(h.param, valueFromX(x, h.left, h.right - h.left,
                                                   kRows[h.param].min, kRows[h.param].max));
            }
            return 1;
        }
    } else if (inMouse == xplm_MouseDrag) {
        if (self->grabbed_ >= 0) {
            const int p = self->grabbed_;
            self->setParam(p, valueFromX(x, self->grabLeft_, self->grabWidth_, kRows[p].min, kRows[p].max));
        }
    } else if (inMouse == xplm_MouseUp) {
        if (self->grabbed_ >= 0) {
            self->grabbed_ = -1;
            if (self->changedCallback_) self->changedCallback_();
        }
    }
    return 1; // consume, so drags keep coming to this window
}

// ---------------- drawing ----------------

void CurveWindow::draw() {
    if (!windowId_) return;
    hits_.clear();

    int left, top, right, bottom;
    XPLMGetWindowGeometry(windowId_, &left, &top, &right, &bottom);
    XPLMDrawTranslucentDarkBox(left, top, right, bottom);

    drawString(left + 10, top - 20, axisLabel_ + " axis curve", 0.8f, 1.0f, 0.8f);
    drawPlot(left + 10, top - 35, kPlotSize);
    drawSliders(left + 10 + kPlotSize + 20, top - 50);

    char line[64];
    std::snprintf(line, sizeof(line), "in %+.2f -> out %+.2f", liveInput_, curve_->apply(liveInput_));
    drawString(left + 10, bottom + 12, line, 0.7f, 0.9f, 1.0f);
}

void CurveWindow::drawPlot(int left, int top, int size) {
    const int bottom = top - size;
    const int right = left + size;
    auto px = [&](float v) { return left + (v + 1.f) * 0.5f * size; };
    auto py = [&](float v) { return bottom + (v + 1.f) * 0.5f * size; };

    XPLMSetGraphicsState(0, 0, 0, 0, 0, 0, 0);

    // Frame + axis cross
    glColor3f(0.35f, 0.35f, 0.35f);
    glBegin(GL_LINE_LOOP);
    glVertex2i(left, bottom); glVertex2i(right, bottom); glVertex2i(right, top); glVertex2i(left, top);
    glEnd();
    glBegin(GL_LINES);
    glVertex2f(px(-1.f), py(0.f)); glVertex2f(px(1.f), py(0.f));
    glVertex2f(px(0.f), py(-1.f)); glVertex2f(px(0.f), py(1.f));
    glEnd();

    // Curve
    glColor3f(0.4f, 0.8f, 1.0f);
    glBegin(GL_LINE_STRIP);
    for (int i = 0; i < kCurveSamples; ++i) {
        const float x = -1.f + 2.f * i / (kCurveSamples - 1);
        glVertex2f(px(x), py(curve_->apply(x)));
    }
    glEnd();

    // Live marker
    const float mx = px(std::max(-1.f, std::min(1.f, liveInput_)));
    const float my = py(curve_->apply(liveInput_));
    glColor3f(1.0f, 0.9f, 0.3f);
    glBegin(GL_QUADS);
    glVertex2f(mx - 3, my - 3); glVertex2f(mx + 3, my - 3); glVertex2f(mx + 3, my + 3); glVertex2f(mx - 3, my + 3);
    glEnd();
}

void CurveWindow::drawSliders(int x, int y) {
    const int labelW = 100;
    for (int p = 0; p < PARAM_COUNT; ++p) {
        const Row& row = kRows[p];
        const float v = getParam(p);
        const int trackLeft = x + labelW;
        const int trackBottom = y - 2;
        const int trackTop = trackBottom + kTrackH;

        drawString(x, y, row.label, 0.8f, 0.8f, 0.8f);

        XPLMSetGraphicsState(0, 0, 0, 0, 0, 0, 0);
        glColor3f(0.25f, 0.3f, 0.4f);
        glBegin(GL_QUADS);
        glVertex2i(trackLeft, trackBottom); glVertex2i(trackLeft + kTrackW, trackBottom);
        glVertex2i(trackLeft + kTrackW, trackTop); glVertex2i(trackLeft, trackTop);
        glEnd();

        const float t = (v - row.min) / (row.max - row.min);
        const int knobX = trackLeft + static_cast<int>(t * kTrackW);
        glColor3f(0.4f, 0.7f, 1.0f);
        glBegin(GL_QUADS);
        glVertex2i(knobX - 3, trackBottom - 3); glVertex2i(knobX + 3, trackBottom - 3);
        glVertex2i(knobX + 3, trackTop + 3);    glVertex2i(knobX - 3, trackTop + 3);
        glEnd();

        // Generous hitbox so the thin track is easy to grab.
        hits_.push_back({ trackLeft - 4, trackTop + 6, trackLeft + kTrackW + 4, trackBottom - 6, p, false });

        char val[16];
        std::snprintf(val, sizeof(val), "%+.2f", v);
        drawString(trackLeft + kTrackW + 10, y, val, 1.0f, 1.0f, 1.0f);

        const int rx = trackLeft + kTrackW + 60;
        drawString(rx, y, "[R]", 0.7f, 0.8f, 0.9f);
        hits_.push_back({ rx, y + 12, rx + 24, y - 3, p, true });

        y -= kRowStep;
    }
}

void CurveWindow::drawString(int x, int y, const std::string& text, float r, float g, float b) {
    float color[3] = { r, g, b };
    XPLMDrawString(color, x, y, const_cast<char*>(text.c_str()), nullptr, xplmFont_Basic);
}
