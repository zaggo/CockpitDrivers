#pragma once
#include "XPLMDisplay.h"
#include "AxisCurve.h"
#include <functional>
#include <string>
#include <vector>

// Floating window that edits one AxisCurve in place: a plot of the curve with
// a live marker of the current axis input on the left, five draggable sliders
// (one per AxisCurve parameter) with reset buttons on the right.
//
// The curve pointer is written directly on every drag, so the change is live
// on the next axis frame. changedCallback_ fires on slider release and reset
// so the owner can persist it.
class CurveWindow {
public:
    CurveWindow(std::string axisLabel, AxisCurve* curve);
    ~CurveWindow();

    CurveWindow(const CurveWindow&) = delete;
    CurveWindow& operator=(const CurveWindow&) = delete;

    // Create the XPLM window. Visibility is restored from the config file.
    void initialize();
    void destroy();

    void setVisible(bool visible);
    bool isVisible() const;

    // Call ~1 Hz: detects the native close button and persists visibility.
    void update();

    // Current raw axis input (-1..1) for the live marker. Call per axis frame.
    void setLiveInput(float raw);

    // Fired on slider release and on reset. Owner persists the curve here.
    void setChangedCallback(std::function<void()> cb);

private:
    enum Param { SENS_POS = 0, SENS_NEG, NEUTRAL, DEAD_ZONE, EXTREMITY, PARAM_COUNT };

    struct Row { const char* label; float min; float max; };
    static const Row kRows[PARAM_COUNT];

    // Clickable region, rebuilt every draw() frame. reset=false is a slider
    // track, reset=true is that row's [R] button.
    struct Hit { int left, top, right, bottom; int param; bool reset; };

    float getParam(int p) const;
    void  setParam(int p, float v);
    static float valueFromX(int x, int trackLeft, int trackWidth, float min, float max);

    static void drawCallback(XPLMWindowID inWindowID, void* inRefcon);
    static void keyCallback(XPLMWindowID inWindowID, char inKey, XPLMKeyFlags inFlags,
                            char inVirtualKey, void* inRefcon, int losingFocus);
    static int  mouseCallback(XPLMWindowID inWindowID, int x, int y,
                              XPLMMouseStatus inMouse, void* inRefcon);

    void draw();
    void drawPlot(int left, int top, int size);
    void drawSliders(int x, int y);
    void drawString(int x, int y, const std::string& text, float r, float g, float b);

    std::string axisLabel_;
    AxisCurve* curve_;
    std::function<void()> changedCallback_;
    XPLMWindowID windowId_ = nullptr;
    std::vector<Hit> hits_;

    // Slider currently being dragged, -1 when none; track geometry captured
    // at MouseDown so MouseDrag doesn't depend on hits_ being rebuilt.
    int grabbed_ = -1;
    int grabLeft_ = 0;
    int grabWidth_ = 1;

    float liveInput_ = 0.f;
    bool lastKnownVisible_ = false;
};
