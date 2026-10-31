#pragma once
#include "XPLMDisplay.h"
#include "AxisCurve.h"
#include <functional>
#include <string>
#include <vector>

// Options for one CurveWindow instance.
struct CurveWindowOptions {
    // true: axis domain -1..1 with Sensitivity +/- and Neutral rows (rudder).
    // false: unipolar 0..1 domain with a single Sensitivity row (toe brakes).
    bool bipolar = true;
    // When set, adds a shared "Diff dead zone" slider row editing this value
    // (the toe-brake differential dead zone; both brake windows get the same
    // pointer, so it stays in sync regardless of which window is dragged).
    float* diffDeadZone = nullptr;
    // Config key suffix: visibility persists as curve_window_visible.<configAxis>.
    std::string configAxis;
    // Initial window position (screen coordinates of the top-left corner).
    int left = 150;
    int top = 600;
};

// Floating window that edits one AxisCurve in place: a plot of the curve with
// a live marker of the current axis input on the left, draggable sliders (one
// per edited parameter) with reset buttons on the right.
//
// The row targets (curve fields, optional shared diff dead zone) are written
// directly on every drag, so the change is live on the next axis frame.
// changedCallback_ fires on slider release and reset so the owner can persist.
class CurveWindow {
public:
    CurveWindow(std::string axisLabel, AxisCurve* curve, CurveWindowOptions options = {});
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

    // Current raw axis input for the live marker, in the window's domain
    // (-1..1 bipolar, 0..1 unipolar). Call per axis frame.
    void setLiveInput(float raw);

    // Fired on slider release and on reset. Owner persists the curve here.
    void setChangedCallback(std::function<void()> cb);

private:
    // One slider row: label, slider range and the value it edits. Built from
    // CurveWindowOptions at construction.
    struct Row { std::string label; float min, max; float* target; };

    // Clickable region, rebuilt every draw() frame. reset=false is a slider
    // track, reset=true is that row's [R] button. row indexes rows_.
    struct Hit { int left, top, right, bottom; int row; bool reset; };

    void setValue(int row, float v);
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
    CurveWindowOptions options_;
    std::vector<Row> rows_;
    float domainLo_ = -1.f;   // plot/marker domain lower end (-1 or 0)
    std::function<void()> changedCallback_;
    XPLMWindowID windowId_ = nullptr;
    std::vector<Hit> hits_;

    // Slider row currently being dragged, -1 when none; track geometry captured
    // at MouseDown so MouseDrag doesn't depend on hits_ being rebuilt.
    int grabbed_ = -1;
    int grabLeft_ = 0;
    int grabWidth_ = 1;

    float liveInput_ = 0.f;
    bool lastKnownVisible_ = false;
};
