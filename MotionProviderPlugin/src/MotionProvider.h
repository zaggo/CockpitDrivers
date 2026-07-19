#pragma once
#include <memory>
#include "MotionCues.h"
#include "StewartKinematics.h"
#include "StatusData.h"
#include "WashoutFilter.h"
#include "EffectsLayer.h"

class StatusWindow;
class DataRefManager;

class MotionProvider {
public:
    MotionProvider();
    ~MotionProvider();

    MotionProvider(const MotionProvider&) = delete;
    MotionProvider& operator=(const MotionProvider&) = delete;

    // Called from Plugin.cpp (X-Plane ABI thread).
    bool initialize();
    void shutdown();
    void onFlightLoopTick(float elapsedSec);
    void onAircraftLoaded();

    void reloadConfig();
    void onUiAction(int action);   // UiAction code from a status-window button

private:
    void pushStatus();             // build a StatusData and refresh the window now

    std::unique_ptr<StatusWindow> statusWindow_;
    std::unique_ptr<DataRefManager> dataRefs_;
    std::unique_ptr<StewartKinematics> kin_;
    std::unique_ptr<WashoutFilter> washout_;
    std::unique_ptr<EffectsLayer> effects_;

    MotionCues latestCues_;
    SolveResult latestSolve_;
    Pose latestPose_;              // pose fed to the IK this tick (for display)

    float statusAccumSec_ = 0.0f;

    // Manual DOF control state
    bool manualMode_ = false;
    int manualAxis_ = 0;
    Pose manualPose_;
    bool lastReloadOk_ = true;
    float reloadFlashRemaining_ = 0.0f;  // seconds left to show "Config loaded"
};
