#pragma once
#include <memory>
#include "MotionCues.h"
#include "StewartKinematics.h"
#include "StatusData.h"

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

private:
    std::unique_ptr<StatusWindow> statusWindow_;
    std::unique_ptr<DataRefManager> dataRefs_;
    std::unique_ptr<StewartKinematics> kin_;

    // Most recent sampled snapshot (updated every 60 Hz tick).
    MotionCues latestCues_;
    SolveResult latestSolve_;

    // Status window refresh accumulator (~1 Hz), independent of the 60 Hz tick.
    float statusAccumSec_ = 0.0f;

    // Manual DOF control state
    bool manualMode_ = false;
    int manualAxis_ = 0;
    Pose manualPose_;
    bool lastReloadOk_ = true;
};
