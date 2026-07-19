#pragma once
#include <memory>
#include <string>
#include "MotionCues.h"
#include "StewartKinematics.h"
#include "StatusData.h"
#include "WashoutFilter.h"
#include "EffectsLayer.h"
#include "SerialLink.h"
#include "SafetyLimiter.h"
#include "SafetyConfig.h"
#include "ArmRamp.h"
#include "BffEncoder.h"

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

    void selectPort(const std::string& port);

private:
    void pushStatus();             // build a StatusData and refresh the window now
    // Blend the live pose toward the park pose by the arm ramp, then clamp to a
    // reachable pose (every intermediate is a valid rigid config).
    Pose blendedCommand(const Pose& rawLive) const;
    void recomputeParkPose();      // park pose = lowest reachable along parkHeaveMm

    std::unique_ptr<StatusWindow> statusWindow_;
    std::unique_ptr<DataRefManager> dataRefs_;
    std::unique_ptr<StewartKinematics> kin_;
    std::unique_ptr<WashoutFilter> washout_;
    std::unique_ptr<EffectsLayer> effects_;
    std::unique_ptr<SerialLink> serial_;
    std::unique_ptr<SafetyLimiter> safety_;

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

    // Serial + safety (arm/disarm soft-start in pose space)
    ArmRamp armRamp_;
    SafetyConfig safetyCfg_;       // cached for ramp durations
    Pose parkPose_;                // low, level park pose (reachable)
    uint16_t sentSetpoints_[6] = {32640,32640,32640,32640,32640,32640};
};
