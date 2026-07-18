#include "MotionProvider.h"
#include "StatusWindow.h"
#include "DataRefManager.h"
#include "StewartGeometry.h"
#include "MotionConfig.h"
#include "XPLMUtilities.h"
#include <algorithm>

MotionProvider::MotionProvider() = default;
MotionProvider::~MotionProvider() = default;

bool MotionProvider::initialize() {
    dataRefs_ = std::make_unique<DataRefManager>();
    dataRefs_->initialize();

    statusWindow_ = std::make_unique<StatusWindow>();
    statusWindow_->initialize();
    statusWindow_->setCommandCallback([this](int a){ onUiAction(a); });

    kin_ = std::make_unique<StewartKinematics>(
        MotionConfig::loadGeometry(MotionConfig::defaultPath()));

    XPLMDebugString("MotionProvider: initialized\n");
    return true;
}

void MotionProvider::shutdown() {
    if (statusWindow_) {
        statusWindow_->destroy();
        statusWindow_.reset();
    }
    dataRefs_.reset();
    kin_.reset();
}

void MotionProvider::reloadConfig() {
    StewartGeometry g = MotionConfig::loadGeometry(MotionConfig::defaultPath());
    kin_ = std::make_unique<StewartKinematics>(g);
    lastReloadOk_ = true;                 // loadGeometry never throws; defaults on error
    reloadFlashRemaining_ = 2.0f;         // show "Config loaded" for ~2 s
}

Pose MotionProvider::currentPose() const {
    if (manualMode_) return manualPose_;
    // AUTO placeholder: aircraft attitude -> platform tilt, clamped so it stays
    // reachable. Replaced by the washout filter in Phase 3.
    auto clampf = [](float v, float lo, float hi){ return std::max(lo, std::min(hi, v)); };
    Pose p;
    p.roll  = clampf(latestCues_.rollDeg,  -8.0f, 8.0f);
    p.pitch = clampf(latestCues_.pitchDeg, -8.0f, 8.0f);
    return p;
}

void MotionProvider::onUiAction(int action) {
    const float kTransStep = 2.0f;   // mm
    const float kRotStep   = 0.5f;   // deg
    switch (action) {
        case UI_RELOAD:      reloadConfig(); break;
        case UI_TOGGLE_MODE: manualMode_ = !manualMode_; break;
        case UI_NEXT_AXIS:   manualAxis_ = (manualAxis_ + 1) % 6; break;
        case UI_RESET:       manualPose_ = Pose{}; break;
        case UI_NUDGE_PLUS:
        case UI_NUDGE_MINUS: {
            const float dir = (action == UI_NUDGE_PLUS) ? 1.0f : -1.0f;
            switch (manualAxis_) {
                case 0: manualPose_.surge += dir * kTransStep; break;
                case 1: manualPose_.sway  += dir * kTransStep; break;
                case 2: manualPose_.heave += dir * kTransStep; break;
                case 3: manualPose_.roll  += dir * kRotStep;   break;
                case 4: manualPose_.pitch += dir * kRotStep;   break;
                case 5: manualPose_.yaw   += dir * kRotStep;   break;
            }
            break;
        }
        default: break;
    }
    // Re-solve and refresh immediately so the click has instant visual feedback.
    if (kin_) latestSolve_ = kin_->solve(currentPose());
    pushStatus();
}

void MotionProvider::pushStatus() {
    if (!statusWindow_) return;
    StatusData sd;
    sd.cues = latestCues_;
    sd.solve = latestSolve_;
    sd.manualMode = manualMode_;
    sd.manualAxis = manualAxis_;
    sd.manualPose = manualPose_;
    sd.lastReloadOk = lastReloadOk_;
    sd.reloadFlash = reloadFlashRemaining_ > 0.0f;
    statusWindow_->update(sd);
}

void MotionProvider::onFlightLoopTick(float elapsedSec) {
    if (dataRefs_) {
        latestCues_ = dataRefs_->sample();
    }
    if (reloadFlashRemaining_ > 0.0f) {
        reloadFlashRemaining_ -= elapsedSec;
        if (reloadFlashRemaining_ < 0.0f) reloadFlashRemaining_ = 0.0f;
    }
    if (kin_) latestSolve_ = kin_->solve(currentPose());

    statusAccumSec_ += elapsedSec;
    if (statusAccumSec_ >= 1.0f) {
        statusAccumSec_ = 0.0f;
        pushStatus();
    }
}

void MotionProvider::onAircraftLoaded() {
    if (dataRefs_) {
        dataRefs_->onAircraftLoaded();
    }
}
