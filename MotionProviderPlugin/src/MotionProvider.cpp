#include "MotionProvider.h"
#include "StatusWindow.h"
#include "DataRefManager.h"
#include "MotionConfig.h"
#include "WashoutFilter.h"
#include "EffectsLayer.h"
#include "XPLMUtilities.h"

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

    washout_ = std::make_unique<WashoutFilter>(MotionConfig::loadWashout(MotionConfig::defaultPath()));
    effects_ = std::make_unique<EffectsLayer>(MotionConfig::loadEffects(MotionConfig::defaultPath()));

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
    washout_.reset();
    effects_.reset();
}

void MotionProvider::reloadConfig() {
    bool loaded = false;
    const std::string path = MotionConfig::defaultPath();
    kin_ = std::make_unique<StewartKinematics>(MotionConfig::loadGeometry(path, &loaded));
    if (washout_) { washout_->setConfig(MotionConfig::loadWashout(path)); washout_->reset(); }
    if (effects_) { effects_->setConfig(MotionConfig::loadEffects(path)); effects_->reset(); }
    lastReloadOk_ = loaded;
    reloadFlashRemaining_ = 2.0f;
}

void MotionProvider::onUiAction(int action) {
    const float kTransStep = 2.0f;   // mm
    const float kRotStep   = 0.5f;   // deg
    switch (action) {
        case UI_RELOAD:      reloadConfig(); break;
        case UI_TOGGLE_MODE:
            manualMode_ = !manualMode_;
            if (!manualMode_ && washout_ && effects_) { washout_->reset(); effects_->reset(); }
            break;
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
    if (kin_ && manualMode_) { latestPose_ = manualPose_; latestSolve_ = kin_->solve(manualPose_); }
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
    sd.commandedPose = latestPose_;
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
    Pose pose;
    if (manualMode_) {
        pose = manualPose_;
    } else if (washout_ && effects_) {
        Pose w = washout_->update(latestCues_, static_cast<double>(elapsedSec));
        Pose e = effects_->update(latestCues_, static_cast<double>(elapsedSec));
        pose.surge = w.surge + e.surge;
        pose.sway  = w.sway  + e.sway;
        pose.heave = w.heave + e.heave;
        pose.roll  = w.roll  + e.roll;
        pose.pitch = w.pitch + e.pitch;
        pose.yaw   = w.yaw   + e.yaw;
    }
    latestPose_ = pose;
    if (kin_) latestSolve_ = kin_->solve(pose);

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
