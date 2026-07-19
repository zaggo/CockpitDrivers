#include "MotionProvider.h"
#include "StatusWindow.h"
#include "DataRefManager.h"
#include "MotionConfig.h"
#include "WashoutFilter.h"
#include "EffectsLayer.h"
#include "SerialLink.h"
#include "SafetyLimiter.h"
#include "BffEncoder.h"
#include "ConfigUtils.h"
#include "XPLMUtilities.h"

MotionProvider::MotionProvider() = default;
MotionProvider::~MotionProvider() = default;

bool MotionProvider::initialize() {
    dataRefs_ = std::make_unique<DataRefManager>();
    dataRefs_->initialize();

    statusWindow_ = std::make_unique<StatusWindow>();
    statusWindow_->initialize();
    statusWindow_->setCommandCallback([this](int a){ onUiAction(a); });
    statusWindow_->setPortSelectedCallback([this](const std::string& p){ selectPort(p); });

    kin_ = std::make_unique<StewartKinematics>(
        MotionConfig::loadGeometry(MotionConfig::defaultPath()));

    washout_ = std::make_unique<WashoutFilter>(MotionConfig::loadWashout(MotionConfig::defaultPath()));
    effects_ = std::make_unique<EffectsLayer>(MotionConfig::loadEffects(MotionConfig::defaultPath()));

    safetyCfg_ = MotionConfig::loadSafety(MotionConfig::defaultPath());
    safety_ = std::make_unique<SafetyLimiter>(safetyCfg_);
    // Park pose = lowest reachable pose along parkHeaveMm; start disarmed there.
    Pose parkTarget; parkTarget.heave = static_cast<float>(safetyCfg_.parkHeaveMm);
    parkPose_ = kin_->clampToReachable(parkTarget);
    SolveResult parkSolve = kin_->solve(parkPose_);
    safety_->reset(parkSolve.setpoints);

    serial_ = std::make_unique<SerialLink>();
    SerialConfig sc = MotionConfig::loadSerial(MotionConfig::defaultPath());
    std::string lastPort = loadLastUsedPort();   // ConfigUtils (~/.motionprovider.cfg)
    if (!lastPort.empty()) {
        serial_->configure(lastPort, sc.baud, sc.rateHz);
        serial_->connect();   // opens DISARMED - streams home until armed
    }

    XPLMDebugString("MotionProvider: initialized\n");
    return true;
}

void MotionProvider::shutdown() {
    if (statusWindow_) {
        statusWindow_->destroy();
        statusWindow_.reset();
    }
    if (serial_) { serial_->stop(); }
    serial_.reset();
    safety_.reset();
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
    safetyCfg_ = MotionConfig::loadSafety(path);
    if (safety_) safety_->setConfig(safetyCfg_);
    // Recompute the park pose (geometry and/or park heave may have changed).
    Pose parkTarget; parkTarget.heave = static_cast<float>(safetyCfg_.parkHeaveMm);
    parkPose_ = kin_->clampToReachable(parkTarget);
    // serial rate/baud change needs a reconnect to take effect:
    if (serial_) {
        SerialConfig sc = MotionConfig::loadSerial(path);
        serial_->configure(serial_->port(), sc.baud, sc.rateHz);
    }
    lastReloadOk_ = loaded;
    reloadFlashRemaining_ = 2.0f;
}

void MotionProvider::selectPort(const std::string& port) {
    if (!serial_) return;
    serial_->stop();
    SerialConfig sc = MotionConfig::loadSerial(MotionConfig::defaultPath());
    serial_->configure(port, sc.baud, sc.rateHz);
    serial_->connect();
    saveLastUsedPort(port);      // ConfigUtils persist
    armRamp_.requestDisarm();    // ramp down to park; re-arm deliberately after a port change
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
        case UI_ARM_TOGGLE:  armRamp_.toggle(); break;
        case UI_RESCAN_PORTS: /* window rescans; nothing to do here */ break;
        default: break;
    }
    // Re-solve and refresh immediately so the click has instant visual feedback.
    if (kin_ && manualMode_) {
        latestPose_ = blendedCommand(manualPose_);
        latestSolve_ = kin_->solve(latestPose_);
    }
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
    for (int i=0;i<6;i++) sd.sentSetpoints[i] = sentSetpoints_[i];
    sd.armState = static_cast<int>(armRamp_.state());
    sd.armBlend = static_cast<float>(armRamp_.blend());
    sd.serialConnected = serial_ && serial_->isConnected();
    sd.framesSent = serial_ ? serial_->framesSent() : 0;
    sd.serialPort = serial_ ? serial_->port() : std::string();
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
    Pose rawLive;
    if (manualMode_) {
        rawLive = manualPose_;
    } else if (washout_ && effects_) {
        Pose w = washout_->update(latestCues_, static_cast<double>(elapsedSec));
        Pose e = effects_->update(latestCues_, static_cast<double>(elapsedSec));
        rawLive.surge = w.surge + e.surge;
        rawLive.sway  = w.sway  + e.sway;
        rawLive.heave = w.heave + e.heave;
        rawLive.roll  = w.roll  + e.roll;
        rawLive.pitch = w.pitch + e.pitch;
        rawLive.yaw   = w.yaw   + e.yaw;
    }

    // Arm/disarm soft-start: advance the ramp, then command a pose blended
    // between the park pose (disarmed) and the live pose (armed). Pose-space
    // blend + reachability clamp keeps every intermediate a valid rigid config.
    armRamp_.update(static_cast<double>(elapsedSec), safetyCfg_.armRampSec, safetyCfg_.disarmRampSec);
    latestPose_ = blendedCommand(rawLive);
    if (kin_) latestSolve_ = kin_->solve(latestPose_);

    // SafetyLimiter is the final velocity/acceleration backstop on the setpoints.
    uint16_t target[6];
    for (int i = 0; i < 6; ++i) target[i] = latestSolve_.setpoints[i];
    if (safety_) safety_->limit(target, static_cast<double>(elapsedSec), sentSetpoints_);
    else for (int i=0;i<6;i++) sentSetpoints_[i] = target[i];

    if (serial_) {
        uint8_t frame[BffEncoder::kFrameSize];
        BffEncoder::encode(sentSetpoints_, frame);
        serial_->setFrame(frame, sizeof(frame));
        serial_->update(elapsedSec);
    }

    statusAccumSec_ += elapsedSec;
    if (statusAccumSec_ >= 1.0f) { statusAccumSec_ = 0.0f; pushStatus(); }
}

void MotionProvider::onAircraftLoaded() {
    if (dataRefs_) {
        dataRefs_->onAircraftLoaded();
    }
}

Pose MotionProvider::blendedCommand(const Pose& rawLive) const {
    if (!kin_) return rawLive;
    const Pose live = kin_->clampToReachable(rawLive);
    const double b = armRamp_.blend();           // 0 = park, 1 = live
    const double p = 1.0 - b;
    Pose eff;
    eff.surge = static_cast<float>(parkPose_.surge * p + live.surge * b);
    eff.sway  = static_cast<float>(parkPose_.sway  * p + live.sway  * b);
    eff.heave = static_cast<float>(parkPose_.heave * p + live.heave * b);
    eff.roll  = static_cast<float>(parkPose_.roll  * p + live.roll  * b);
    eff.pitch = static_cast<float>(parkPose_.pitch * p + live.pitch * b);
    eff.yaw   = static_cast<float>(parkPose_.yaw   * p + live.yaw   * b);
    return kin_->clampToReachable(eff);           // guard the blended pose too
}
