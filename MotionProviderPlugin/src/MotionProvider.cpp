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

    safety_ = std::make_unique<SafetyLimiter>(MotionConfig::loadSafety(MotionConfig::defaultPath()));
    uint16_t home[6]; for (int i=0;i<6;i++) home[i]=32640;
    safety_->reset(home);

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
    if (safety_) safety_->setConfig(MotionConfig::loadSafety(path));
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
    armed_ = false;              // always re-arm deliberately after a port change
}

void MotionProvider::rescanPorts() { /* handled in the window via enumerateSerialPorts */ }

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
        case UI_ARM_TOGGLE:  armed_ = !armed_; break;
        case UI_RESCAN_PORTS: /* window rescans; nothing to do here */ break;
        default: break;
    }
    // Re-solve and refresh immediately so the click has instant visual feedback.
    if (kin_ && manualMode_) { Pose r = kin_->clampToReachable(manualPose_); latestPose_=r; latestSolve_=kin_->solve(r); }
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
    sd.armed = armed_;
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
    if (kin_) {
        Pose reachable = kin_->clampToReachable(pose);
        latestPose_ = reachable;
        latestSolve_ = kin_->solve(reachable);
    }

    // Arm gate: disarmed -> stream home; armed -> live setpoints. Either target
    // passes through the SafetyLimiter so disarm/home is a smooth ramp.
    uint16_t target[6];
    for (int i = 0; i < 6; ++i)
        target[i] = armed_ ? latestSolve_.setpoints[i] : static_cast<uint16_t>(32640);
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
