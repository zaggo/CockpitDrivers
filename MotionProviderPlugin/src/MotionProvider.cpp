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
#include "XPLMPlugin.h"
#include <cmath>
#include <fstream>
#include <string>

namespace {
constexpr double kIdentifyAmpDeg    = 5.0;   // jitter amplitude, deg
constexpr double kIdentifyPeriodSec = 1.0;   // jitter period
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kHeartbeatMaxAgeSec = 1.5;   // 3x the gateway's 500ms send period

// Resolve <plugins>/MotionProvider/configuration.toml from this plugin's own
// .xpl path (.../MotionProvider/<arch>/mac.xpl -> strip filename + arch dir).
// Returns "" if the API path is unusable (caller falls back to ~).
std::string resolvePluginConfigPath() {
    char path[512] = {0};
    XPLMGetPluginInfo(XPLMGetMyID(), nullptr, path, nullptr, nullptr);
    std::string p(path);
    const size_t f = p.find_last_of("/\\");
    if (f == std::string::npos) return "";
    const std::string archDir = p.substr(0, f);          // .../MotionProvider/<arch>
    const size_t a = archDir.find_last_of("/\\");
    if (a == std::string::npos) return "";
    return archDir.substr(0, a) + "/configuration.toml";  // .../MotionProvider/
}
}  // namespace

MotionProvider::MotionProvider() = default;
MotionProvider::~MotionProvider() = default;

bool MotionProvider::initialize() {
    dataRefs_ = std::make_unique<DataRefManager>();
    dataRefs_->initialize();

    statusWindow_ = std::make_unique<StatusWindow>();
    statusWindow_->initialize();
    statusWindow_->setCommandCallback([this](int a){ onUiAction(a); });
    statusWindow_->setPortSelectedCallback([this](const std::string& p){ selectPort(p); });

    // configuration.toml lives in the plugin directory (next to the arch folder);
    // fall back to ~ if the plugin path can't be resolved. Seed it with defaults
    // if it doesn't exist yet.
    configPath_ = resolvePluginConfigPath();
    if (configPath_.empty()) configPath_ = MotionConfig::defaultPath();
    {
        std::ifstream test(configPath_);
        if (!test.good()) MotionConfig::writeDefaults(configPath_);
    }

    kin_ = std::make_unique<StewartKinematics>(MotionConfig::loadGeometry(configPath_));

    washout_ = std::make_unique<WashoutFilter>(MotionConfig::loadWashout(configPath_));
    effects_ = std::make_unique<EffectsLayer>(MotionConfig::loadEffects(configPath_));

    safetyCfg_ = MotionConfig::loadSafety(configPath_);
    safety_ = std::make_unique<SafetyLimiter>(safetyCfg_);
    monitor_.setConfig(safetyCfg_);
    // Park pose = lowest reachable pose along parkHeaveMm; start disarmed there
    // with the limiter pre-seeded to the park setpoints (no startup jump).
    recomputeParkPose();
    SolveResult parkSolve = kin_->solve(parkPose_);
    safety_->reset(parkSolve.setpoints);

    serial_ = std::make_unique<SerialLink>();
    SerialConfig sc = MotionConfig::loadSerial(configPath_);
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
    const std::string path = configPath_;
    kin_ = std::make_unique<StewartKinematics>(MotionConfig::loadGeometry(path, &loaded));
    if (washout_) { washout_->setConfig(MotionConfig::loadWashout(path)); washout_->reset(); }
    if (effects_) { effects_->setConfig(MotionConfig::loadEffects(path)); effects_->reset(); }
    safetyCfg_ = MotionConfig::loadSafety(path);
    if (safety_) safety_->setConfig(safetyCfg_);
    monitor_.setConfig(safetyCfg_);
    recomputeParkPose();   // geometry and/or park heave may have changed
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
    SerialConfig sc = MotionConfig::loadSerial(configPath_);
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
            // Switch SIM <-> MANUAL only while fully disarmed (avoids a pose jump).
            if (armRamp_.state() == ArmState::Disarmed) manualMode_ = !manualMode_;
            break;
        case UI_DISARM:      armRamp_.requestDisarm(); armGate_.latchDisarm(); break;
        case UI_NEXT_AXIS:   manualAxis_ = (manualAxis_ + 1) % 7; break;  // 6 = Identify
        case UI_RESET:
            if (manualAxis_ == 6) identifyLeg_ = 0;
            else manualPose_ = Pose{};
            break;
        case UI_NUDGE_PLUS:
        case UI_NUDGE_MINUS: {
            const bool plus = (action == UI_NUDGE_PLUS);
            if (manualAxis_ == 6) {   // Identify: +/- selects the actor
                identifyLeg_ = (identifyLeg_ + (plus ? 1 : 5)) % 6;
                break;
            }
            const float dir = plus ? 1.0f : -1.0f;
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
    sd.identifyActor = (kin_ && identifyLeg_ >= 0 && identifyLeg_ < 6)
                       ? kin_->geometry().legName[identifyLeg_] : "";
    sd.faultCode = static_cast<int>(monitor_.fault());
    sd.faultReason = monitor_.reason();
    sd.serialConnected = serial_ && serial_->isConnected();
    sd.framesSent = serial_ ? serial_->framesSent() : 0;
    sd.serialPort = serial_ ? serial_->port() : std::string();
    sd.heartbeatPresent = serial_ && serial_->heartbeatFresh(kHeartbeatMaxAgeSec);
    sd.heartbeatArmed = serial_ && serial_->heartbeatArmed();
    statusWindow_->update(sd);
}

void MotionProvider::onFlightLoopTick(float elapsedSec) {
    if (dataRefs_) latestCues_ = dataRefs_->sample();

    if (reloadFlashRemaining_ > 0.0f) {
        reloadFlashRemaining_ -= elapsedSec;
        if (reloadFlashRemaining_ < 0.0f) reloadFlashRemaining_ = 0.0f;
    }

    // Pause/stall guard: clamp the timestep the stateful filters/ramp see so a
    // long X-Plane stall can't diverge them. Serial reconnect uses real dt.
    double dt = static_cast<double>(elapsedSec);
    if (dt > safetyCfg_.maxDtSec) dt = safetyCfg_.maxDtSec;

    Pose rawLive;
    if (manualMode_) {
        // Identify mode (axis 6) commands the home pose as base; a per-actuator
        // jitter is added after IK below. Other manual axes use the hand pose.
        rawLive = (manualAxis_ == 6) ? Pose{} : manualPose_;
    } else if (washout_ && effects_) {
        Pose w = washout_->update(latestCues_, dt);
        Pose e = effects_->update(latestCues_, dt);
        rawLive.surge = w.surge + e.surge;  rawLive.sway  = w.sway  + e.sway;
        rawLive.heave = w.heave + e.heave;  rawLive.roll  = w.roll  + e.roll;
        rawLive.pitch = w.pitch + e.pitch;  rawLive.yaw   = w.yaw   + e.yaw;
    }

    // Watchdog + runaway/NaN monitor (before the envelope clamp masks divergence).
    if (serial_ && serial_->isConnected()) serialWasConnected_ = true;
    const bool notDisarmed = (armRamp_.state() != ArmState::Disarmed);
    const bool serialLost = notDisarmed && serialWasConnected_ &&
                            serial_ && !serial_->isConnected();
    const bool finite =
        std::isfinite(rawLive.surge) && std::isfinite(rawLive.sway) &&
        std::isfinite(rawLive.heave) && std::isfinite(rawLive.roll) &&
        std::isfinite(rawLive.pitch) && std::isfinite(rawLive.yaw);
    monitor_.update(rawLive, finite, serialLost, dt);
    // Hardware arm switch. Trust the switch position only from FRESH heartbeat
    // frames; freeze the last known position while the heartbeat is stale, so a
    // transient gap is NOT mistaken for the pilot physically cycling the switch.
    const bool hbFresh = serial_ && serial_->heartbeatFresh(kHeartbeatMaxAgeSec);
    if (hbFresh) lastSwitchArmed_ = serial_->heartbeatArmed();

    // Reset point = a genuine armed->disarmed switch transition. Because
    // lastSwitchArmed_ is frozen while stale, this edge fires only on a fresh
    // frame reporting the switch open — matching a physical E-stop reset (flip
    // off, then on). It clears the fault + e-stop latch.
    if (armGate_.update(lastSwitchArmed_)) monitor_.clear();
    if (monitor_.fault() != FaultCode::None) armGate_.latchDisarm();  // home-on-fault (latched)

    // Arm only when the heartbeat is fresh AND the switch is armed AND not latched.
    // A stale heartbeat forces disarm but must NOT have cleared the latch above.
    const bool armIntent = hbFresh && lastSwitchArmed_ && !armGate_.latched();
    if (armIntent && armRamp_.state() == ArmState::Disarmed) {
        // Rising edge into an auto (SIM) arm: reset the stateful filters so the
        // ramp starts from a clean pose. Manual mode has no filters to reset.
        if (!manualMode_ && washout_ && effects_) { washout_->reset(); effects_->reset(); }
    }
    if (armIntent) armRamp_.requestArm();
    else           armRamp_.requestDisarm();

    // Arm ramp + pose-space blend -> IK.
    armRamp_.update(dt, safetyCfg_.armRampSec, safetyCfg_.disarmRampSec);
    latestPose_ = blendedCommand(rawLive);
    if (kin_) latestSolve_ = kin_->solve(latestPose_);

    uint16_t target[6];
    for (int i = 0; i < 6; ++i) target[i] = latestSolve_.setpoints[i];

    // Identify: jitter only the selected actuator (+-kIdentifyAmpDeg, 1 s period)
    // as an offset on its setpoint, scaled by the arm blend. Others hold at home.
    if (manualMode_ && manualAxis_ == 6 && kin_) {
        const StewartGeometry& gg = kin_->geometry();
        identifyPhase_ += kTwoPi * (1.0 / kIdentifyPeriodSec) * dt;
        if (identifyPhase_ > kTwoPi) identifyPhase_ -= kTwoPi;
        const double amp = (kIdentifyAmpDeg / gg.angleAtFullScale)
                           * static_cast<double>(gg.demandHome) * armRamp_.blend();
        const int slot = gg.bff[identifyLeg_] - 1;
        double v = static_cast<double>(target[slot]) + amp * std::sin(identifyPhase_);
        if (v < 0.0) v = 0.0;
        if (v > static_cast<double>(gg.demandMax)) v = static_cast<double>(gg.demandMax);
        target[slot] = static_cast<uint16_t>(std::lround(v));
    }

    if (safety_) safety_->limit(target, dt, sentSetpoints_);
    else for (int i=0;i<6;i++) sentSetpoints_[i] = target[i];

    if (serial_) {
        uint8_t frame[BffEncoder::kFrameSize];
        BffEncoder::encode(sentSetpoints_, frame);
        serial_->setFrame(frame, sizeof(frame));
        serial_->update(elapsedSec);   // real dt for reconnect timing
    }

    statusAccumSec_ += elapsedSec;
    if (statusAccumSec_ >= 1.0f) { statusAccumSec_ = 0.0f; pushStatus(); }
}

void MotionProvider::onAircraftLoaded() {
    if (dataRefs_) {
        dataRefs_->onAircraftLoaded();
    }
}

void MotionProvider::recomputeParkPose() {
    if (!kin_) return;
    Pose parkTarget;
    parkTarget.heave = static_cast<float>(safetyCfg_.parkHeaveMm);
    parkPose_ = kin_->clampToReachable(parkTarget);
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
