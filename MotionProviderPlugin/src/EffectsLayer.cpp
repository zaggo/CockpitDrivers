#include "EffectsLayer.h"
#include <cmath>

namespace {
constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
double clampd(double v, double lo, double hi){ return v<lo?lo:(v>hi?hi:v); }
}

EffectsLayer::EffectsLayer(const EffectsConfig& cfg) : cfg_(cfg) {}

void EffectsLayer::reset() {
    prevOnGround_ = false;
    tdActive_ = false;
    tdT_ = 0.0;
    rumblePhase_ = 0.0;
}

Pose EffectsLayer::update(const MotionCues& c, double dt) {
    if (dt <= 0.0) dt = 1.0 / 60.0;
    Pose off;  // all zero

    // Touchdown bump on rising edge of onGround.
    if (c.onGround && !prevOnGround_) { tdActive_ = true; tdT_ = 0.0; }
    prevOnGround_ = c.onGround;
    if (tdActive_) {
        tdT_ += dt;
        const double env = std::exp(-tdT_ / cfg_.touchdownDecayTau);
        off.heave += static_cast<float>(
            -cfg_.touchdownGain * env * std::sin(kTwoPi * cfg_.touchdownFreqHz * tdT_));
        if (env < 0.02) tdActive_ = false;
    }

    // Ground-roll rumble while moving on the ground.
    if (c.onGround && c.groundspeed > 0.5f) {
        const double amp = cfg_.rumbleGain *
            clampd(static_cast<double>(c.groundspeed) / cfg_.rumbleSpeedRefMps, 0.0, 1.0);
        rumblePhase_ += kTwoPi * cfg_.rumbleFreqHz * dt;
        if (rumblePhase_ > kTwoPi) rumblePhase_ -= kTwoPi;
        off.heave += static_cast<float>(amp * std::sin(rumblePhase_));
        off.pitch += static_cast<float>(0.1 * amp * std::sin(rumblePhase_ * 1.7));
    }

    // engineGain / buffetGain reserved (not wired this phase).
    return off;
}
