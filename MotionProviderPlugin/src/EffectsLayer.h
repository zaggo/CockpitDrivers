#pragma once
#include "Pose.h"
#include "MotionCues.h"
#include "EffectsConfig.h"

// Additive motion effects layered on top of the washout pose. Stateful; time
// via dt only (deterministic - phase-advanced sines, no RNG). No X-Plane deps.
class EffectsLayer {
public:
    explicit EffectsLayer(const EffectsConfig& cfg);

    Pose update(const MotionCues& cues, double dt);  // additive offset
    void reset();
    void setConfig(const EffectsConfig& cfg) { cfg_ = cfg; }

private:
    EffectsConfig cfg_;
    bool   prevOnGround_ = false;
    bool   tdActive_ = false;
    double tdT_ = 0.0;          // seconds since touchdown
    double rumblePhase_ = 0.0;  // rad
};
