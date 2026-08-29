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

    // The layer's complete internal state -- everything `update()` carries
    // between ticks. Exists so a recording can capture it (Telemetry's eff_*
    // state columns) and a replay can seed from it (washout_replay's
    // loadCues/runChain): rumblePhase_ in particular is a free-running
    // oscillator that never decays on its own, so an unrecorded rumblePhase_
    // is not a transient a warm-up can wash out -- it has to be seeded
    // instead. Read-only by design; the only way in is setState/reset.
    struct State {
        bool   prevOnGround = false;
        bool   tdActive     = false;
        double tdT          = 0.0;  // seconds since touchdown
        double rumblePhase  = 0.0;  // rad
    };
    State state() const { return {prevOnGround_, tdActive_, tdT_, rumblePhase_}; }
    void  setState(const State& s) {
        prevOnGround_ = s.prevOnGround;
        tdActive_     = s.tdActive;
        tdT_          = s.tdT;
        rumblePhase_  = s.rumblePhase;
    }

private:
    EffectsConfig cfg_;
    bool   prevOnGround_ = false;
    bool   tdActive_ = false;
    double tdT_ = 0.0;          // seconds since touchdown
    double rumblePhase_ = 0.0;  // rad
};
