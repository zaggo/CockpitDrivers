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
        // Slab joints. slabDist is an odometer and slabTo a held offset --
        // neither decays, so both have to be recorded and seeded for the same
        // reason rumblePhase does.
        double slabDist     = 0.0;  // m travelled since the last joint
        double slabFrom     = 0.0;  // mm, offset the current move started from
        double slabTo       = 0.0;  // mm, offset it ends at (held when idle)
        double slabT        = 0.0;  // s into the current move
        double slabDur      = 0.0;  // s, total duration; 0 = idle
    };
    State state() const {
        return {prevOnGround_, tdActive_, tdT_, rumblePhase_,
                slabDist_, slabFrom_, slabTo_, slabT_, slabDur_};
    }
    void  setState(const State& s) {
        prevOnGround_ = s.prevOnGround;
        tdActive_     = s.tdActive;
        tdT_          = s.tdT;
        rumblePhase_  = s.rumblePhase;
        slabDist_     = s.slabDist;
        slabFrom_     = s.slabFrom;
        slabTo_       = s.slabTo;
        slabT_        = s.slabT;
        slabDur_      = s.slabDur;
    }

private:
    // Start a rest-to-rest move to `target` mm. `availableSec` is the time
    // before the next joint is due; the step is shrunk rather than the budget
    // exceeded if it does not fit. Pass a negative value for "no deadline".
    void startSlabMove(double target, double availableSec);

    EffectsConfig cfg_;
    bool   prevOnGround_ = false;
    bool   tdActive_ = false;
    double tdT_ = 0.0;          // seconds since touchdown
    double rumblePhase_ = 0.0;  // rad
    double slabDist_ = 0.0;     // m since last joint
    double slabFrom_ = 0.0;     // mm
    double slabTo_   = 0.0;     // mm
    double slabT_    = 0.0;     // s into the move
    double slabDur_  = 0.0;     // s, 0 = idle
};
