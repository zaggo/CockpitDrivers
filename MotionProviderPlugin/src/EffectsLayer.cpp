#include "EffectsLayer.h"
#include <cmath>

namespace {
constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
// Fraction of the gap between slab joints a single step may occupy. Margin for
// the aircraft accelerating between one joint and the next; see the call site.
constexpr double kSlabDuty = 0.8;
double clampd(double v, double lo, double hi){ return v<lo?lo:(v>hi?hi:v); }
}

EffectsLayer::EffectsLayer(const EffectsConfig& cfg) : cfg_(cfg) {}

void EffectsLayer::reset() {
    prevOnGround_ = false;
    tdActive_ = false;
    tdT_ = 0.0;
    rumblePhase_ = 0.0;
    slabDist_ = 0.0;
    slabFrom_ = 0.0;
    slabTo_   = 0.0;
    slabT_    = 0.0;
    slabDur_  = 0.0;
}

void EffectsLayer::startSlabMove(double target, double availableSec) {
    const double budget = cfg_.slabAccelMmS2;
    double delta = target - slabTo_;
    if (budget <= 0.0 || delta == 0.0) return;

    // Shaped as x(tau) = from + delta * (tau - sin(2*pi*tau)/(2*pi)), tau = t/T.
    // Velocity is zero at both ends and peak acceleration is 2*pi*|delta|/T^2,
    // so the smallest T that respects the budget is sqrt(2*pi*|delta|/budget).
    double dur = std::sqrt(kTwoPi * std::fabs(delta) / budget);

    // If the next joint arrives before that, shrink the step instead of
    // overrunning the budget -- at 60 m/s over 5 m slabs there is only 83 ms.
    if (availableSec > 0.0 && dur > availableSec) {
        dur = availableSec;
        const double maxDelta = budget * dur * dur / kTwoPi;
        delta = (delta > 0.0 ? maxDelta : -maxDelta);
        if (delta == 0.0) return;
    }

    slabFrom_ = slabTo_;
    slabTo_   = slabFrom_ + delta;
    slabT_    = 0.0;
    slabDur_  = dur;
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

    // Runway slab joints: one alternating step per slab, triggered by distance.
    if (cfg_.slabSpacingM > 0.0) {
        const bool rolling = c.onGround && c.groundspeed > cfg_.slabMinSpeedMps;
        if (rolling) {
            slabDist_ += static_cast<double>(c.groundspeed) * dt;
            if (slabDist_ >= cfg_.slabSpacingM) {
                slabDist_ = std::fmod(slabDist_, cfg_.slabSpacingM);
                // Skip a joint that arrives while the previous one is still
                // being rendered. Interrupting mid-move would restart a
                // zero-start-velocity profile from a moving platform, i.e. a
                // velocity step -- exactly the kind of demand the limiter then
                // clips. Dropping the joint instead is what really happens when
                // slabs pass faster than the suspension can follow.
                if (slabDur_ <= 0.0) {
                    // Alternate: a joint that stepped up is followed by one
                    // that steps down, so the platform never walks off level.
                    const double step = (slabTo_ > 0.0) ? -cfg_.slabStepMm : cfg_.slabStepMm;
                    // Fit the move into rather less than the gap to the next
                    // joint. The gap is estimated from the CURRENT speed, but
                    // on a takeoff roll the aircraft is accelerating, so the
                    // next joint arrives sooner than that estimate -- and a
                    // move still running when it does gets the joint skipped.
                    // Measured at 10 m spacing: without this margin a 2 mm step
                    // lost 14 % of its joints and a 3 mm step 18 %, which is a
                    // stuttering rhythm rather than a regular one.
                    startSlabMove(step, kSlabDuty * cfg_.slabSpacingM /
                                            static_cast<double>(c.groundspeed));
                }
            }
        } else if (slabDur_ <= 0.0 && slabTo_ != 0.0) {
            // Stopped or airborne: return to level, or leaving the ground would
            // strand a permanent offset.
            slabDist_ = 0.0;
            startSlabMove(0.0, -1.0);
        }

        if (slabDur_ > 0.0) {
            slabT_ += dt;
            if (slabT_ >= slabDur_) {
                slabDur_ = 0.0;   // move complete; hold at slabTo_
                slabT_   = 0.0;
            } else {
                const double tau = slabT_ / slabDur_;
                off.heave += static_cast<float>(
                    slabFrom_ + (slabTo_ - slabFrom_) * (tau - std::sin(kTwoPi * tau) / kTwoPi));
            }
        }
        if (slabDur_ <= 0.0) off.heave += static_cast<float>(slabTo_);
    }

    // engineGain / buffetGain reserved (not wired this phase).
    return off;
}
