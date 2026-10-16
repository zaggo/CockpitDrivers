#include "AttitudeGyro.h"

#include <cmath>

namespace {

constexpr float kTwoPi = 6.283185307179586f;

float clamp01(float v)
{
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

float wrapPhase(float phase)
{
    phase = std::fmod(phase, kTwoPi);
    return phase < 0.0f ? phase + kTwoPi : phase;
}

} // namespace

AttitudeGyro::AttitudeGyro() = default;

void AttitudeGyro::update(float vacuumInHg, float dt)
{
    if (!std::isfinite(vacuumInHg) || !std::isfinite(dt))
    {
        return;
    }
    if (dt < 0.0f)
        dt = 0.0f;
    if (dt > 0.25f)
        dt = 0.25f;

    const float target = clamp01(vacuumInHg / kNominalSuctionInHg);

    // Exact first-order lag toward the target RPM - stable for any dt.
    const float tau = (target > rotorSpeed_) ? kSpinUpTauSec : kSpinDownTauSec;
    rotorSpeed_ = target + (rotorSpeed_ - target) * std::exp(-dt / tau);

    const float rel = 1.0f - rotorSpeed_; // degradation factor, 0 = healthy

    // Wander: very slow pseudo-drift from two incommensurate sines per axis,
    // present as soon as the rotor droops and growing with rel.
    wanderPhasePitch_ = wrapPhase(wanderPhasePitch_ + kTwoPi * 0.045f * dt); // ~22 s period
    wanderPhaseRoll_ = wrapPhase(wanderPhaseRoll_ + kTwoPi * 0.031f * dt);   // ~32 s period
    pitchErr_ = kWanderPitchDeg * rel *
                (0.6f * std::sin(wanderPhasePitch_) +
                 0.4f * std::sin(wanderPhasePitch_ * 3.7f + 1.3f));
    rollErr_ = kWanderRollDeg * rel *
               (0.6f * std::sin(wanderPhaseRoll_) +
                0.4f * std::sin(wanderPhaseRoll_ * 2.9f + 0.7f));

    // Nutation wobble ("taumeln"): ~1 Hz, amplitude peaks at half rotor speed
    // and vanishes both at full speed and at a standstill.
    wobblePhase_ = wrapPhase(wobblePhase_ + kTwoPi * 0.9f * dt);
    const float wobble = kWobbleDeg * 4.0f * rel * rotorSpeed_;
    pitchErr_ += wobble * std::sin(wobblePhase_);
    rollErr_ += wobble * std::sin(wobblePhase_ * 1.31f + 0.9f);

    // Below ~a quarter speed the horizon starts falling toward its rest pose.
    float w = clamp01((kRestBeginSpeed - rotorSpeed_) / (kRestBeginSpeed - kRestFullSpeed));
    restWeight_ = w * w * (3.0f - 2.0f * w); // smoothstep
}

void AttitudeGyro::apply(float& pitchDeg, float& rollDeg) const
{
    pitchDeg += pitchErr_;
    rollDeg += rollErr_;
    pitchDeg += (kRestPitchDeg - pitchDeg) * restWeight_;
    rollDeg += (kRestRollDeg - rollDeg) * restWeight_;
}
