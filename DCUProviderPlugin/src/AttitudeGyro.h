#pragma once

// Vacuum-gyro degradation model for the attitude indicator - the only
// vacuum-driven instrument on the rig (the suction gauge isn't installed).
//
// The sim's pitch_vacuum/roll_vacuum datarefs do NOT react to suction loss
// (verified on the rig: they stay stable with sim/cockpit/misc/vacuum at 0),
// so the failure behaviour is modelled here, centrally, driven by the
// measured suction. The model follows how a real spinning-mass AI dies:
//
//   1. The rotor coasts down over a minute or two - nothing changes at first.
//   2. As RPM droops, the gyro precesses: a slow, believable wander creeps
//      into the indicated attitude. This is the insidious phase - the
//      instrument lies smoothly, with no flag.
//   3. Mid spin-down the rotor nutates: a faster wobble ("taumeln") overlays
//      the wander, strongest around half speed, gone again when the rotor has
//      stopped (a dead rotor doesn't nutate).
//   4. Fully stopped, the horizon hangs at its mechanical rest pose,
//      disconnected from whatever the aircraft does.
//
// Engine start runs the same physics backwards: the rotor spins up, wobble
// comes and goes, wander shrinks, and the horizon re-erects onto truth.
//
// All motion here is deterministic (sums of sines, no RNG) so the tests in
// tests/ can assert exact behaviour. XPLM-free like BusPower.
class AttitudeGyro {
public:
    AttitudeGyro();

    // Feed the measured suction (inHg) and the frame delta time, once per
    // downlink tick. Non-finite input is ignored; dt is clamped to 0.25 s so
    // a sim hitch can't jump the phases.
    void update(float vacuumInHg, float dt);

    // Degrades a true attitude into the indicated one. Cheap, const, no state
    // change - safe to call from the rate-limited send block.
    void apply(float& pitchDeg, float& rollDeg) const;

    // Normalized rotor speed, 0..1. 1 = full RPM.
    float rotorSpeed() const { return rotorSpeed_; }

    // Suction (inHg) that spins the rotor to full speed. Measured on the rig:
    // ~4.8-4.9 at full throttle, ~3.4 at idle - idle taxi therefore slowly
    // droops the gyro, exactly like the real pump.
    static constexpr float kNominalSuctionInHg = 4.9f;

    // First-order rotor lag. Spin-down is much slower than spin-up: a healthy
    // rotor coasts for a minute or two, the pump revs it back up faster.
    static constexpr float kSpinUpTauSec = 30.0f;
    static constexpr float kSpinDownTauSec = 75.0f;

    // Amplitudes of the two error components, degrees.
    static constexpr float kWanderPitchDeg = 5.0f;
    static constexpr float kWanderRollDeg = 8.0f;
    static constexpr float kWobbleDeg = 6.0f;

    // Rest pose of a dead gyro, degrees: horizon dropped (indicated climb)
    // and rolled left - the classic "engine off for a while" AI look. Tunable;
    // the AttitudeCAN board clamps to its mechanical window anyway.
    static constexpr float kRestPitchDeg = 8.0f;
    static constexpr float kRestRollDeg = -20.0f;

    // Rotor speeds below which the rest pose blends in / is fully reached.
    static constexpr float kRestBeginSpeed = 0.25f;
    static constexpr float kRestFullSpeed = 0.08f;

private:
    // Starts healthy: until the first update() proves otherwise, behave as if
    // the gyro were at full speed - same fail-live default as BusPower, so
    // loading the plugin onto a running aircraft doesn't tumble the horizon.
    float rotorSpeed_ = 1.0f;

    // Phase accumulators (radians). They run unconditionally; the amplitudes
    // gate them to zero while the rotor is healthy.
    float wanderPhasePitch_ = 0.0f;
    float wanderPhaseRoll_ = 2.4f;
    float wobblePhase_ = 0.0f;

    // Outputs of update(), consumed by apply().
    float pitchErr_ = 0.0f;
    float rollErr_ = 0.0f;
    float restWeight_ = 0.0f; // 0 = live attitude, 1 = rest pose
};
