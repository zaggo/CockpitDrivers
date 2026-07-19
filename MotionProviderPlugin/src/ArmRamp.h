#pragma once

// Arm/disarm soft-start state machine. Pure, deterministic (dt-driven), no deps.
// blend() is 0 at the park pose and 1 at the live pose; the owner blends the
// commanded pose between park and live by this factor.
enum class ArmState { Disarmed, Arming, Armed, Disarming };

class ArmRamp {
public:
    // Request the opposite of the current intent: disarmed/disarming -> arming,
    // armed/arming -> disarming. Ramps continue smoothly from the current blend.
    void toggle() {
        if (state_ == ArmState::Disarmed || state_ == ArmState::Disarming)
            state_ = ArmState::Arming;
        else
            state_ = ArmState::Disarming;
    }

    // Force a ramp-down to the park pose (e.g. on a serial-port change).
    void requestDisarm() {
        if (state_ != ArmState::Disarmed) state_ = ArmState::Disarming;
    }

    void update(double dt, double armRampSec, double disarmRampSec) {
        if (dt < 0.0) dt = 0.0;
        switch (state_) {
            case ArmState::Arming: {
                const double rate = (armRampSec > 1e-6) ? (dt / armRampSec) : 1.0;
                blend_ += rate;
                if (blend_ >= 1.0) { blend_ = 1.0; state_ = ArmState::Armed; }
                break;
            }
            case ArmState::Disarming: {
                const double rate = (disarmRampSec > 1e-6) ? (dt / disarmRampSec) : 1.0;
                blend_ -= rate;
                if (blend_ <= 0.0) { blend_ = 0.0; state_ = ArmState::Disarmed; }
                break;
            }
            case ArmState::Armed:    blend_ = 1.0; break;
            case ArmState::Disarmed: blend_ = 0.0; break;
        }
    }

    double   blend() const { return blend_; }   // 0 = park, 1 = live
    ArmState state() const { return state_; }
    bool     fullyDisarmed() const { return state_ == ArmState::Disarmed; }

private:
    ArmState state_ = ArmState::Disarmed;
    double   blend_ = 0.0;
};
