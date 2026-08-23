#pragma once
#include "Pose.h"
#include "MotionCues.h"
#include "WashoutConfig.h"

// Classical (pragmatic) motion-cueing washout: flight cues -> platform pose.
// Stateful; all time dependence is via the dt argument. No X-Plane deps.
class WashoutFilter {
public:
    explicit WashoutFilter(const WashoutConfig& cfg);

    Pose update(const MotionCues& cues, double dt);
    void reset();
    void setConfig(const WashoutConfig& cfg) { cfg_ = cfg; }

private:
    WashoutConfig cfg_;

    // Heave state
    double heaveAccelLp_ = 0.0;
    double heaveVel_ = 0.0;
    double heavePos_ = 0.0;

    // Tilt-coordination state
    double surgeLp_ = 0.0;
    double swayLp_ = 0.0;
    double tiltPitch_ = 0.0;
    double tiltRoll_ = 0.0;

    // Rotational state
    double rollRateLp_ = 0.0, pitchRateLp_ = 0.0, yawRateLp_ = 0.0;
    double rollAngle_ = 0.0, pitchAngle_ = 0.0, yawAngle_ = 0.0;

    // Output smoothing state (two cascaded one-pole LPs per DOF; heave, roll,
    // pitch, yaw - surge/sway are always 0 here). Active when cfg_.smoothTau > 0.
    double sm1_[4] = {0, 0, 0, 0};
    double sm2_[4] = {0, 0, 0, 0};
};
