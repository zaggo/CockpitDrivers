#pragma once
#include "Pose.h"
#include "MotionCues.h"
#include "WashoutConfig.h"

// Per-tick snapshot of the filter's internals. Written on every update();
// read by Telemetry and by the offline replay tool. Purely observational --
// nothing here feeds back into the filter.
struct WashoutTrace {
    double heaveAHp       = 0.0;   // high-passed vertical specific force, m/s^2
    double heaveVel       = 0.0;   // leaky velocity integrator, m/s
    double heavePosRaw    = 0.0;   // position BEFORE the +/-heaveLimitMm clamp, mm
    bool   heaveClamped   = false;
    double tiltPitch      = 0.0;   // deg, after rate limiting
    double tiltRoll       = 0.0;
    bool   tiltRateActive = false; // rate limiter engaged on either tilt axis
    double rotRollRaw     = 0.0;   // angles BEFORE the +/-rotLimitDeg clamp, deg
    double rotPitchRaw    = 0.0;
    double rotYawRaw      = 0.0;
    bool   rotRollClamped  = false;
    bool   rotPitchClamped = false;
    bool   rotYawClamped   = false;
};

// Classical (pragmatic) motion-cueing washout: flight cues -> platform pose.
// Stateful; all time dependence is via the dt argument. No X-Plane deps.
class WashoutFilter {
public:
    explicit WashoutFilter(const WashoutConfig& cfg);

    Pose update(const MotionCues& cues, double dt);
    void reset();
    void setConfig(const WashoutConfig& cfg) { cfg_ = cfg; }
    const WashoutTrace& trace() const { return trace_; }

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

    WashoutTrace trace_;
};
