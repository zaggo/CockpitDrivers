#include "WashoutFilter.h"
#include <cmath>

namespace {
constexpr double G = 9.80665;
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;

double lpAlpha(double dt, double tau) { return tau > 0.0 ? dt / (tau + dt) : 1.0; }
double leak(double dt, double tau)    { return tau > 0.0 ? std::exp(-dt / tau) : 0.0; }
double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

double rateLimit(double cur, double tgt, double ratePerSec, double dt) {
    const double step = ratePerSec * dt;
    const double d = tgt - cur;
    if (d >  step) return cur + step;
    if (d < -step) return cur - step;
    return tgt;
}
}  // namespace

WashoutFilter::WashoutFilter(const WashoutConfig& cfg) : cfg_(cfg) {}

void WashoutFilter::reset() {
    heaveAccelLp_ = heaveVel_ = heavePos_ = 0.0;
    surgeLp_ = swayLp_ = tiltPitch_ = tiltRoll_ = 0.0;
    rollRateLp_ = pitchRateLp_ = yawRateLp_ = 0.0;
    rollAngle_ = pitchAngle_ = yawAngle_ = 0.0;
}

Pose WashoutFilter::update(const MotionCues& c, double dt) {
    if (dt <= 0.0) dt = 1.0 / 60.0;

    // --- Heave: HP(accel) -> leaky double-integrate -> mm ---
    const double aZ = cfg_.heaveGain * (static_cast<double>(c.heaveG) - 1.0) * G;
    heaveAccelLp_ += lpAlpha(dt, cfg_.heaveHpTau) * (aZ - heaveAccelLp_);
    const double aHp = aZ - heaveAccelLp_;
    heaveVel_ = heaveVel_ * leak(dt, cfg_.heaveVelWashoutTau) + aHp * dt;
    heavePos_ = heavePos_ * leak(dt, cfg_.heavePosWashoutTau) + heaveVel_ * dt * 1000.0;
    heavePos_ = clampd(heavePos_, -cfg_.heaveLimitMm, cfg_.heaveLimitMm);

    // --- Tilt-coordination: sustained horizontal accel -> rate-limited tilt ---
    const double aX = cfg_.tiltSurgeGain * static_cast<double>(c.surgeG) * G;
    const double aY = cfg_.tiltSwayGain  * static_cast<double>(c.swayG)  * G;
    surgeLp_ += lpAlpha(dt, cfg_.tiltLpTau) * (aX - surgeLp_);
    swayLp_  += lpAlpha(dt, cfg_.tiltLpTau) * (aY - swayLp_);
    double tgtPitch = std::asin(clampd(surgeLp_ / G, -1.0, 1.0)) * kRad2Deg;
    double tgtRoll  = std::asin(clampd(swayLp_  / G, -1.0, 1.0)) * kRad2Deg;
    tgtPitch = clampd(tgtPitch, -cfg_.tiltLimitDeg, cfg_.tiltLimitDeg);
    tgtRoll  = clampd(tgtRoll,  -cfg_.tiltLimitDeg, cfg_.tiltLimitDeg);
    tiltPitch_ = rateLimit(tiltPitch_, tgtPitch, cfg_.tiltRateLimitDps, dt);
    tiltRoll_  = rateLimit(tiltRoll_,  tgtRoll,  cfg_.tiltRateLimitDps, dt);

    // --- Rotational: HP(rate) -> integrate -> washout leak ---
    auto rotChan = [&](double gain, double rate, double& rateLp, double& angle) {
        const double w = gain * rate;                // deg/s
        rateLp += lpAlpha(dt, cfg_.rotHpTau) * (w - rateLp);
        const double wHp = w - rateLp;
        angle = (angle + wHp * dt) * leak(dt, cfg_.rotWashoutTau);
        angle = clampd(angle, -cfg_.rotLimitDeg, cfg_.rotLimitDeg);
    };
    rotChan(cfg_.rotRollGain,  c.rollRate,  rollRateLp_,  rollAngle_);
    rotChan(cfg_.rotPitchGain, c.pitchRate, pitchRateLp_, pitchAngle_);
    rotChan(cfg_.rotYawGain,   c.yawRate,   yawRateLp_,   yawAngle_);

    Pose p;
    p.surge = 0.0f;
    p.sway  = 0.0f;
    p.heave = static_cast<float>(heavePos_);
    p.roll  = static_cast<float>(tiltRoll_  + rollAngle_);
    p.pitch = static_cast<float>(tiltPitch_ + pitchAngle_);
    p.yaw   = static_cast<float>(yawAngle_);
    return p;
}
