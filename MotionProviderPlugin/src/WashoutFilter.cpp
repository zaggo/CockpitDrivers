#include "WashoutFilter.h"
#include <cmath>

namespace {
constexpr double G = 9.80665;
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;

double lpAlpha(double dt, double tau) { return tau > 0.0 ? dt / (tau + dt) : 1.0; }
double leak(double dt, double tau)    { return tau > 0.0 ? std::exp(-dt / tau) : 0.0; }
double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

double rateLimit(double cur, double tgt, double ratePerSec, double dt, bool& limited) {
    const double step = ratePerSec * dt;
    const double d = tgt - cur;
    if (d >  step) { limited = true; return cur + step; }
    if (d < -step) { limited = true; return cur - step; }
    return tgt;
}
}  // namespace

WashoutFilter::WashoutFilter(const WashoutConfig& cfg) : cfg_(cfg) {}

void WashoutFilter::reset() {
    heaveAccelLp_ = heaveVel_ = heavePos_ = 0.0;
    surgeLpRaw_ = swayLpRaw_ = tiltPitch_ = tiltRoll_ = 0.0;
    surgeVel_ = surgePos_ = swayVel_ = swayPos_ = 0.0;
    rollRateLp_ = pitchRateLp_ = yawRateLp_ = 0.0;
    rollAngle_ = pitchAngle_ = yawAngle_ = 0.0;
    for (int i = 0; i < 6; ++i) { sm1_[i] = 0.0; sm2_[i] = 0.0; }
    trace_ = WashoutTrace{};
}

Pose WashoutFilter::update(const MotionCues& c, double dt) {
    if (dt <= 0.0) dt = 1.0 / 60.0;

    // --- Heave: HP(accel) -> leaky double-integrate -> mm ---
    const double aZ = cfg_.heaveGain * (static_cast<double>(c.heaveG) - 1.0) * G;
    heaveAccelLp_ += lpAlpha(dt, cfg_.heaveHpTau) * (aZ - heaveAccelLp_);
    const double aHp = aZ - heaveAccelLp_;
    heaveVel_ = heaveVel_ * leak(dt, cfg_.heaveVelWashoutTau) + aHp * dt;
    heavePos_ = heavePos_ * leak(dt, cfg_.heavePosWashoutTau) + heaveVel_ * dt * 1000.0;
    trace_.heaveAHp    = aHp;
    trace_.heaveVel    = heaveVel_;
    trace_.heavePosRaw = heavePos_;
    const double heaveLimited = clampd(heavePos_, -cfg_.heaveLimitMm, cfg_.heaveLimitMm);
    trace_.heaveClamped = (heaveLimited != heavePos_);
    heavePos_ = heaveLimited;

    // --- Horizontal specific force: one low-pass, two consumers ---
    // Tilt coordination takes the low-passed (sustained) part; the onset
    // channel takes the complement. The gains sit outside the filter so both
    // can be scaled independently without the split stopping being a split.
    const double aRawX = static_cast<double>(c.surgeG) * G;
    const double aRawY = static_cast<double>(c.swayG)  * G;
    surgeLpRaw_ += lpAlpha(dt, cfg_.tiltLpTau) * (aRawX - surgeLpRaw_);
    swayLpRaw_  += lpAlpha(dt, cfg_.tiltLpTau) * (aRawY - swayLpRaw_);

    double tgtPitch = std::asin(clampd(cfg_.tiltSurgeGain * surgeLpRaw_ / G, -1.0, 1.0)) * kRad2Deg;
    double tgtRoll  = std::asin(clampd(cfg_.tiltSwayGain  * swayLpRaw_  / G, -1.0, 1.0)) * kRad2Deg;
    tgtPitch = clampd(tgtPitch, -cfg_.tiltLimitDeg, cfg_.tiltLimitDeg);
    tgtRoll  = clampd(tgtRoll,  -cfg_.tiltLimitDeg, cfg_.tiltLimitDeg);
    bool tiltLimited = false;
    tiltPitch_ = rateLimit(tiltPitch_, tgtPitch, cfg_.tiltRateLimitDps, dt, tiltLimited);
    tiltRoll_  = rateLimit(tiltRoll_,  tgtRoll,  cfg_.tiltRateLimitDps, dt, tiltLimited);
    trace_.tiltPitch      = tiltPitch_;
    trace_.tiltRoll       = tiltRoll_;
    trace_.tiltRateActive = tiltLimited;

    // --- Translational onset: HP(accel) -> leaky double-integrate -> mm ---
    // Same shape as the heave channel, including the clamp writing back into
    // the integrator state (windup with no anti-windup -- consistent with
    // heave, deliberately).
    auto transChan = [&](double gain, double aLp, double aRaw, double limitMm,
                         double& vel, double& pos,
                         double& aHpOut, double& velOut, double& rawOut, bool& clampedOut) {
        const double aHp = gain * (aRaw - aLp);
        vel = vel * leak(dt, cfg_.transVelWashoutTau) + aHp * dt;
        pos = pos * leak(dt, cfg_.transPosWashoutTau) + vel * dt * 1000.0;
        aHpOut = aHp;
        velOut = vel;
        rawOut = pos;
        const double limited = clampd(pos, -limitMm, limitMm);
        clampedOut = (limited != pos);
        pos = limited;
    };
    transChan(cfg_.surgeGain, surgeLpRaw_, aRawX, cfg_.surgeLimitMm,
              surgeVel_, surgePos_,
              trace_.surgeAHp, trace_.surgeVel, trace_.surgePosRaw, trace_.surgeClamped);
    transChan(cfg_.swayGain, swayLpRaw_, aRawY, cfg_.swayLimitMm,
              swayVel_, swayPos_,
              trace_.swayAHp, trace_.swayVel, trace_.swayPosRaw, trace_.swayClamped);

    // --- Rotational: HP(rate) -> integrate -> washout leak ---
    auto rotChan = [&](double gain, double rate, double& rateLp, double& angle,
                       double& rawOut, bool& clampedOut) {
        const double w = gain * rate;                // deg/s
        rateLp += lpAlpha(dt, cfg_.rotHpTau) * (w - rateLp);
        const double wHp = w - rateLp;
        angle = (angle + wHp * dt) * leak(dt, cfg_.rotWashoutTau);
        rawOut = angle;
        const double limited = clampd(angle, -cfg_.rotLimitDeg, cfg_.rotLimitDeg);
        clampedOut = (limited != angle);
        angle = limited;
    };
    rotChan(cfg_.rotRollGain,  c.rollRate,  rollRateLp_,  rollAngle_,
            trace_.rotRollRaw,  trace_.rotRollClamped);
    rotChan(cfg_.rotPitchGain, c.pitchRate, pitchRateLp_, pitchAngle_,
            trace_.rotPitchRaw, trace_.rotPitchClamped);
    rotChan(cfg_.rotYawGain,   c.yawRate,   yawRateLp_,   yawAngle_,
            trace_.rotYawRaw,   trace_.rotYawClamped);

    // --- Output smoothing: 2nd-order LP removes high-frequency grain the HP
    // channels pass through (turbulence/engine jitter in g/PQR). The actuators
    // then track a jerk-limited trajectory instead of a jittery one.
    double out[6] = { surgePos_,
                      swayPos_,
                      heavePos_,
                      tiltRoll_  + rollAngle_,
                      tiltPitch_ + pitchAngle_,
                      yawAngle_ };
    if (cfg_.smoothTau > 0.0) {
        const double a = lpAlpha(dt, cfg_.smoothTau);
        for (int i = 0; i < 6; ++i) {
            sm1_[i] += a * (out[i] - sm1_[i]);
            sm2_[i] += a * (sm1_[i] - sm2_[i]);
            out[i] = sm2_[i];
        }
    } else {
        // Keep state tracking the raw output so enabling smoothing via a config
        // reload doesn't start from zero (= a pose jump).
        for (int i = 0; i < 6; ++i) { sm1_[i] = out[i]; sm2_[i] = out[i]; }
    }

    Pose p;
    // Sign convention: +X forward, so a forward specific force commands a
    // forward platform translation. Confirmed against the tilt axis by bench
    // jog in Task 8 -- do not flip it on reasoning alone.
    p.surge = static_cast<float>(out[0]);
    p.sway  = static_cast<float>(out[1]);
    p.heave = static_cast<float>(out[2]);
    p.roll  = static_cast<float>(out[3]);
    p.pitch = static_cast<float>(out[4]);
    p.yaw   = static_cast<float>(out[5]);
    return p;
}
