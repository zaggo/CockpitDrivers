#include "SafetyLimiter.h"
#include <cmath>

namespace {
double clampd(double v, double lo, double hi){ return v<lo?lo:(v>hi?hi:v); }
}

void SafetyLimiter::reset(const uint16_t initial[6]) {
    for (int i = 0; i < 6; ++i) { pos_[i] = initial[i]; vel_[i] = 0.0; }
    init_ = true;
}

void SafetyLimiter::limit(const uint16_t desired[6], double dt, uint16_t out[6]) {
    if (dt <= 0.0) dt = 1.0 / 60.0;
    if (!init_) { reset(desired); }  // first call: start at the desired position

    const double vMax = cfg_.maxVelocity;
    const double dvMax = cfg_.maxAcceleration * dt;

    for (int i = 0; i < 6; ++i) {
        const double target = static_cast<double>(desired[i]);
        // Velocity needed to reach target in one step, capped to vMax.
        double desiredVel = clampd((target - pos_[i]) / dt, -vMax, vMax);
        // Acceleration-limit the change in velocity.
        double dv = clampd(desiredVel - vel_[i], -dvMax, dvMax);
        vel_[i] += dv;
        pos_[i] += vel_[i] * dt;
        pos_[i] = clampd(pos_[i], 0.0, 65280.0);
        out[i] = static_cast<uint16_t>(std::lround(pos_[i]));
    }
}
