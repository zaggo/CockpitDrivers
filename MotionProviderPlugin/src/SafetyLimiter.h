#pragma once
#include <cstdint>
#include "SafetyConfig.h"

// Velocity- and acceleration-limits each of the six setpoints so commanded
// motion never jumps. Stateful; dt-driven; no X-Plane deps.
class SafetyLimiter {
public:
    explicit SafetyLimiter(const SafetyConfig& cfg) : cfg_(cfg) {}

    // Rate-limit toward the desired setpoints; writes the limited result to out.
    void limit(const uint16_t desired[6], double dt, uint16_t out[6]);

    // Snap internal state to a known position (e.g. home) with zero velocity.
    void reset(const uint16_t initial[6]);

    void setConfig(const SafetyConfig& cfg) { cfg_ = cfg; }

private:
    SafetyConfig cfg_;
    double pos_[6] = {0,0,0,0,0,0};
    double vel_[6] = {0,0,0,0,0,0};
    bool   init_ = false;
};
