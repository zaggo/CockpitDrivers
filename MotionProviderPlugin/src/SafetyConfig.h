#pragma once

// Per-setpoint rate limits (16-bit demand counts, 0..65280 full scale) plus
// the arm/disarm soft-start behaviour.
struct SafetyConfig {
    double maxVelocity     = 30000.0;   // counts / second
    double maxAcceleration = 120000.0;  // counts / second^2

    // Arm/disarm pose-space ramp. Disarmed holds a low, level "park" pose
    // (the lowest reachable pose along parkHeaveMm); ARM blends park->live over
    // armRampSec, DISARM blends live->park over disarmRampSec. Ramping the POSE
    // (IK each step) keeps every intermediate a valid rigid platform config.
    double parkHeaveMm    = -500.0;     // target park heave; clamped to reachable
    double armRampSec     = 3.0;
    double disarmRampSec  = 2.0;

    static SafetyConfig defaults() { return SafetyConfig{}; }
};
