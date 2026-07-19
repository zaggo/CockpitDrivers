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

    // Runaway / watchdog (Phase 5). A command DOF beyond these hard sanity
    // bounds, sustained for runawayHoldSec, latches a Runaway fault. maxDtSec
    // clamps the timestep fed to the filters so an X-Plane stall can't diverge
    // the washout.
    double runawayTiltDeg = 45.0;   // |roll|/|pitch|/|yaw| sanity bound (deg)
    double runawayTransMm = 500.0;  // |surge|/|sway|/|heave| sanity bound (mm)
    double runawayHoldSec = 1.0;    // sustained out-of-bounds -> fault
    double maxDtSec       = 0.1;    // filter timestep clamp

    static SafetyConfig defaults() { return SafetyConfig{}; }
};
