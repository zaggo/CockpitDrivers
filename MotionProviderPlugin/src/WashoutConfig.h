#pragma once

// Pragmatic classical washout parameters. Times in seconds, gains dimensionless
// unless noted. Defaults are conservative starting points for tuning.
struct WashoutConfig {
    // Heave (vertical translation), driven by (g_nrml - 1)
    double heaveGain          = 0.5;
    double heaveHpTau         = 1.0;   // high-pass on accel
    double heaveVelWashoutTau = 2.0;   // leaky velocity integrator
    double heavePosWashoutTau = 2.0;   // leaky position integrator
    double heaveLimitMm       = 40.0;

    // Tilt-coordination: sustained surge/sway specific force -> pitch/roll
    double tiltSurgeGain   = 1.0;      // g_axil -> pitch
    double tiltSwayGain    = 1.0;      // g_side -> roll
    double tiltLpTau       = 1.5;      // low-pass to extract sustained accel
    double tiltLimitDeg    = 6.0;
    double tiltRateLimitDps = 5.0;     // max tilt-coordination rate

    // Rotational: angular rate -> angle (high-pass + washout)
    double rotRollGain   = 0.7;
    double rotPitchGain  = 0.7;
    double rotYawGain    = 0.7;
    double rotHpTau      = 1.0;        // high-pass on rate
    double rotWashoutTau = 3.0;        // slow angle recentering
    double rotLimitDeg   = 6.0;

    static WashoutConfig defaults() { return WashoutConfig{}; }
};
