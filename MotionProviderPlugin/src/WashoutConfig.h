#pragma once

// Pragmatic classical washout parameters. Times in seconds, gains dimensionless
// unless noted. Defaults are conservative starting points for tuning.
struct WashoutConfig {
    // Heave (vertical translation), driven by (g_nrml - 1)
    double heaveGain          = 0.5;
    double heaveHpTau         = 1.0;   // high-pass on accel
    double heaveVelWashoutTau = 2.0;   // leaky velocity integrator
    double heavePosWashoutTau = 2.0;   // leaky position integrator
    double heaveLimitMm       = 30.0;

    // Tilt-coordination: sustained surge/sway specific force -> pitch/roll
    double tiltSurgeGain   = 1.0;      // g_axil -> pitch
    double tiltSwayGain    = 1.0;      // g_side -> roll
    double tiltLpTau       = 1.5;      // low-pass to extract sustained accel
    double tiltLimitDeg    = 3.0;      // tilt + rotational sum on one axis stays in envelope
    double tiltRateLimitDps = 5.0;     // max tilt-coordination rate

    // Translational onset cue: the complement of the tilt low-pass, leaky
    // double-integrated to mm. Renders the first fraction of a second of a
    // longitudinal/lateral acceleration, which tilt coordination cannot --
    // it is low-passed and rate-limited by design. The crossover constant is
    // tiltLpTau above, shared by both halves, so LP + HP = 1 and the same
    // acceleration is never counted twice.
    // Gains ship at 0: the channel is off until a rig verdict adopts values.
    double surgeGain           = 0.0;
    double swayGain            = 0.0;
    double transVelWashoutTau  = 0.25;   // shared by surge and sway
    double transPosWashoutTau  = 0.25;
    double surgeLimitMm        = 43.0;   // Task 1 measurement
    double swayLimitMm         = 41.0;   // Task 1 measurement

    // Rotational: angular rate -> angle (high-pass + washout)
    double rotRollGain   = 0.7;
    double rotPitchGain  = 0.7;
    double rotYawGain    = 0.7;
    double rotHpTau      = 1.0;        // high-pass on rate
    double rotWashoutTau = 3.0;        // slow angle recentering
    double rotLimitDeg   = 3.0;        // tilt + rotational sum on one axis stays in envelope

    // Output smoothing: 2nd-order (two cascaded one-pole) low-pass on the final
    // washout pose. Removes high-frequency grain (turbulence/engine jitter in the
    // g/PQR datarefs) that the heave/rot channels otherwise pass straight to the
    // actuators. 0 = off. Cost: ~2*tau of added cue latency.
    double smoothTau = 0.0;

    static WashoutConfig defaults() { return WashoutConfig{}; }
};
