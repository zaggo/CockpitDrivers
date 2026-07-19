#pragma once

// Per-setpoint rate limits, in 16-bit demand counts (0..65280 full scale).
struct SafetyConfig {
    double maxVelocity     = 30000.0;   // counts / second
    double maxAcceleration = 120000.0;  // counts / second^2
    static SafetyConfig defaults() { return SafetyConfig{}; }
};
