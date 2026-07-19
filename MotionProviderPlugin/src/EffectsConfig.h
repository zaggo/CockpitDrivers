#pragma once

// Additive motion effects. Amplitudes in mm (heave) / deg (jitter), freqs in Hz.
struct EffectsConfig {
    // Gear touchdown bump (decaying sine on heave, triggered on landing)
    double touchdownGain    = 8.0;   // mm peak
    double touchdownFreqHz  = 6.0;
    double touchdownDecayTau = 0.25; // s

    // Ground-roll rumble (speed-scaled sine while rolling)
    double rumbleGain        = 2.0;  // mm at reference speed
    double rumbleFreqHz      = 12.0;
    double rumbleSpeedRefMps = 40.0; // full amplitude at/above this groundspeed

    // Reserved (wired in a later phase), no output while gain == 0
    double engineGain = 0.0;
    double buffetGain = 0.0;

    static EffectsConfig defaults() { return EffectsConfig{}; }
};
