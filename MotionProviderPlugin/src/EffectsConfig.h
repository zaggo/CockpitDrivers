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

    // Runway slab joints. One step per slab, alternating up and down, triggered
    // by distance travelled rather than by a clock -- so the rate follows
    // groundspeed the way real expansion joints do.
    //
    // A step is cheaper than a bump: returning to zero costs four
    // accelerate/decelerate phases in the same window, a step costs two, and
    // the return trip is the next joint. That is what makes a 1 mm cue
    // renderable here where the 12 Hz rumble was 14x over the acceleration
    // limit. The move is shaped so its PEAK ACCELERATION is the quantity being
    // budgeted (slabAccelMmS2), because acceleration -- not displacement -- is
    // both what the pilot feels and what SafetyLimiter clips.
    //
    // Off by default: spacing 0 disables the effect entirely.
    // The budget is deliberately well under the platform's ~363 mm/s2 ceiling:
    // the washout is spending part of it at the same time, and measured on a
    // real ground roll, 300 pushed limiter engagement from 14 % to 39 % while
    // 200 left it at 15 % and still delivered the full 1 mm step.
    double slabSpacingM    = 0.0;    // m between joints; 0 = off
    double slabStepMm      = 1.0;    // step height per joint
    double slabAccelMmS2   = 200.0;  // peak acceleration budget for one step
    double slabMinSpeedMps = 0.5;    // no joints below this groundspeed

    // Reserved (wired in a later phase), no output while gain == 0
    double engineGain = 0.0;
    double buffetGain = 0.0;

    static EffectsConfig defaults() { return EffectsConfig{}; }
};
