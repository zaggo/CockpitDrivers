#pragma once

// MSFS-style response curve for one joystick axis. Pure math, no X-Plane
// dependency, so it is unit-tested natively (tests/test_axiscurve.cpp).
//
// apply() runs: clamp -> neutral shift -> dead zone -> extremity dead zone ->
// sensitivity power curve -> clamp. Every stage is continuous and monotonic.
struct AxisCurve {
    float sensPos  = 0.f;   // -1..1, positive half: -1 flat centre (x^3), +1 steep centre (cbrt)
    float sensNeg  = 0.f;   // -1..1, negative half
    float neutral  = 0.f;   // -1..1, raw value that maps to output 0
    float deadZone = 0.f;   //  0..1, |x| below this (after neutral) -> 0
    float extremityDeadZone = 0.f; // 0..1, |x| above 1-this -> 1

    // in -1..1 (anything else is clamped, NaN -> 0) -> out -1..1
    float apply(float in) const;

    // Clamp every parameter to its range; NaN -> 0. Call after loading from config.
    void clampToRange();
};
