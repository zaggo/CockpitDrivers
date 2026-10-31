#pragma once

// Differential toe-brake equalizer. Pure math, no X-Plane dependency, so it
// is unit-tested natively (tests/test_brakeequalizer.cpp).
//
// Problem: resting a foot on the pedals often drags one toe brake slightly
// more than the other, which the sim reads as intentional asymmetric braking.
// apply() compares the two (already curve-shaped) brake values; while their
// difference stays inside diffDeadZone both sides output their average. Past
// the dead zone the differential component fades in linearly from zero —
// rescaled like the AxisCurve dead zones, so there is no jump at the edge and
// full differential braking is still reached at full pedal separation.
struct BrakeEqualizer {
    float diffDeadZone = 0.f;   // 0..1, |left-right| below this -> equalized

    // Clamp diffDeadZone to 0..1; NaN -> 0. Call after loading from config.
    void clampToRange();

    // left/right in 0..1 (anything else is clamped, NaN -> 0), modified in place.
    void apply(float& left, float& right) const;
};
