#ifndef COMPASSGEOMETRY_H
#define COMPASSGEOMETRY_H

#include <math.h>
#include <stdint.h>

// Geometry for the whiskey compass card: heading to step position, the shortest
// way round between two positions, and the Hall-zero-to-painted-N offset.
// Deliberately Arduino-free so all of it can be exercised by the native Unity
// tests in test/test_geometry (pio test -e native).

// --- step positions ---------------------------------------------------------

// Folds any step position, negative included, into 0..totalSteps-1. CheapStepper
// reports an int32_t that goes negative after a counter-clockwise move, so this
// runs on everything that comes out of getPosition().
inline uint32_t normalizeStepPosition(int32_t position, uint32_t totalSteps)
{
    if (totalSteps == 0)
    {
        return 0;
    }
    const int32_t total = (int32_t)totalSteps;
    return (uint32_t)((position % total + total) % total);
}

// Heading in degrees to a normalised step position, rounded to the nearest step.
// Headings outside 0..360 are wrapped first: compass_heading_deg_mag can arrive
// slightly negative or above 360, and an unwrapped value would send the card to
// a position most of a turn away.
inline uint32_t degreesToSteps(double degrees, uint32_t totalSteps)
{
    if (totalSteps == 0)
    {
        return 0;
    }
    double wrapped = fmod(degrees, 360.0);
    if (wrapped < 0.0)
    {
        wrapped += 360.0;
    }
    const int32_t steps = (int32_t)(wrapped * (double)totalSteps / 360.0 + 0.5);
    return normalizeStepPosition(steps, totalSteps);
}

// Signed step delta from `current` to `target`, taking the short way round.
// Positive is clockwise. Without this a card at 359 degrees moving to 1 degree
// would wind 358 degrees backwards in full view of the pilot.
// At exactly half a turn both ways are equal and the tie breaks clockwise.
inline int32_t shortestPathSteps(uint32_t current, uint32_t target, uint32_t totalSteps)
{
    if (totalSteps == 0)
    {
        return 0;
    }
    const uint32_t cur = normalizeStepPosition((int32_t)current, totalSteps);
    const uint32_t tgt = normalizeStepPosition((int32_t)target, totalSteps);
    const uint32_t diffCW = (tgt + totalSteps - cur) % totalSteps;
    const uint32_t diffCCW = (cur + totalSteps - tgt) % totalSteps;
    if (diffCW <= diffCCW)
    {
        return (int32_t)diffCW;
    }
    return -(int32_t)diffCCW;
}

// --- zero adjust ------------------------------------------------------------

// Folds a degree value into -180..179 so repeated calibration passes cannot
// accumulate whole turns.
inline int16_t normalizeZeroAdjust(int32_t degrees)
{
    int32_t wrapped = degrees % 360;
    if (wrapped < -180)
    {
        wrapped += 360;
    }
    else if (wrapped > 179)
    {
        wrapped -= 360;
    }
    return (int16_t)wrapped;
}

// After homing the card sits at position 0 with the stored offset already
// applied, so jogging it onto the painted N mark means the new offset is the sum
// of the two. Replacing instead of adding throws away every previous pass and
// makes the card wander a little further each time.
inline int16_t accumulateZeroAdjust(int16_t stored, int32_t jogDegrees)
{
    return normalizeZeroAdjust((int32_t)stored + jogDegrees);
}

// The stepper reports its position normalised into 0..totalSteps-1, so a small
// counter-clockwise jog comes back as nearly a full turn. Fold it back onto the
// short way round before converting to degrees.
inline int32_t jogDegreesFromPosition(uint32_t position, uint32_t totalSteps)
{
    if (totalSteps == 0)
    {
        return 0;
    }

    const int32_t total = (int32_t)totalSteps;
    int32_t steps = (int32_t)position;
    if (steps > total / 2)
    {
        steps -= total;
    }
    return steps * 360 / total;
}

#endif // COMPASSGEOMETRY_H
