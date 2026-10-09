#ifndef ATTITUDEGEOMETRY_H
#define ATTITUDEGEOMETRY_H

#include <math.h>
#include <stdint.h>

// Geometry for the attitude indicator: two nested 28BYJ-48 axes (the pitch
// motor rides on the roll frame), heading-style shortest paths on both, the
// mechanical pitch window, the Hall-window centre for homing and the EEPROM
// zero offsets. Deliberately Arduino-free so all of it runs under the native
// Unity tests in test/test_geometry (pio test -e native).

// --- step positions ---------------------------------------------------------

// Folds any step position, negative included, into 0..totalSteps-1. The
// vendored CheapStepper reports an int32_t that goes negative after a
// counter-clockwise move, so this runs on everything out of getPosition().
inline uint32_t normalizeStepPosition(int32_t position, uint32_t totalSteps)
{
    if (totalSteps == 0)
    {
        return 0;
    }
    const int32_t total = (int32_t)totalSteps;
    return (uint32_t)((position % total + total) % total);
}

// Signed step delta from `current` to `target`, the short way round. Positive
// is clockwise. At exactly half a turn both ways are equal; the tie breaks CW.
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

// --- homing window ----------------------------------------------------------

// Centre of the Hall window from its two edge captures, as an always-forward
// modular distance from zeroStart to zeroEnd. A Hall window spans a few
// degrees, so anything wider than an eighth of a turn is a misread (typically
// a phase that latched before the axis stepped, leaving one edge at 0 and the
// difference wrapped to most of a revolution — halving THAT would park the
// axis ~180 degrees off and still report success). Returns false and leaves
// *centreSteps untouched in that case.
inline bool homingWindowCentre(uint32_t zeroEnd, uint32_t zeroStart,
                               uint32_t totalSteps, int32_t *centreSteps)
{
    if (totalSteps == 0 || centreSteps == 0)
    {
        return false;
    }
    const uint32_t span = (zeroEnd + totalSteps - (zeroStart % totalSteps)) % totalSteps;
    if (span > totalSteps / 8)
    {
        return false;
    }
    *centreSteps = (int32_t)(span / 2);
    return true;
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

// After homing an axis sits at 0 with the stored offset already applied, so a
// jog onto the true zero means the new offset is the SUM. Replacing instead of
// adding throws away every previous pass.
inline int16_t accumulateZeroAdjust(int16_t stored, int32_t jogDegrees)
{
    return normalizeZeroAdjust((int32_t)stored + jogDegrees);
}

// A normalised position just below a full turn is a small backwards jog, not
// most of a turn forwards. Fold onto the short way round, then to degrees.
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

#endif // ATTITUDEGEOMETRY_H
