#ifndef HSIGEOMETRY_H
#define HSIGEOMETRY_H

#include <stdint.h>

// Axis geometry and homing maths for HSICAN. Deliberately Arduino-free so both
// can be exercised by the native Unity tests in test/test_geometry
// (pio test -e native).

// --- axis targets -----------------------------------------------------------

// Motor angles for the three stepper axes, in hundredths of a degree, 0..35999.
struct AxisTargets
{
    uint16_t compassAxis100;
    uint16_t obsAxis100;
    uint16_t hdgAxis100;
};

// The course pointer and the heading bug sit on the rotating compass card, so
// their axes carry the pointer's offset from the current heading rather than an
// absolute angle. Only the card itself is mirrored.
//
// The old AirManager LUA pre-mirrored all three values before sending them
// (-h, 360-o, 360-b) and the firmware then differenced them. Substituting those
// in, the mirroring cancels on the two pointers and only the card keeps it:
//
//     compass = (-h)    mod 360
//     obs     = (o - h) mod 360
//     hdg     = (b - h) mod 360
//
// Unrelated to CheapStepper's `inversed` flag, which reverses the coil sequence
// rather than the target angle. Both are active on the rig.
inline AxisTargets axisTargets(uint16_t heading100, uint16_t obs100, uint16_t hdgBug100)
{
    const int32_t full = 36000;
    const int32_t h = (int32_t)heading100;

    AxisTargets t;
    t.compassAxis100 = (uint16_t)(((full - h) % full + full) % full);
    t.obsAxis100 = (uint16_t)((((int32_t)obs100 - h) % full + full) % full);
    t.hdgAxis100 = (uint16_t)((((int32_t)hdgBug100 - h) % full + full) % full);
    return t;
}

// --- homing window centre ---------------------------------------------------

// Half the hall window, measured from its far edge back to its near edge.
//
// The two edges are captured in different homing phases with a resetPosition()
// between them, so the difference is naturally unsigned - and that is the trap.
// If `zeroEnd` has wrapped below `zeroStart` the difference comes back as
// almost a whole turn and halves into a silent 180 degree mis-home that still
// reports success. WhiskeyCompassCAN was bitten by exactly this, twice.
//
// A hall window spans a few degrees, so anything wider than an eighth of a turn
// is a misread rather than a window. Returns false and leaves *centreSteps
// untouched in that case; the caller turns that into a homing timeout.
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

#endif // HSIGEOMETRY_H
