#ifndef HSICALIBRATION_H
#define HSICALIBRATION_H

#include <stdint.h>

// Calibration maths for HSICAN: needle zero offsets, servo linkage travel and
// the deviation-dot conversion. Deliberately Arduino-free so all of it can be
// exercised by the native Unity tests in test/test_calibration
// (pio test -e native).

// --- needle zero adjust -----------------------------------------------------

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

// After homing, an axis sits at position 0 with the stored offset already
// applied. Jogging it by `jogDegrees` onto the true mark therefore means the new
// offset is the sum of the two - replacing instead of adding throws away every
// previous pass and makes the needle wander a little further each time.
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

// --- servo travel -----------------------------------------------------------

// Clamp to the mechanical travel, scale by the linkage ratio, then add the
// mounting offset. That order is load-bearing: clamping after scaling would let
// a ratio of 2 drive twice the intended travel.
//
// ratio1000 and adjustDeg100 are the EEPROM-stored fixed-point forms of the
// per-servo linkage ratio and mounting offset.
inline double servoTargetDegree(double input, double minDeg, double maxDeg,
                                int16_t ratio1000, int16_t adjustDeg100)
{
    double clamped = input;
    if (clamped < minDeg)
    {
        clamped = minDeg;
    }
    if (clamped > maxDeg)
    {
        clamped = maxDeg;
    }
    return clamped * ((double)ratio1000 / 1000.0) + ((double)adjustDeg100 / 100.0);
}

// --- deviation dots ---------------------------------------------------------

// CAN 0x105 carries deviation as dots * 1000. degPerDot100 is the EEPROM-stored
// conversion; the AirManager LUA used a flat factor of 10 degrees per dot for
// both needles, which is the shipped default.
inline double dotsToServoDegree(int16_t dots1000, int16_t degPerDot100)
{
    return ((double)dots1000 / 1000.0) * ((double)degPerDot100 / 100.0);
}

#endif // HSICALIBRATION_H
