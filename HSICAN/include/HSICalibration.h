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
    // Round, don't truncate: moveToDegree already truncates the target to
    // whole steps, so a 2 degree jog lands on 22 steps = 1.93 degrees, and
    // truncating that again stored only 1.
    const int32_t scaled = steps * 360;
    return (scaled >= 0 ? scaled + total / 2 : scaled - total / 2) / total;
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

// --- GS servo table ---------------------------------------------------------
//
// The two GS servos are not driven through servoTargetDegree: each has a
// taught table of (dots, percent) points, percent being the absolute servo
// deflection around the 50 % centre (-100 = 0 deg, 0 = 90 deg, 100 = 180 deg).
// Sign, linkage ratio and non-linearity of the mechanism all live in the
// taught points. Points are kept sorted by dots.

struct GsPoint
{
    int16_t dots1000;
    int8_t percent;
};

const uint8_t kGsPointCount = 9;

// GS "not available" park target, in dots x 1000. interpolateGsPercent clamps
// outside the taught table at the outermost point, so a value beyond every
// realistic reading parks the needle at the end of the valid range instead of
// the scale centre - which reads as a valid on-glidepath indication and hid
// the missing signal on test flights. Positive parks at the high-dots end;
// which physical way that is lives in the taught tables, so if the needle
// parks at the wrong end on the rig, flip the sign rather than re-teaching.
const int16_t kGsParkDots1000 = INT16_MAX;

inline double gsPercentToServoDegree(double percent)
{
    return 90.0 + percent * 0.9;
}

// Linear between neighbours; outside the table the outermost point holds, so
// the taught extremes double as soft endstops. No points centres the servo.
inline double interpolateGsPercent(const GsPoint* points, uint8_t count, int16_t dots1000)
{
    if (count == 0)
    {
        return 0.0;
    }
    if (dots1000 <= points[0].dots1000)
    {
        return points[0].percent;
    }
    if (dots1000 >= points[count - 1].dots1000)
    {
        return points[count - 1].percent;
    }
    uint8_t i = 1;
    while (points[i].dots1000 < dots1000)
    {
        i++;
    }
    const GsPoint& a = points[i - 1];
    const GsPoint& b = points[i];
    const double t = (double)(dots1000 - a.dots1000) / (double)(b.dots1000 - a.dots1000);
    return a.percent + (b.percent - a.percent) * t;
}

// Sorted insert. The same dots value re-teaches the existing point; a full
// table refuses new ones.
inline bool insertGsPoint(GsPoint* points, uint8_t* count, uint8_t capacity,
                          int16_t dots1000, int8_t percent)
{
    uint8_t i = 0;
    while (i < *count && points[i].dots1000 < dots1000)
    {
        i++;
    }
    if (i < *count && points[i].dots1000 == dots1000)
    {
        points[i].percent = percent;
        return true;
    }
    if (*count >= capacity)
    {
        return false;
    }
    for (uint8_t j = *count; j > i; j--)
    {
        points[j] = points[j - 1];
    }
    points[i].dots1000 = dots1000;
    points[i].percent = percent;
    (*count)++;
    return true;
}

#endif // HSICALIBRATION_H
