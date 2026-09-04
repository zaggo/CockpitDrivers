#ifndef ALTIMETERCALIBRATION_H
#define ALTIMETERCALIBRATION_H

#include <stdint.h>

// Calibration maths for AltimeterCAN: the per-axis needle zero offset and the
// barometer pot conversion. Deliberately Arduino-free so both can be exercised
// by the native Unity tests in test/test_calibration (pio test -e native).

// --- Needle zero adjust -----------------------------------------------------

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
// offset is the sum of the two — replacing instead of adding throws away every
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

// --- Barometer pot ----------------------------------------------------------

struct BaroCalibrationPoint
{
    uint16_t raw;      // raw ADC reading
    uint16_t inHg100;  // inches of mercury * 100
};

struct BaroCalibration
{
    BaroCalibrationPoint low;
    BaroCalibrationPoint high;
};

inline void baroCalibrationDefaults(BaroCalibration &table,
                                    uint16_t lowRaw, uint16_t lowInHg100,
                                    uint16_t highRaw, uint16_t highInHg100)
{
    table.low.raw = lowRaw;
    table.low.inHg100 = lowInHg100;
    table.high.raw = highRaw;
    table.high.inHg100 = highInHg100;
}

// Stores one taught endpoint. Kept here rather than inline at the call site so
// the isHigh-to-endpoint wiring is covered by the native tests.
inline void baroCalibrationSet(BaroCalibration &table, bool isHigh, uint16_t raw, uint16_t inHg100)
{
    BaroCalibrationPoint &point = isHigh ? table.high : table.low;
    point.raw = raw;
    point.inHg100 = inHg100;
}

// Linear interpolation between the two taught points, clamped beyond them. The
// pot may be wired either way round, so the direction is taken from the points
// rather than assumed. Two points taught at the same raw value would divide by
// zero and instead park on the low point.
inline uint16_t baroInHg100(const BaroCalibration &table, uint16_t raw)
{
    if (table.high.raw == table.low.raw)
    {
        return table.low.inHg100;
    }

    const float lowRaw = (float)table.low.raw;
    const float highRaw = (float)table.high.raw;
    const float value = (float)raw;
    const bool ascending = highRaw > lowRaw;

    if (ascending ? (value <= lowRaw) : (value >= lowRaw))
    {
        return table.low.inHg100;
    }
    if (ascending ? (value >= highRaw) : (value <= highRaw))
    {
        return table.high.inHg100;
    }

    const float ratio = (value - lowRaw) / (highRaw - lowRaw);
    const float inHg100 = (float)table.low.inHg100 +
                          ratio * ((float)table.high.inHg100 - (float)table.low.inHg100);
    return (uint16_t)(inHg100 + 0.5f);
}

#endif // ALTIMETERCALIBRATION_H
