#ifndef TURNCOORDINATORCALIBRATION_H
#define TURNCOORDINATORCALIBRATION_H

#include <stdint.h>

// Degrees -> needle step mapping for both needles of the turn coordinator,
// described by a table of measured calibration points and linear interpolation
// in between. Deliberately Arduino-free so the mapping can be exercised by the
// native Unity tests in test/test_calibration (pio test -e native).
//
// Same shape as VerticalSpeedCAN's table, because the needles share its two
// quirks:
//
//  * Values are signed. Points are kept sorted ascending, so the leftmost
//    deflection sits at index 0 and the rightmost at count-1.
//  * 0 is NOT the mechanical home position. Each needle homes against a stop
//    off the scale; the 0 mark sits mid-dial at whatever step count the bench
//    calibration finds. So a wiped table has no points at all and holds the
//    needle on the home stop until the first point is taught.
//
// The turn rate needle uses degrees of the aircraft symbol (points around
// +-90), the slip ball degrees of slip (points at -8/0/8). One table each,
// both of this type. Nothing here assumes symmetry about 0 - the slip dial
// is not symmetric, and the table simply records what the bench measured.

static const uint8_t kMaxCalibrationPoints = 12;

struct CalibrationPoint
{
    int16_t value;
    uint16_t step;
};

struct CalibrationTable
{
    uint8_t count;
    CalibrationPoint points[kMaxCalibrationPoints];
};

// Factory reset: no points at all. The needle parks on the mechanical home
// stop (step 0) until the bench teaches at least one point.
inline void calibrationWipe(CalibrationTable &table)
{
    table.count = 0;
}

// Adds or updates the point for `value`, keeping the table sorted by value.
// Returns false only when a *new* point does not fit any more - replacing a
// known value always succeeds.
inline bool calibrationSet(CalibrationTable &table, int16_t value, uint16_t step)
{
    uint8_t index = 0;
    while (index < table.count && table.points[index].value < value)
    {
        index++;
    }

    if (index < table.count && table.points[index].value == value)
    {
        table.points[index].step = step;
        return true;
    }

    if (table.count >= kMaxCalibrationPoints)
    {
        return false;
    }

    for (uint8_t i = table.count; i > index; i--)
    {
        table.points[i] = table.points[i - 1];
    }
    table.points[index].value = value;
    table.points[index].step = step;
    table.count++;
    return true;
}

// The leftmost calibrated deflection, i.e. the bottom of the displayable range.
inline int16_t calibrationMinValue(const CalibrationTable &table)
{
    if (table.count == 0)
    {
        return 0;
    }
    return table.points[0].value;
}

// The rightmost calibrated deflection, i.e. the top of the displayable range.
inline int16_t calibrationMaxValue(const CalibrationTable &table)
{
    if (table.count == 0)
    {
        return 0;
    }
    return table.points[table.count - 1].value;
}

// Needle position for a value, clamped to the lowest and highest calibrated
// point - the dial simply has no marks beyond either end.
inline uint16_t calibrationStepFor(const CalibrationTable &table, float value)
{
    if (table.count == 0)
    {
        return 0;
    }
    if (value <= (float)table.points[0].value)
    {
        return table.points[0].step;
    }

    const uint8_t last = table.count - 1;
    if (value >= (float)table.points[last].value)
    {
        return table.points[last].step;
    }

    uint8_t upper = 1;
    while (upper < last && (float)table.points[upper].value < value)
    {
        upper++;
    }

    const CalibrationPoint &low = table.points[upper - 1];
    const CalibrationPoint &high = table.points[upper];
    const float span = (float)high.value - (float)low.value;
    const float ratio = (value - (float)low.value) / span;
    const float step = (float)low.step + ratio * ((float)high.step - (float)low.step);
    return (uint16_t)(step + 0.5f);
}

#endif // TURNCOORDINATORCALIBRATION_H
