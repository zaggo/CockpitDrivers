#ifndef VERTICALSPEEDCALIBRATION_H
#define VERTICALSPEEDCALIBRATION_H

#include <stdint.h>

// The dial's tick spacing is not linear in feet per minute, so the needle
// position is described by a table of measured calibration points and linear
// interpolation in between. Deliberately Arduino-free so the mapping can be
// exercised by the native Unity tests in test/test_calibration
// (pio test -e native).
//
// Two things differ from the ASI's otherwise identical table:
//
//  * Climb rates are signed. Points are kept sorted ascending, so the deepest
//    descent sits at index 0 and the steepest climb at count-1.
//  * 0 ft/min is NOT the mechanical home position. The needle homes against a
//    stop somewhere off the scale; the 0 mark sits mid-dial at whatever step
//    count the bench calibration finds. So a wiped table has no points at all
//    and holds the needle on the home stop until the first point is taught -
//    there is nothing sensible to anchor it to beforehand.

static const uint8_t kMaxCalibrationPoints = 12;

struct CalibrationPoint
{
    int16_t fpm;
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

// Adds or updates the point for `fpm`, keeping the table sorted by fpm.
// Returns false only when a *new* point does not fit any more - replacing a
// known fpm value always succeeds.
inline bool calibrationSet(CalibrationTable &table, int16_t fpm, uint16_t step)
{
    uint8_t index = 0;
    while (index < table.count && table.points[index].fpm < fpm)
    {
        index++;
    }

    if (index < table.count && table.points[index].fpm == fpm)
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
    table.points[index].fpm = fpm;
    table.points[index].step = step;
    table.count++;
    return true;
}

// The deepest calibrated descent, i.e. the bottom of the displayable range.
inline int16_t calibrationMinFpm(const CalibrationTable &table)
{
    if (table.count == 0)
    {
        return 0;
    }
    return table.points[0].fpm;
}

// The steepest calibrated climb, i.e. the top of the displayable range.
inline int16_t calibrationMaxFpm(const CalibrationTable &table)
{
    if (table.count == 0)
    {
        return 0;
    }
    return table.points[table.count - 1].fpm;
}

// Needle position for a climb rate, clamped to the lowest and highest
// calibrated point - the dial simply has no marks beyond either end.
inline uint16_t calibrationStepFor(const CalibrationTable &table, float fpm)
{
    if (table.count == 0)
    {
        return 0;
    }
    if (fpm <= (float)table.points[0].fpm)
    {
        return table.points[0].step;
    }

    const uint8_t last = table.count - 1;
    if (fpm >= (float)table.points[last].fpm)
    {
        return table.points[last].step;
    }

    uint8_t upper = 1;
    while (upper < last && (float)table.points[upper].fpm < fpm)
    {
        upper++;
    }

    const CalibrationPoint &low = table.points[upper - 1];
    const CalibrationPoint &high = table.points[upper];
    const float span = (float)high.fpm - (float)low.fpm;
    const float ratio = (fpm - (float)low.fpm) / span;
    const float step = (float)low.step + ratio * ((float)high.step - (float)low.step);
    return (uint16_t)(step + 0.5f);
}

#endif // VERTICALSPEEDCALIBRATION_H
