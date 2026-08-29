#ifndef AIRSPEEDCALIBRATION_H
#define AIRSPEEDCALIBRATION_H

#include <stdint.h>

// The dial's tick spacing is not linear in knots, so the needle position is
// described by a table of measured calibration points and linear interpolation
// in between. Deliberately Arduino-free so the mapping can be exercised by the
// native Unity tests in test/test_calibration (pio test -e native).

static const uint8_t kMaxCalibrationPoints = 12;

struct CalibrationPoint
{
    uint16_t knots;
    uint16_t step;
};

struct CalibrationTable
{
    uint8_t count;
    CalibrationPoint points[kMaxCalibrationPoints];
};

// Factory reset: only the 0kt anchor survives, sitting on the mechanical home
// position the needle reaches after zeroing. If the dial's 0 mark is offset
// from that stop, recalibrate the anchor with calibrationSet(table, 0, step).
inline void calibrationWipe(CalibrationTable &table)
{
    table.count = 1;
    table.points[0].knots = 0;
    table.points[0].step = 0;
}

// Adds or updates the point for `knots`, keeping the table sorted by knots.
// Returns false only when a *new* point does not fit any more - replacing a
// known knots value always succeeds.
inline bool calibrationSet(CalibrationTable &table, uint16_t knots, uint16_t step)
{
    uint8_t index = 0;
    while (index < table.count && table.points[index].knots < knots)
    {
        index++;
    }

    if (index < table.count && table.points[index].knots == knots)
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
    table.points[index].knots = knots;
    table.points[index].step = step;
    table.count++;
    return true;
}

// The highest calibrated speed, i.e. the top of the displayable range.
inline uint16_t calibrationMaxKnots(const CalibrationTable &table)
{
    if (table.count == 0)
    {
        return 0;
    }
    return table.points[table.count - 1].knots;
}

// Needle position for an airspeed, clamped to the anchor below and to the
// highest calibrated point above - the dial simply has no marks past it.
inline uint16_t calibrationStepFor(const CalibrationTable &table, float knots)
{
    if (table.count == 0)
    {
        return 0;
    }
    if (knots <= (float)table.points[0].knots)
    {
        return table.points[0].step;
    }

    const uint8_t last = table.count - 1;
    if (knots >= (float)table.points[last].knots)
    {
        return table.points[last].step;
    }

    uint8_t upper = 1;
    while (upper < last && (float)table.points[upper].knots < knots)
    {
        upper++;
    }

    const CalibrationPoint &low = table.points[upper - 1];
    const CalibrationPoint &high = table.points[upper];
    const float span = (float)high.knots - (float)low.knots;
    const float ratio = (knots - (float)low.knots) / span;
    const float step = (float)low.step + ratio * ((float)high.step - (float)low.step);
    return (uint16_t)(step + 0.5f);
}

#endif // AIRSPEEDCALIBRATION_H
