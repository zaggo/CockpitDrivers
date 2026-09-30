#ifndef BACKLASH_H
#define BACKLASH_H

#include <stdint.h>

// Gear backlash bookkeeping for HSICAN's three geared stepper axes.
// Deliberately Arduino-free so the partial-reversal case - the one that is easy
// to get wrong and invisible anywhere but the rig - is covered by the native
// Unity tests in test/test_backlash (pio test -e native).
//
// The model: `unused` counts slack steps still to be crossed before the output
// moves again. While it is nonzero the motor turns and the output does not,
// which is what makes the tracked position an *output* position rather than a
// motor position - and that in turn is what makes shortest-path routing and the
// homing captures correct.

// Slack still to cross after a direction change. Whatever was already travelled
// in the old direction (total - unused) is exactly what must be travelled back.
inline uint32_t backlashOnReversal(uint32_t unused, uint32_t total)
{
    if (unused >= total)
    {
        return 0;
    }
    return total - unused;
}

// Accounts one motor step. Returns true if the step was swallowed by the slack,
// false if it should advance the tracked output position.
inline bool consumeBacklashStep(uint32_t *unused)
{
    if (unused == 0 || *unused == 0)
    {
        return false;
    }
    (*unused)--;
    return true;
}

// Whether a computed move is worth issuing.
//
// 0x104 arrives at 50 Hz carrying hundredths of a degree, and at 11.38 steps per
// degree the integer step target jitters by +-1 continuously. Every one of those
// flips would append a whole slack crossing - up to 75 steps on the heading bug -
// so a pointer that should be standing still would instead be driven back and
// forth through the gear play: audible, wearing, and never settling.
//
// A reversal therefore has to clear a larger threshold than a continuation,
// because only a reversal pays for the slack. A reversalDeadband of 0 turns the
// behaviour off.
inline bool moveIsWorthMaking(int32_t steps, bool lastDirectionClockwise,
                              uint32_t moveDeadband, uint32_t reversalDeadband)
{
    if (steps == 0)
    {
        return false;
    }

    const uint32_t magnitude = (uint32_t)(steps < 0 ? -steps : steps);
    const bool clockwise = steps > 0;
    const uint32_t limit = (clockwise != lastDirectionClockwise) ? reversalDeadband
                                                                 : moveDeadband;
    return magnitude >= limit;
}

#endif // BACKLASH_H
