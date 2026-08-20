#ifndef AXIS_MAPPING_H
#define AXIS_MAPPING_H

#include <stdint.h>

// Q6 fixed point throughout: one raw ADC count is 64 units. Working at 64x the
// ADC resolution is what lets the filtered value reach the wire without being
// rounded back to whole counts.
inline int32_t rawToQ6(uint16_t raw)
{
    return (int32_t)raw << 6;
}

// A percentage of a Q6 span, keeping the span's sign so callers can add or
// subtract it to shrink an interval from either direction without a special
// case for inverted wiring.
//
// The percentages themselves live in Configuration.h; this header takes them as
// parameters so it stays free of Arduino dependencies and testable natively.
//
// No overflow risk at any real calibration: a full-scale span is 1023 << 6 =
// 65472, and 65472 * 100 still fits an int32_t with three decimal digits spare.
inline int32_t deadbandOfSpanQ6(int32_t spanQ6, uint8_t percent)
{
    return (spanQ6 * (int32_t)percent) / 100;
}

// Linear interpolation of `v` from [fromV, toV] onto 0..1000, clamped at both
// ends. Works with a reversed interval (toV < fromV) so inverted sensor wiring
// needs no special case.
inline int32_t mapToWire(int32_t v, int32_t fromV, int32_t toV)
{
    if (fromV == toV) {
        return 0;
    }
    int32_t out = ((v - fromV) * 1000) / (toV - fromV);
    if (out < 0) {
        out = 0;
    }
    if (out > 1000) {
        out = 1000;
    }
    return out;
}

// Maps one half of a travel (from `centerQ6` to `endQ6`) onto 0..1000. All
// offsets are derived from the signed (end - center) delta, so an inverted
// sensor — where the raw value falls as the pedal is pushed — works without
// special casing.
// `centerDeadbandQ6` is an absolute magnitude, already sized by the caller;
// `endDeadbandPercent` is a percentage of this half's own span.
inline uint16_t mapHalfQ6(int32_t vQ6, int32_t centerQ6, int32_t endQ6,
                          int32_t centerDeadbandQ6, uint8_t endDeadbandPercent)
{
    const int32_t span = endQ6 - centerQ6;
    if (span == 0) {
        return 0;
    }
    const int32_t from = centerQ6 + ((span > 0) ? centerDeadbandQ6 : -centerDeadbandQ6);
    const int32_t to   = endQ6 - deadbandOfSpanQ6(span, endDeadbandPercent);

    // If the two deadbands together are as wide as (or wider than) the half-span,
    // `from` has crossed past `to` and the usable interval has collapsed or
    // inverted. Report 0 rather than let mapToWire divide by a delta of the
    // opposite sign, which would reverse the axis over its whole travel. With
    // both deadbands proportional this only happens if the two percentages in
    // Configuration.h sum to 100 or more, so it is a misconfiguration guard.
    if ((to - from) * ((span > 0) ? 1 : -1) <= 0) {
        return 0;
    }
    return (uint16_t)mapToWire(vQ6, from, to);
}

// Maps a unidirectional axis (brake) between two calibration points onto 0..1000.
// `endDeadbandPercent` is applied at both ends of the travel.
inline uint16_t mapUnipolarQ6(int32_t vQ6, int32_t loQ6, int32_t hiQ6, uint8_t endDeadbandPercent)
{
    const int32_t span = hiQ6 - loQ6;
    if (span == 0) {
        return 0;
    }
    const int32_t deadband = deadbandOfSpanQ6(span, endDeadbandPercent);
    return (uint16_t)mapToWire(vQ6, loQ6 + deadband, hiQ6 - deadband);
}

// Full bipolar rudder axis: -1000..1000, left negative.
//
// The centre deadband is a percentage of the half travel being entered, not of
// the whole sweep, so an asymmetric calibration keeps the same proportional feel
// on both sides.
inline int16_t mapRudderQ6(int32_t vQ6, int32_t minQ6, int32_t centerQ6, int32_t maxQ6,
                           uint8_t centerDeadbandPercent, uint8_t endDeadbandPercent)
{
    // Which half of the travel are we on? Decided against the signed direction
    // of the max endpoint, so it stays correct for inverted wiring too.
    const bool towardsMax = (maxQ6 > centerQ6) ? (vQ6 > centerQ6) : (vQ6 < centerQ6);
    const int32_t endQ6   = towardsMax ? maxQ6 : minQ6;

    // Magnitude, because it is tested symmetrically either side of centre below.
    int32_t centerDeadbandQ6 = deadbandOfSpanQ6(endQ6 - centerQ6, centerDeadbandPercent);
    if (centerDeadbandQ6 < 0) {
        centerDeadbandQ6 = -centerDeadbandQ6;
    }

    const int32_t d = vQ6 - centerQ6;
    if (d <= centerDeadbandQ6 && -d <= centerDeadbandQ6) {
        return 0;
    }

    if (towardsMax) {
        return (int16_t)mapHalfQ6(vQ6, centerQ6, maxQ6, centerDeadbandQ6, endDeadbandPercent);
    }
    return (int16_t)-(int16_t)mapHalfQ6(vQ6, centerQ6, minQ6, centerDeadbandQ6, endDeadbandPercent);
}

#endif // AXIS_MAPPING_H
