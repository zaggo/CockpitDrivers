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
inline uint16_t mapHalfQ6(int32_t vQ6, int32_t centerQ6, int32_t endQ6, int32_t centerDeadbandQ6)
{
    const int32_t span = endQ6 - centerQ6;
    if (span == 0) {
        return 0;
    }
    const int32_t from = centerQ6 + ((span > 0) ? centerDeadbandQ6 : -centerDeadbandQ6);
    const int32_t to   = endQ6 - span / 20; // 5% end deadband, guarantees the endpoint is reachable

    // If the centre deadband is as wide as (or wider than) the half-span, `from`
    // has crossed past `to` and the usable interval has collapsed or inverted.
    // Report 0 rather than let mapToWire divide by a delta of the opposite sign,
    // which would reverse the axis over its whole travel.
    if ((to - from) * ((span > 0) ? 1 : -1) <= 0) {
        return 0;
    }
    return (uint16_t)mapToWire(vQ6, from, to);
}

// Maps a unidirectional axis (brake) between two calibration points onto 0..1000.
inline uint16_t mapUnipolarQ6(int32_t vQ6, int32_t loQ6, int32_t hiQ6)
{
    const int32_t span = hiQ6 - loQ6;
    if (span == 0) {
        return 0;
    }
    const int32_t deadband = span / 20; // 5% at each end
    return (uint16_t)mapToWire(vQ6, loQ6 + deadband, hiQ6 - deadband);
}

// Full bipolar rudder axis: -1000..1000, left negative.
inline int16_t mapRudderQ6(int32_t vQ6, int32_t minQ6, int32_t centerQ6, int32_t maxQ6, int32_t centerDeadbandQ6)
{
    const int32_t d = vQ6 - centerQ6;
    if (d <= centerDeadbandQ6 && -d <= centerDeadbandQ6) {
        return 0;
    }

    // Which half of the travel are we on? Decided against the signed direction
    // of the max endpoint, so it stays correct for inverted wiring too.
    const bool towardsMax = (maxQ6 > centerQ6) ? (vQ6 > centerQ6) : (vQ6 < centerQ6);

    if (towardsMax) {
        return (int16_t)mapHalfQ6(vQ6, centerQ6, maxQ6, centerDeadbandQ6);
    }
    return (int16_t)-(int16_t)mapHalfQ6(vQ6, centerQ6, minQ6, centerDeadbandQ6);
}

#endif // AXIS_MAPPING_H
