#ifndef ADAPTIVE_FILTER_H
#define ADAPTIVE_FILTER_H

#include <stdint.h>

// Speed-adaptive exponential moving average over one raw ADC channel.
//
// The estimate is kept in Q6 fixed point (raw << 6) so it carries sub-LSB
// resolution into the axis mapping instead of being rounded back to whole ADC
// counts. The smoothing factor rises with the distance between the incoming
// sample and the current estimate: standing still it stays at `alphaMin` and
// smooths hard, on fast movement it saturates at 256/256 and the filter becomes
// transparent. That is what keeps the axis quiet at rest without adding lag
// when a pedal is actually moved.
class AdaptiveFilter
{
public:
    // alphaMin: Q8 smoothing factor applied when the input is not moving.
    // slope:    added to alpha per LSB of deviation between input and estimate.
    AdaptiveFilter(int32_t alphaMin = 32, int32_t slope = 12)
        : _alphaMin(alphaMin), _slope(slope), _q6(0), _seeded(false) {}

    // Jumps the estimate straight to `raw`, skipping the settling ramp.
    void reset(uint16_t raw)
    {
        _q6 = (int32_t)raw << 6;
        _seeded = true;
    }

    void update(uint16_t raw)
    {
        if (!_seeded) {
            reset(raw);
            return;
        }

        const int32_t target = (int32_t)raw << 6;
        const int32_t d      = target - _q6;
        const int32_t dLsb   = ((d < 0) ? -d : d) >> 6;

        int32_t alpha = _alphaMin + _slope * dLsb;
        if (alpha > 256) {
            alpha = 256;
        }

        // Division, not a shift: it truncates towards zero, so the residual
        // sub-step deadband is symmetric. An arithmetic shift would floor and
        // leave the estimate creeping downwards on small negative deltas.
        _q6 += (d * alpha) / 256;
    }

    bool seeded() const { return _seeded; }

    // Filtered value in Q6 (raw << 6).
    int32_t valueQ6() const { return _q6; }

private:
    int32_t _alphaMin;
    int32_t _slope;
    int32_t _q6;
    bool    _seeded;
};

#endif // ADAPTIVE_FILTER_H
