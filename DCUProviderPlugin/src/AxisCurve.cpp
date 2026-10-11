#include "AxisCurve.h"
#include <algorithm>
#include <cmath>

namespace {

// Smallest span any rescale divides by. Keeps neutral = +-1 and
// deadZone/extremity = 1 finite instead of dividing by zero.
constexpr float kMinSpan = 0.05f;

float clamp1(float v) { return std::max(-1.f, std::min(1.f, v)); }

float sane(float v, float lo, float hi) {
    if (std::isnan(v)) return 0.f;
    return std::max(lo, std::min(hi, v));
}

} // namespace

void AxisCurve::clampToRange() {
    sensPos = sane(sensPos, -1.f, 1.f);
    sensNeg = sane(sensNeg, -1.f, 1.f);
    neutral = sane(neutral, -1.f, 1.f);
    deadZone = sane(deadZone, 0.f, 1.f);
    extremityDeadZone = sane(extremityDeadZone, 0.f, 1.f);
}

float AxisCurve::apply(float in) const {
    float x = std::isnan(in) ? 0.f : clamp1(in);

    // Neutral: shift the zero point, then stretch each half back to full
    // length so the pedal stops still reach +-1.
    x -= neutral;
    const float span = (x >= 0.f) ? std::max(kMinSpan, 1.f - neutral)
                                  : std::max(kMinSpan, 1.f + neutral);
    x = clamp1(x / span);

    const float sign = (x < 0.f) ? -1.f : 1.f;
    float m = std::fabs(x);

    // Dead zone around the centre, rescaled so there is no jump at the edge.
    const float dz = std::min(deadZone, 1.f - kMinSpan);
    m = (m <= dz) ? 0.f : (m - dz) / (1.f - dz);

    // Extremity dead zone: full deflection is reached before the stop.
    const float ext = std::min(extremityDeadZone, 1.f - kMinSpan);
    m = (m >= 1.f - ext) ? 1.f : m / (1.f - ext);

    // Sensitivity: power curve, p = 3^(-s). s=0 linear, s=-1 cubic, s=+1 cube root.
    const float s = (sign > 0.f) ? sensPos : sensNeg;
    m = std::pow(m, std::pow(3.f, -s));

    return clamp1(sign * m);
}
