#include "BrakeEqualizer.h"
#include <algorithm>
#include <cmath>

namespace {

// Smallest span the rescale divides by. Keeps diffDeadZone = 1 finite.
constexpr float kMinSpan = 0.05f;

float clamp01(float v) { return std::max(0.f, std::min(1.f, v)); }

} // namespace

void BrakeEqualizer::clampToRange() {
    diffDeadZone = std::isnan(diffDeadZone) ? 0.f : clamp01(diffDeadZone);
}

void BrakeEqualizer::apply(float& left, float& right) const {
    const float l = std::isnan(left) ? 0.f : clamp01(left);
    const float r = std::isnan(right) ? 0.f : clamp01(right);

    const float mean = 0.5f * (l + r);
    const float d = std::fabs(l - r);
    const float dz = std::min(diffDeadZone, 1.f - kMinSpan);
    if (d <= dz) {
        left = right = mean;
        return;
    }
    // Rescale the overshoot past the dead zone back to full range: the
    // differential component grows from zero instead of jumping at the edge,
    // and d = 1 still yields the full difference.
    const float eff = (d - dz) / std::max(kMinSpan, 1.f - dz);
    const float sign = (l > r) ? 1.f : -1.f;
    left = clamp01(mean + sign * 0.5f * eff);
    right = clamp01(mean - sign * 0.5f * eff);
}
