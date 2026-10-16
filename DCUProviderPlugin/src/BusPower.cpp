#include "BusPower.h"

#include <cmath>

BusPower::BusPower() = default;

void BusPower::update(float volts)
{
    if (!std::isfinite(volts))
    {
        return;
    }

    float f = (volts - kLightsOutVolts) / (kLightsFullVolts - kLightsOutVolts);
    lightFactor_ = f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f);

    if (instrumentsPowered_)
    {
        instrumentsPowered_ = volts > kInstrumentsOffVolts;
    }
    else
    {
        instrumentsPowered_ = volts >= kInstrumentsOnVolts;
    }
}
