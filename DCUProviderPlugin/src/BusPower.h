#pragma once

// Central brownout model for the whole dashboard, fed once per downlink tick
// with the electrical bus voltage (sim/cockpit2/electrical/bus_volts).
//
// Two outputs, both derived from the same voltage:
//
//   lightFactor()         0..1 brightness factor for every dashboard light
//                         (panel, radio, dome). Full at/above kLightsFullVolts,
//                         linear fade down to fully dark at kLightsOutVolts.
//
//   instrumentsPowered()  true while bus-powered devices (fuel gauge, HSI
//                         needles/flags, turn coordinator needle, transponder)
//                         may show live values. Hysteresis around ~10V keeps a
//                         flickering bus from toggling the whole panel.
//
// The thresholds are sized for the Arrow's 14 V bus, calibrated against
// measured values: ~14.2 V with the engine running, ~12.1 V on battery alone.
// Battery-only therefore already sits at ~75% lighting, avionics die first as
// the battery sags, and the last glow of the lamps survives them - the order a
// real cockpit goes dark in. Vacuum (attitude), pitot-static (ASI, altimeter,
// VSI), magnetic (whiskey compass) and mechanical (tach) instruments are not
// consumers of this model at all; they keep running off-grid by design.
//
// XPLM-free on purpose, so the tests in tests/ can exercise it natively.
class BusPower {
public:
    BusPower();

    // Feed the current bus voltage, once per downlink tick. Non-finite input
    // (a broken dataref read) keeps the previous state.
    void update(float volts);

    float lightFactor() const { return lightFactor_; }
    bool instrumentsPowered() const { return instrumentsPowered_; }

    // Volts. See the class comment for the reasoning behind the ordering.
    // Full lighting needs the alternator (~14.2 V running); the ~12.1 V of a
    // healthy battery alone lands mid-fade.
    static constexpr float kLightsFullVolts = 13.5f;
    static constexpr float kLightsOutVolts = 8.0f;
    static constexpr float kInstrumentsOnVolts = 10.5f;
    static constexpr float kInstrumentsOffVolts = 10.0f;

private:
    // Both start healthy: until the first update() says otherwise, behave the
    // way the plugin always has - everything on.
    float lightFactor_ = 1.0f;
    bool instrumentsPowered_ = true;
};
