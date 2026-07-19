#pragma once
#include "Pose.h"
#include "SafetyConfig.h"

enum class FaultCode { None, Nan, Runaway, SerialLost };

// Evaluates and LATCHES safety faults. Pure, dt-driven, no X-Plane deps.
class SafetyMonitor {
public:
    void setConfig(const SafetyConfig& cfg) { cfg_ = cfg; }

    // Observe one tick. rawCmd is the pre-clamp commanded pose; finite is false
    // if any commanded value is NaN/inf; serialLostWhileArmed is true if the
    // link dropped while not fully disarmed. Once faulted, further calls are
    // ignored until clear().
    void update(const Pose& rawCmd, bool finite, bool serialLostWhileArmed, double dt);

    FaultCode   fault() const { return fault_; }
    const char* reason() const;
    void clear() { fault_ = FaultCode::None; oobAccum_ = 0.0; }

private:
    SafetyConfig cfg_;
    FaultCode fault_ = FaultCode::None;
    double oobAccum_ = 0.0;   // seconds the command has been out of sanity bounds
};
