#include "SafetyMonitor.h"
#include <cmath>

void SafetyMonitor::update(const Pose& c, bool finite, bool serialLostWhileArmed, double dt) {
    if (fault_ != FaultCode::None) return;   // latched
    if (dt < 0.0) dt = 0.0;

    if (!finite) { fault_ = FaultCode::Nan; return; }
    if (serialLostWhileArmed) { fault_ = FaultCode::SerialLost; return; }

    const bool oob =
        std::fabs(c.roll)  > cfg_.runawayTiltDeg ||
        std::fabs(c.pitch) > cfg_.runawayTiltDeg ||
        std::fabs(c.yaw)   > cfg_.runawayTiltDeg ||
        std::fabs(c.surge) > cfg_.runawayTransMm ||
        std::fabs(c.sway)  > cfg_.runawayTransMm ||
        std::fabs(c.heave) > cfg_.runawayTransMm;

    if (oob) {
        oobAccum_ += dt;
        if (oobAccum_ >= cfg_.runawayHoldSec) fault_ = FaultCode::Runaway;
    } else {
        oobAccum_ = 0.0;
    }
}

const char* SafetyMonitor::reason() const {
    switch (fault_) {
        case FaultCode::Nan:        return "FAULT: NaN in command";
        case FaultCode::Runaway:    return "FAULT: runaway command";
        case FaultCode::SerialLost: return "FAULT: serial link lost";
        default:                    return "";
    }
}
