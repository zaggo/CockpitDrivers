#pragma once
#include <cstdint>
#include "Pose.h"
#include "StewartGeometry.h"

struct LegResult {
    double angleDeg = 0.0;
    bool   reachable = true;
};

struct SolveResult {
    LegResult legs[6];
    uint16_t  setpoints[6];   // BFF actuator order (index 0 == BFF #1)
    bool      allReachable = true;
};

class StewartKinematics {
public:
    explicit StewartKinematics(const StewartGeometry& geo);

    double homeHeight() const { return z0_; }      // mm
    SolveResult solve(const Pose& pose) const;
    const StewartGeometry& geometry() const { return geo_; }

private:
    StewartGeometry geo_;
    double z0_ = 0.0;
};
