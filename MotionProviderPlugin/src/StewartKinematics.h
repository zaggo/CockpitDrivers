#pragma once
#include <cstdint>
#include "Pose.h"

struct LegResult {
    double angleDeg = 0.0;   // servo horn elevation, deg (0 = horizontal/home)
    bool   reachable = true; // false if the pose is outside this leg's envelope
};

struct SolveResult {
    LegResult legs[6];        // P1..P6 order
    uint16_t  setpoints[6];   // BFF actuator order (index 0 == BFF #1)
    bool      allReachable = true;
};

class StewartKinematics {
public:
    // Home platform height (mm) at which all horns rest horizontal. Derived
    // from geometry; equal for all legs by symmetry.
    static double homeHeight();

    // Solve inverse kinematics for a pose. Never throws; unreachable legs are
    // flagged and their angle is clamped to the envelope edge.
    static SolveResult solve(const Pose& pose);
};
