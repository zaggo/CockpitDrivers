#pragma once
#include <cstdint>

// Runtime rotary 6-RSS geometry. defaults() reproduces the confirmed rig.
// Legs indexed 0..5 == P1..P6. Lengths mm, angles deg.
struct StewartGeometry {
    double baseRadius     = 425.0;   // Rb: base servo pivot radius
    double platformRadius = 480.0;   // Rp: platform anchor radius
    double hornLength     = 100.0;   // a
    double rodLength      = 466.0;   // s

    double phiDeg[6]  = { 60.0,   0.0, 300.0, 240.0, 180.0, 120.0 };
    double psiDeg[6]  = { 83.75, 336.25, 323.75, 216.25, 203.75, 96.25 };
    double betaDeg[6] = { 150.0, 270.0,  30.0, 150.0, 270.0,  30.0 };
    int    bff[6]     = { 5, 6, 1, 2, 3, 4 };
    // Physical actuator/node name per leg (P1..P6) for identify/debug display.
    const char* legName[6] = { "N1C2", "N2C1", "N2C2", "N3C1", "N3C2", "N1C1" };

    double angleAtFullScale = 45.0;  // deg mapped to demand extremes
    int    demandHome = 32640;
    int    demandMax  = 65280;

    static StewartGeometry defaults() { return StewartGeometry{}; }
};
