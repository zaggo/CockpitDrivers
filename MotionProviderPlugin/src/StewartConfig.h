#pragma once

// Fixed rotary 6-RSS Stewart-platform geometry. Legs indexed 0..5 == P1..P6.
// Moves to a TOML file in Phase 2a; kept as constants here so Phase 2 has no
// config-parsing dependency. Lengths mm, angles deg.
namespace stewart {
    constexpr int    kLegs = 6;
    constexpr double kRb   = 425.0;  // base servo pivot radius
    constexpr double kRp   = 480.0;  // platform anchor radius
    constexpr double kHorn = 100.0;  // servo horn length (a)
    constexpr double kRod  = 466.0;  // push-rod length (s)

    // Base servo angle (phi), platform anchor angle (psi), horn azimuth (beta).
    constexpr double kPhi[6]  = { 60.0,   0.0, 300.0, 240.0, 180.0, 120.0 };
    constexpr double kPsi[6]  = { 83.75, 336.25, 323.75, 216.25, 203.75, 96.25 };
    constexpr double kBeta[6] = { 150.0, 270.0,  30.0, 150.0, 270.0,  30.0 };

    // BFF actuator index (1..6) each leg P1..P6 is wired to.
    constexpr int kBff[6] = { 5, 6, 1, 2, 3, 4 };

    // Servo angle -> 16-bit demand mapping.
    constexpr double kAngleAtFullScale = 45.0; // deg at demand extremes
    constexpr int    kDemandHome = 32640;      // theta = 0
    constexpr int    kDemandMax  = 65280;      // theta = +kAngleAtFullScale
}
