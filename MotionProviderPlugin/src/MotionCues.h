#pragma once

// One sampled snapshot of the flight-model inputs the motion platform reacts to.
// Plain data; produced by DataRefManager, consumed by MotionProvider/StatusWindow
// (and, in later phases, the washout filter and effects layer).
struct MotionCues {
    // Specific forces (body frame), in g
    float surgeG = 0.0f;   // sim/flightmodel/forces/g_axil
    float swayG  = 0.0f;   // sim/flightmodel/forces/g_side
    float heaveG = 0.0f;   // sim/flightmodel/forces/g_nrml

    // Angular rates, deg/s
    float rollRate  = 0.0f; // sim/flightmodel/position/P
    float pitchRate = 0.0f; // sim/flightmodel/position/Q
    float yawRate   = 0.0f; // sim/flightmodel/position/R

    // Attitude, deg
    float pitchDeg = 0.0f;  // sim/flightmodel/position/theta
    float rollDeg  = 0.0f;  // sim/flightmodel/position/phi

    // Effects inputs
    bool  onGround    = false; // sim/flightmodel/failures/onground_any
    float groundspeed = 0.0f;  // sim/flightmodel/position/groundspeed (m/s)
    float engineRpm   = 0.0f;  // sim/cockpit2/engine/indicators/engine_speed_rpm[0]
    float alphaDeg    = 0.0f;  // sim/flightmodel/position/alpha

    bool simPaused = false;    // sim/time/paused
};
