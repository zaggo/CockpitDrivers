#pragma once
#include "MotionCues.h"
#include "StewartKinematics.h"
#include "Pose.h"

// Everything the status window renders in one snapshot.
struct StatusData {
    MotionCues  cues;
    SolveResult solve;
    bool        manualMode = false;
    int         manualAxis = 0;      // 0..5: surge,sway,heave,roll,pitch,yaw
    Pose        manualPose;
    bool        lastReloadOk = true;
};
