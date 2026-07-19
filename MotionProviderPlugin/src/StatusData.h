#pragma once
#include <string>
#include <cstdint>
#include "MotionCues.h"
#include "StewartKinematics.h"
#include "Pose.h"

// Clickable UI actions the status window emits (via the command callback).
enum UiAction {
    UI_RELOAD = 0,
    UI_TOGGLE_MODE,
    UI_NEXT_AXIS,
    UI_NUDGE_MINUS,
    UI_NUDGE_PLUS,
    UI_RESET,
    UI_ARM_TOGGLE,
    UI_RESCAN_PORTS
};

// Everything the status window renders in one snapshot.
struct StatusData {
    MotionCues  cues;
    SolveResult solve;
    bool        manualMode = false;
    int         manualAxis = 0;      // 0..5: surge,sway,heave,roll,pitch,yaw
    Pose        manualPose;
    Pose        commandedPose;       // pose actually fed to the IK this tick
    bool        lastReloadOk = true;
    bool        reloadFlash = false; // show transient "Config loaded" confirmation
    uint16_t    sentSetpoints[6] = {32640,32640,32640,32640,32640,32640};
    bool        armed = false;
    bool        serialConnected = false;
    unsigned long long framesSent = 0;
    std::string serialPort;         // empty = none selected
};
