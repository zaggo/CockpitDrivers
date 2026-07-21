#pragma once
#include <string>
#include <cstdint>
#include "MotionCues.h"
#include "StewartKinematics.h"
#include "Pose.h"

// Clickable UI actions the status window emits (via the command callback).
enum UiAction {
    UI_RELOAD = 0,
    UI_NEXT_AXIS,
    UI_NUDGE_MINUS,
    UI_NUDGE_PLUS,
    UI_RESET,
    UI_RESCAN_PORTS,
    UI_TOGGLE_MODE,   // toggle SIM <-> MANUAL (only while disarmed)
    UI_DISARM         // manual e-stop
};

// Everything the status window renders in one snapshot.
struct StatusData {
    MotionCues  cues;
    SolveResult solve;
    bool        manualMode = false;
    int         manualAxis = 0;      // 0..5: surge,sway,heave,roll,pitch,yaw; 6: Identify
    Pose        manualPose;
    std::string identifyActor;       // selected actor name in Identify mode (axis 6)
    Pose        commandedPose;       // pose actually fed to the IK this tick
    bool        lastReloadOk = true;
    bool        reloadFlash = false; // show transient "Config loaded" confirmation
    uint16_t    sentSetpoints[6] = {32640,32640,32640,32640,32640,32640};
    int         armState = 0;        // ArmState: 0 Disarmed,1 Arming,2 Armed,3 Disarming
    float       armBlend = 0.0f;     // 0 = park pose, 1 = live pose
    int         faultCode = 0;       // FaultCode: 0 None,1 Nan,2 Runaway,3 SerialLost
    std::string faultReason;         // human-readable, empty if no fault
    bool        serialConnected = false;
    unsigned long long framesSent = 0;
    std::string serialPort;         // empty = none selected
    bool        heartbeatPresent = false;  // fresh arm heartbeat from the gateway
    bool        heartbeatArmed = false;    // last decoded arm-switch state (valid if present)
};
