#pragma once
#include "XPLMDataAccess.h"
#include "MotionCues.h"

// The single owner of all XPLMFindDataRef / XPLMGetData* calls for this plugin.
class DataRefManager {
public:
    DataRefManager();
    ~DataRefManager();

    DataRefManager(const DataRefManager&) = delete;
    DataRefManager& operator=(const DataRefManager&) = delete;

    // Resolve datarefs once at startup (delegates to onAircraftLoaded()).
    void initialize();

    // (Re)resolve datarefs; called at init and on XPLM_MSG_PLANE_LOADED.
    void onAircraftLoaded();

    // Read all cues into a fresh snapshot. Safe to call every tick.
    MotionCues sample() const;

private:
    static float readFloat(XPLMDataRef dr, float def = 0.0f);
    static int   readInt(XPLMDataRef dr, int def = 0);
    static float readFloatArrayElem(XPLMDataRef dr, int index);

    XPLMDataRef dr_gAxil_ = nullptr;
    XPLMDataRef dr_gSide_ = nullptr;
    XPLMDataRef dr_gNrml_ = nullptr;
    XPLMDataRef dr_p_ = nullptr;
    XPLMDataRef dr_q_ = nullptr;
    XPLMDataRef dr_r_ = nullptr;
    XPLMDataRef dr_theta_ = nullptr;
    XPLMDataRef dr_phi_ = nullptr;
    XPLMDataRef dr_onGround_ = nullptr;
    XPLMDataRef dr_groundspeed_ = nullptr;
    XPLMDataRef dr_engineRpm_ = nullptr;
    XPLMDataRef dr_alpha_ = nullptr;
    XPLMDataRef dr_paused_ = nullptr;
};
