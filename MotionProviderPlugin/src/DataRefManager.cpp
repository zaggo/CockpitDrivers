#include "DataRefManager.h"

DataRefManager::DataRefManager() = default;
DataRefManager::~DataRefManager() = default;

void DataRefManager::initialize() {
    onAircraftLoaded();
}

void DataRefManager::onAircraftLoaded() {
    dr_gAxil_       = XPLMFindDataRef("sim/flightmodel/forces/g_axil");
    dr_gSide_       = XPLMFindDataRef("sim/flightmodel/forces/g_side");
    dr_gNrml_       = XPLMFindDataRef("sim/flightmodel/forces/g_nrml");
    dr_p_           = XPLMFindDataRef("sim/flightmodel/position/P");
    dr_q_           = XPLMFindDataRef("sim/flightmodel/position/Q");
    dr_r_           = XPLMFindDataRef("sim/flightmodel/position/R");
    dr_theta_       = XPLMFindDataRef("sim/flightmodel/position/theta");
    dr_phi_         = XPLMFindDataRef("sim/flightmodel/position/phi");
    dr_onGround_    = XPLMFindDataRef("sim/flightmodel/failures/onground_any");
    dr_groundspeed_ = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
    dr_engineRpm_   = XPLMFindDataRef("sim/cockpit2/engine/indicators/engine_speed_rpm");
    dr_alpha_       = XPLMFindDataRef("sim/flightmodel/position/alpha");
    dr_paused_      = XPLMFindDataRef("sim/time/paused");
}

MotionCues DataRefManager::sample() const {
    MotionCues c;
    c.surgeG      = readFloat(dr_gAxil_);
    c.swayG       = readFloat(dr_gSide_);
    c.heaveG      = readFloat(dr_gNrml_);
    c.rollRate    = readFloat(dr_p_);
    c.pitchRate   = readFloat(dr_q_);
    c.yawRate     = readFloat(dr_r_);
    c.pitchDeg    = readFloat(dr_theta_);
    c.rollDeg     = readFloat(dr_phi_);
    c.onGround    = readInt(dr_onGround_) != 0;
    c.groundspeed = readFloat(dr_groundspeed_);
    c.engineRpm   = readFloatArrayElem(dr_engineRpm_, 0);
    c.alphaDeg    = readFloat(dr_alpha_);
    c.simPaused   = readInt(dr_paused_) != 0;
    return c;
}

float DataRefManager::readFloat(XPLMDataRef dr, float def) {
    return dr ? XPLMGetDataf(dr) : def;
}

int DataRefManager::readInt(XPLMDataRef dr, int def) {
    return dr ? XPLMGetDatai(dr) : def;
}

float DataRefManager::readFloatArrayElem(XPLMDataRef dr, int index) {
    if (!dr) return 0.0f;
    float v = 0.0f;
    XPLMGetDatavf(dr, &v, index, 1);
    return v;
}
