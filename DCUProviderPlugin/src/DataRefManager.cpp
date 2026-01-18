#include "DataRefManager.h"
#include <cstdio>

DataRefManager::DataRefManager() = default;

DataRefManager::~DataRefManager() = default;

void DataRefManager::initialize() {
    // Fuel
    dr_fuelL = XPLMFindDataRef("sim/flightmodel/weight/fuel/fuel_tank[0]/current_weight_kgs");
    dr_fuelR = XPLMFindDataRef("sim/flightmodel/weight/fuel/fuel_tank[1]/current_weight_kgs");
    
    // Panel/Radio brightness and dome light
    dr_panelDim = XPLMFindDataRef("sim/cockpit/electrical/instrument_brightness");
    dr_radioDim = XPLMFindDataRef("sim/cockpit2/switches/instrument_brightness_ratio");
    dr_domeArr  = XPLMFindDataRef("sim/cockpit2/switches/panel_brightness_ratio");
    
    // Cache all datarefs for later use
    // TODO: Load from CSV or config file
    dataRefCache_.clear();
}

float DataRefManager::getFuelLeft() const {
    return readFloat(dr_fuelL, 0.0f);
}

float DataRefManager::getFuelRight() const {
    return readFloat(dr_fuelR, 0.0f);
}

float DataRefManager::getPanelBrightness() const {
    return readFloat(dr_panelDim, 0.0f);
}

float DataRefManager::getRadioBrightness() const {
    return readFloatArrayIdx0(dr_radioDim, 0.0f);
}

bool DataRefManager::getDomeLightOn() const {
    float val = readFloatArrayIdx0(dr_domeArr, 0.0f);
    return val > 0.5f;
}

void DataRefManager::setBarometerSetting(float inHg) {
    XPLMDataRef dr = XPLMFindDataRef("sim/cockpit/misc/barometer_setting");
    if (dr) {
        XPLMSetDataf(dr, inHg);
    }
}

void DataRefManager::setHeadingBug(float degrees) {
    XPLMDataRef dr = XPLMFindDataRef("sim/cockpit/autopilot/heading_bug_deg_mag_pil");
    if (dr) {
        XPLMSetDataf(dr, degrees);
    }
}

float DataRefManager::readFloat(XPLMDataRef dr, float def) {
    if (!dr) {
        return def;
    }
    return XPLMGetDataf(dr);
}

float DataRefManager::readFloatArrayIdx0(XPLMDataRef dr, float def) {
    if (!dr) {
        return def;
    }
    float val = 0.0f;
    XPLMGetDatavf(dr, &val, 0, 1);
    return val;
}