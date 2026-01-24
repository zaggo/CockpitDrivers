#include "DataRefManager.h"
#include <cstdio>

DataRefManager::DataRefManager() = default;

DataRefManager::~DataRefManager() = default;

void DataRefManager::initialize() {
    // Fuel
    dr_fuelL = XPLMFindDataRef("sim/cockpit2/fuel/fuel_level_indicated_left");
    dr_fuelR = XPLMFindDataRef("sim/cockpit2/fuel/fuel_level_indicated_right");
    
    // Panel/Radio brightness and dome light
    dr_panelDim = XPLMFindDataRef("sim/cockpit2/switches/instrument_brightness_ratio");
    dr_domeLightDim  = XPLMFindDataRef("sim/cockpit2/switches/panel_brightness_ratio");

    dr_HeadingBug = XPLMFindDataRef("sim/cockpit/autopilot/heading_bug_deg_mag_pil");
    dr_BarometerSetting = XPLMFindDataRef("sim/cockpit/misc/barometer_setting");
}

float DataRefManager::getFuelLeft() const {
    return readFloat(dr_fuelL, 0.0f);
}

float DataRefManager::getFuelRight() const {
    return readFloat(dr_fuelR, 0.0f);
}

std::vector<float> DataRefManager::getPanelBrightness() const {
    return readFloatArray(dr_panelDim, 0, 2);
}

float DataRefManager::getDomeLightBrightness() const {
    auto values = readFloatArray(dr_domeLightDim, 1, 1);
    return values[0];
}

void DataRefManager::setBarometerSetting(float inHg) {
    if (dr_BarometerSetting) {
        XPLMSetDataf(dr_BarometerSetting, inHg);
    }
}

void DataRefManager::setHeadingBug(float degrees) {
    if (dr_HeadingBug) {
        XPLMSetDataf(dr_HeadingBug, degrees);
    }
}

float DataRefManager::readFloat(XPLMDataRef dr, float def) {
    if (!dr) {
        return def;
    }
    return XPLMGetDataf(dr);
}

std::vector<float> DataRefManager::readFloatArray(XPLMDataRef dr, int index, int count) {
    std::vector<float> result(count, 0.0f);
    if (dr && count > 0) {
        XPLMGetDatavf(dr, result.data(), index, count);
    }
    return result;
}