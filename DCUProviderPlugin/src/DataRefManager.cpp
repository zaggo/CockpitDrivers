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

    dr_TransponderCode = XPLMFindDataRef("sim/cockpit2/radios/actuators/transponder_code");
    dr_TransponderModeR = XPLMFindDataRef("sim/cockpit/radios/transponder_mode");
    dr_TransponderModeW = XPLMFindDataRef("VFLYTEAIR/AXP340/AXP340_MODE");
    dr_TransponderLight = XPLMFindDataRef("sim/cockpit/radios/transponder_light");

    cr_TransponderIdent = XPLMFindCommand("sim/transponder/transponder_ident");
}

// Fuel
float DataRefManager::getFuelLeft() const {
    return readFloat(dr_fuelL, 0.0f);
}

float DataRefManager::getFuelRight() const {
    return readFloat(dr_fuelR, 0.0f);
}

// Lights
std::vector<float> DataRefManager::getPanelBrightness() const {
    return readFloatArray(dr_panelDim, 0, 2);
}

float DataRefManager::getDomeLightBrightness() const {
    auto values = readFloatArray(dr_domeLightDim, 1, 1);
    return values[0];
}

// Altimeter
void DataRefManager::setBarometerSetting(float inHg) {
    if (dr_BarometerSetting) {
        XPLMSetDataf(dr_BarometerSetting, inHg);
    }
}

// HSI
void DataRefManager::setHeadingBug(float degrees) {
    if (dr_HeadingBug) {
        XPLMSetDataf(dr_HeadingBug, degrees);
    }
}

// Transponder
uint16_t DataRefManager::getTransponderCode() const {
    return static_cast<uint16_t>(XPLMGetDatai(dr_TransponderCode));
}

uint8_t DataRefManager::getTransponderMode() const {
    return static_cast<uint8_t>(XPLMGetDatai(dr_TransponderModeR));
}

bool DataRefManager::getTransponderLight() const {
    return static_cast<bool>(XPLMGetDatai(dr_TransponderLight)) ;
}
    
void DataRefManager::setTransponderCode(uint16_t code) {
    if (dr_TransponderCode) {
        XPLMSetDatai(dr_TransponderCode, static_cast<int>(code));
    }
}

void DataRefManager::setTransponderMode(uint8_t mode) {
    int timeout = 10;
    XPLMCommandRef cmd = XPLMFindCommand("sim/transponder/transponder_ident");
    while (cmd != nullptr && mode != getTransponderMode() && timeout-- > 0) {
        XPLMCommandOnce(cmd);   
    }
}

void DataRefManager::transponderIdentOnce() {
    if (cr_TransponderIdent)
        XPLMCommandOnce(cr_TransponderIdent);   
}

// ----

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