#pragma once

#include "XPLMDataAccess.h"
#include <string>
#include <map>

class DataRefManager {
public:
    DataRefManager();
    ~DataRefManager();
    
    // Prevent copying
    DataRefManager(const DataRefManager&) = delete;
    DataRefManager& operator=(const DataRefManager&) = delete;
    
    /// Initializes all datarefs.
    /// Must be called once during plugin startup.
    void initialize();
    
    // ============ DOWNLINK (Plugin → Gateway) ============
    // Fuel system
    float getFuelLeft() const;
    float getFuelRight() const;
    
    // Lights
    float getPanelBrightness() const;      // 0.0 - 1.0
    float getRadioBrightness() const;      // 0.0 - 1.0
    bool  getDomeLightOn() const;          // bool
    
    // ============ UPLINK (Gateway → Plugin) ============
    /// Sets barometer altimeter setting (inHg)
    void setBarometerSetting(float inHg);
    
    /// Sets autopilot heading bug (degrees)
    void setHeadingBug(float degrees);
    
    // TODO: Add more getters/setters based on CAN Message IDs
    // - Altimeter QNH setting
    // - VSI target
    // - Heading bug
    // - Course selector
    // - etc.
    
private:
    /// Helper: read float from dataref
    static float readFloat(XPLMDataRef dr, float def = 0.0f);
    
    /// Helper: read float array at index 0
    static float readFloatArrayIdx0(XPLMDataRef dr, float def = 0.0f);
    
    // Cached datarefs (downlink)
    XPLMDataRef dr_fuelL = nullptr;
    XPLMDataRef dr_fuelR = nullptr;
    XPLMDataRef dr_panelDim = nullptr;
    XPLMDataRef dr_radioDim = nullptr;
    XPLMDataRef dr_domeArr = nullptr;
    
    // Optional: Cache für dynamisch geladene Datarefs
    std::map<std::string, XPLMDataRef> dataRefCache_;
};