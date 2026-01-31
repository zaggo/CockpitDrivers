#pragma once

#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"

#include <string>
#include <map>
#include <vector>

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
    std::vector<float> getPanelBrightness() const;  // [0] and [1], 0.0 - 1.0
    float getDomeLightBrightness() const;  // 0.0 - 1.0

    // Transponder
    uint16_t getTransponderCode() const;
    uint8_t getTransponderMode() const;
    bool getTransponderLight() const;
    void setTransponderCode(uint16_t code);
    void setTransponderMode(uint8_t mode);
    void transponderIdentOnce();
    
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
        
    /// Helper: read multiple floats from array (returns vector)
    static std::vector<float> readFloatArray(XPLMDataRef dr, int index, int count);

    // Cached datarefs (downlink)
    XPLMDataRef dr_fuelL = nullptr;
    XPLMDataRef dr_fuelR = nullptr;
    XPLMDataRef dr_panelDim = nullptr;
    XPLMDataRef dr_domeLightDim = nullptr;
    XPLMDataRef dr_HeadingBug = nullptr;
    XPLMDataRef dr_BarometerSetting = nullptr;
    XPLMDataRef dr_TransponderCode = nullptr;
    XPLMDataRef dr_TransponderModeR = nullptr;
    XPLMDataRef dr_TransponderModeW = nullptr;
    XPLMDataRef dr_TransponderLight = nullptr;

    XPLMCommandRef cr_TransponderIdent = nullptr;
};