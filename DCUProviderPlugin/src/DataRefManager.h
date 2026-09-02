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
    
    /// Called when a new aircraft is loaded.
    /// Reinitializes datarefs to match the new aircraft.
    void onAircraftLoaded();
    
    // ============ DOWNLINK (Plugin → Gateway) ============
    // Fuel system
    float getFuelLeft() const;
    float getFuelRight() const;
    
    // Lights
    std::vector<float> getPanelBrightness() const;  // [0] and [1], 0.0 - 1.0
    float getDomeLightBrightness() const;  // 0.0 - 1.0

    // RPM / Tach
    float getRpm() const;  // engine_speed_rpm[0], direct RPM value
    int8_t getTachHours1000() const;
    int8_t getTachHours100() const;
    int8_t getTachHours10() const;
    int8_t getTachHours1() const;
    float getTachHoursTenths() const;
    float getTachHoursHundredths() const;  // 0.0-1.0 fraction, *1000 on the wire

    // Airspeed indicator (ASI)
    float getIas() const;  // knots, indicated
    float getTas() const;  // knots, true

    // Altimeter + vertical speed indicator (both halves of one message)
    float getAltitudeFt() const;  // feet, pilot altimeter
    float getVsiFpm() const;      // feet/min, negative = descending

    // Transponder
    uint16_t getTransponderCode() const;
    uint8_t getTransponderMode() const;
    bool getTransponderLight() const;
    void setTransponderCode(uint16_t code);
    void setTransponderMode(uint8_t mode);
    void transponderIdentOnce();
    
    // Parking Brake
    void setParkingBrakeRatio(float ratio);

    // Rudder pedals + toe brakes
    void setRudderRatio(float ratio);      // -1.0 .. 1.0, left negative
    void setLeftBrakeRatio(float ratio);   //  0.0 .. 1.0
    void setRightBrakeRatio(float ratio);  //  0.0 .. 1.0

    /// Takes the rudder/toe-brake axes away from X-Plane's own joystick input, so the
    /// values written above survive instead of being overwritten every frame.
    /// Idempotent — only writes when the state actually changes.
    void setRudderOverrideEnabled(bool enabled);
    
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

    /// Helper: read int from dataref, returned as float
    static int readInt(XPLMDataRef dr, int def = 0);

    /// Helper: re-attempts XPLMFindDataRef if cached is still null.
    /// Some third-party (aircraft-specific) datarefs are registered by the aircraft's
    /// own plugin later than our onAircraftLoaded(), so the first lookup can miss them.
    static XPLMDataRef resolveLazy(XPLMDataRef& cached, const char* name);

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
    XPLMDataRef dr_ParkingBrake = nullptr;
    XPLMDataRef dr_Rudder = nullptr;
    XPLMDataRef dr_LeftBrake = nullptr;
    XPLMDataRef dr_RightBrake = nullptr;
    XPLMDataRef dr_OverrideJoystickHeading = nullptr;
    XPLMDataRef dr_OverrideToeBrakes = nullptr;
    bool rudderOverrideEnabled = false;

    XPLMDataRef dr_rpm = nullptr;
    mutable XPLMDataRef dr_tachHrs1000 = nullptr;
    mutable XPLMDataRef dr_tachHrs100 = nullptr;
    mutable XPLMDataRef dr_tachHrs10 = nullptr;
    mutable XPLMDataRef dr_tachHrs1 = nullptr;
    mutable XPLMDataRef dr_tachHrsTenths = nullptr;
    mutable XPLMDataRef dr_tachHrsHundredths = nullptr;

    XPLMDataRef dr_ias = nullptr;
    XPLMDataRef dr_tas = nullptr;

    XPLMDataRef dr_altitude = nullptr;
    XPLMDataRef dr_vsi = nullptr;

    XPLMCommandRef cr_TransponderIdent = nullptr;
};