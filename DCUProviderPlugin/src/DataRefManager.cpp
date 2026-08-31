#include "DataRefManager.h"
#include <cstdio>

DataRefManager::DataRefManager() = default;

DataRefManager::~DataRefManager() = default;

void DataRefManager::initialize()
{
    onAircraftLoaded();
}

void DataRefManager::onAircraftLoaded()
{
    // Fuel
    dr_fuelL = XPLMFindDataRef("sim/cockpit2/fuel/fuel_level_indicated_left");
    dr_fuelR = XPLMFindDataRef("sim/cockpit2/fuel/fuel_level_indicated_right");

    // Panel/Radio brightness and dome light
    dr_panelDim = XPLMFindDataRef("sim/cockpit2/switches/instrument_brightness_ratio");
    dr_domeLightDim = XPLMFindDataRef("sim/cockpit2/switches/panel_brightness_ratio");

    dr_HeadingBug = XPLMFindDataRef("sim/cockpit/autopilot/heading_bug_deg_mag_pil");
    dr_BarometerSetting = XPLMFindDataRef("sim/cockpit/misc/barometer_setting");

    dr_TransponderCode = XPLMFindDataRef("sim/cockpit2/radios/actuators/transponder_code");
    dr_TransponderModeR = XPLMFindDataRef("sim/cockpit/radios/transponder_mode");
    dr_TransponderModeW = XPLMFindDataRef("VFLYTEAIR/AXP340/AXP340_MODE");
    dr_TransponderLight = XPLMFindDataRef("sim/cockpit/radios/transponder_light");

    cr_TransponderIdent = XPLMFindCommand("sim/transponder/transponder_ident");

    dr_ParkingBrake = XPLMFindDataRef("sim/cockpit2/controls/parking_brake_ratio");

    // Rudder pedals + toe brakes. X-Plane rewrites these from its own joystick
    // input every frame, so they only hold while the override datarefs below are
    // set (see setRudderOverrideEnabled).
    dr_Rudder = XPLMFindDataRef("sim/joystick/yoke_heading_ratio");
    dr_LeftBrake = XPLMFindDataRef("sim/cockpit2/controls/left_brake_ratio");
    dr_RightBrake = XPLMFindDataRef("sim/cockpit2/controls/right_brake_ratio");

    dr_OverrideJoystickHeading = XPLMFindDataRef("sim/operation/override/override_joystick_heading");
    dr_OverrideToeBrakes = XPLMFindDataRef("sim/operation/override/override_toe_brakes");

    if (!dr_OverrideJoystickHeading)
    {
        XPLMDebugString("DCUProvider: dataref sim/operation/override/override_joystick_heading not found - rudder input will be overwritten by X-Plane\n");
    }
    if (!dr_OverrideToeBrakes)
    {
        XPLMDebugString("DCUProvider: dataref sim/operation/override/override_toe_brakes not found - toe brake input will be overwritten by X-Plane\n");
    }

    // X-Plane clears its override datarefs when an aircraft is loaded, so drop our
    // cached state here — the next rudder frame re-asserts it.
    rudderOverrideEnabled = false;

    // RPM / Tach (VFLYTEAIR is an aircraft-specific third-party dataref namespace;
    // absent on other aircraft, getters fall back to 0 via readFloat's default.
    // The aircraft's own plugin may register it later than our onAircraftLoaded(),
    // so the tach getters lazily retry via resolveLazy() if still null here.)
    dr_rpm = XPLMFindDataRef("sim/cockpit2/engine/indicators/engine_speed_rpm");
    dr_tachHrs1000 = XPLMFindDataRef("VFLYTEAIR/tach/TachTimeHrs1000");
    dr_tachHrs100 = XPLMFindDataRef("VFLYTEAIR/tach/TachTimeHrs100");
    dr_tachHrs10 = XPLMFindDataRef("VFLYTEAIR/tach/TachTimeHrs10");
    dr_tachHrs1 = XPLMFindDataRef("VFLYTEAIR/tach/TachTimeHrs1");
    dr_tachHrsTenths = XPLMFindDataRef("VFLYTEAIR/tach/TachTimeTenths");
    dr_tachHrsHundredths = XPLMFindDataRef("VFLYTEAIR/tach/TachTimeHundredths");

    // Airspeed indicator (ASI)
    dr_ias = XPLMFindDataRef("sim/cockpit2/gauges/indicators/airspeed_kts_pilot");
    dr_tas = XPLMFindDataRef("sim/cockpit2/gauges/indicators/true_airspeed_kts_pilot");
}

// Fuel
float DataRefManager::getFuelLeft() const
{
    return readFloat(dr_fuelL, 0.0f);
}

float DataRefManager::getFuelRight() const
{
    return readFloat(dr_fuelR, 0.0f);
}

// Lights
std::vector<float> DataRefManager::getPanelBrightness() const
{
    return readFloatArray(dr_panelDim, 0, 2);
}

float DataRefManager::getDomeLightBrightness() const
{
    auto values = readFloatArray(dr_domeLightDim, 1, 1);
    return values[0];
}

// RPM / Tach
float DataRefManager::getRpm() const
{
    auto values = readFloatArray(dr_rpm, 0, 1);
    return values[0];
}

int8_t DataRefManager::getTachHours1000() const
{
    return static_cast<int8_t>(readInt(resolveLazy(dr_tachHrs1000, "VFLYTEAIR/tach/TachTimeHrs1000"), 0));
}

int8_t DataRefManager::getTachHours100() const
{
    return static_cast<int8_t>(readInt(resolveLazy(dr_tachHrs100, "VFLYTEAIR/tach/TachTimeHrs100"), 0));
}

int8_t DataRefManager::getTachHours10() const
{
    return static_cast<int8_t>(readInt(resolveLazy(dr_tachHrs10, "VFLYTEAIR/tach/TachTimeHrs10"), 0));
}

int8_t DataRefManager::getTachHours1() const
{
    return static_cast<int8_t>(readInt(resolveLazy(dr_tachHrs1, "VFLYTEAIR/tach/TachTimeHrs1"), 0));
}

float DataRefManager::getTachHoursTenths() const
{
    return readFloat(resolveLazy(dr_tachHrsTenths, "VFLYTEAIR/tach/TachTimeTenths"), 0.0f);
}

float DataRefManager::getTachHoursHundredths() const
{
    return readFloat(resolveLazy(dr_tachHrsHundredths, "VFLYTEAIR/tach/TachTimeHundredths"), 0.0f);
}

// Airspeed indicator (ASI)
float DataRefManager::getIas() const
{
    return readFloat(dr_ias, 0.0f);
}

float DataRefManager::getTas() const
{
    return readFloat(dr_tas, 0.0f);
}

// Altimeter
void DataRefManager::setBarometerSetting(float inHg)
{
    if (dr_BarometerSetting)
    {
        XPLMSetDataf(dr_BarometerSetting, inHg);
    }
}

// HSI
void DataRefManager::setHeadingBug(float degrees)
{
    if (dr_HeadingBug)
    {
        XPLMSetDataf(dr_HeadingBug, degrees);
    }
}

// Transponder
uint16_t DataRefManager::getTransponderCode() const
{
    return static_cast<uint16_t>(XPLMGetDatai(dr_TransponderCode));
}

uint8_t DataRefManager::getTransponderMode() const
{
    return static_cast<uint8_t>(XPLMGetDatai(dr_TransponderModeR));
}

bool DataRefManager::getTransponderLight() const
{
    return static_cast<bool>(XPLMGetDatai(dr_TransponderLight));
}

void DataRefManager::setTransponderCode(uint16_t code)
{
    if (dr_TransponderCode)
    {
        XPLMSetDatai(dr_TransponderCode, static_cast<int>(code));
    }
}

void DataRefManager::setTransponderMode(uint8_t mode)
{
    if (dr_TransponderModeW == nullptr) {
        dr_TransponderModeW = XPLMFindDataRef("VFLYTEAIR/AXP340/AXP340_MODE");  // Third-party dataref might not exist at init
    }
    if (dr_TransponderModeW)
    {
        XPLMSetDatai(dr_TransponderModeW, static_cast<int>(mode));
    }
}

void DataRefManager::transponderIdentOnce()
{
    if (cr_TransponderIdent)
        XPLMCommandOnce(cr_TransponderIdent);
}

// Parking Brake
void DataRefManager::setParkingBrakeRatio(float ratio)
{
    if (dr_ParkingBrake)
    {
        XPLMSetDataf(dr_ParkingBrake, ratio);
    }
}
// Rudder pedals + toe brakes
void DataRefManager::setRudderRatio(float ratio)
{
    if (dr_Rudder)
    {
        XPLMSetDataf(dr_Rudder, ratio);
    }
}

void DataRefManager::setLeftBrakeRatio(float ratio)
{
    if (dr_LeftBrake)
    {
        XPLMSetDataf(dr_LeftBrake, ratio);
    }
}

void DataRefManager::setRightBrakeRatio(float ratio)
{
    if (dr_RightBrake)
    {
        XPLMSetDataf(dr_RightBrake, ratio);
    }
}

void DataRefManager::setRudderOverrideEnabled(bool enabled)
{
    if (enabled == rudderOverrideEnabled)
    {
        return;
    }
    rudderOverrideEnabled = enabled;

    const int value = enabled ? 1 : 0;
    if (dr_OverrideJoystickHeading)
    {
        XPLMSetDatai(dr_OverrideJoystickHeading, value);
    }
    if (dr_OverrideToeBrakes)
    {
        XPLMSetDatai(dr_OverrideToeBrakes, value);
    }

    XPLMDebugString(enabled ? "DCUProvider: rudder/toe-brake override enabled\n"
                            : "DCUProvider: rudder/toe-brake override released\n");
}
// ----

float DataRefManager::readFloat(XPLMDataRef dr, float def)
{
    if (!dr)
    {
        return def;
    }
    return XPLMGetDataf(dr);
}

int DataRefManager::readInt(XPLMDataRef dr, int def)
{
    if (!dr)
    {
        return def;
    }
    return XPLMGetDatai(dr);
}

std::vector<float> DataRefManager::readFloatArray(XPLMDataRef dr, int index, int count)
{
    std::vector<float> result(count, 0.0f);
    if (dr && count > 0)
    {
        XPLMGetDatavf(dr, result.data(), index, count);
    }
    return result;
}

XPLMDataRef DataRefManager::resolveLazy(XPLMDataRef& cached, const char* name)
{
    if (!cached)
    {
        cached = XPLMFindDataRef(name);
    }
    return cached;
}