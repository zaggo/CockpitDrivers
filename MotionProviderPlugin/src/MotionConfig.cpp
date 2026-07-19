#include "MotionConfig.h"
#include "toml.hpp"
#include <cstdlib>

namespace {

// toml++ value<double>() does NOT accept integer literals (and value<int64_t>()
// does not accept floats). Config authors will freely mix `= 120` and `= 120.0`,
// so read every numeric field leniently: try the exact type, then the other.

bool getDouble(const toml::table& t, const char* key, double& out) {
    auto n = t[key];
    if (auto v = n.value<double>())  { out = *v; return true; }
    if (auto v = n.value<int64_t>()) { out = static_cast<double>(*v); return true; }
    return false;
}

bool getInt(const toml::table& t, const char* key, int& out) {
    auto n = t[key];
    if (auto v = n.value<int64_t>()) { out = static_cast<int>(*v); return true; }
    if (auto v = n.value<double>())  { out = static_cast<int>(*v); return true; }
    return false;
}

void getArr6d(const toml::table& t, const char* key, double out[6]) {
    if (auto arr = t[key].as_array()) {
        for (int i = 0; i < 6 && i < static_cast<int>(arr->size()); ++i) {
            auto e = arr->get(i);
            if (auto v = e->value<double>())       out[i] = *v;
            else if (auto v = e->value<int64_t>()) out[i] = static_cast<double>(*v);
        }
    }
}

void getArr6i(const toml::table& t, const char* key, int out[6]) {
    if (auto arr = t[key].as_array()) {
        for (int i = 0; i < 6 && i < static_cast<int>(arr->size()); ++i) {
            auto e = arr->get(i);
            if (auto v = e->value<int64_t>())     out[i] = static_cast<int>(*v);
            else if (auto v = e->value<double>()) out[i] = static_cast<int>(*v);
        }
    }
}

}  // namespace

std::string MotionConfig::defaultPath() {
    const char* home = std::getenv("HOME");
    return home ? std::string(home) + "/.motionprovider.toml"
                : std::string(".motionprovider.toml");
}

StewartGeometry MotionConfig::loadGeometry(const std::string& path, bool* outLoaded) {
    if (outLoaded) *outLoaded = false;
    StewartGeometry g = StewartGeometry::defaults();

    toml::table tbl;
    try {
        tbl = toml::parse_file(path);
    } catch (const toml::parse_error&) {
        return g;  // missing or invalid file -> full defaults, outLoaded stays false
    }
    if (outLoaded) *outLoaded = true;

    if (auto geo = tbl["geometry"].as_table()) {
        getDouble(*geo, "base_radius_mm",     g.baseRadius);
        getDouble(*geo, "platform_radius_mm", g.platformRadius);
        getDouble(*geo, "horn_length_mm",     g.hornLength);
        getDouble(*geo, "rod_length_mm",      g.rodLength);
        getArr6d(*geo, "base_angle_deg",   g.phiDeg);
        getArr6d(*geo, "anchor_angle_deg", g.psiDeg);
        getArr6d(*geo, "horn_azimuth_deg", g.betaDeg);
        getArr6i(*geo, "bff_actuator",     g.bff);
    }

    if (auto servo = tbl["servo"].as_table()) {
        getDouble(*servo, "angle_at_full_scale_deg", g.angleAtFullScale);
        getInt(*servo, "demand_home", g.demandHome);
        getInt(*servo, "demand_max",  g.demandMax);
    }

    return g;
}

WashoutConfig MotionConfig::loadWashout(const std::string& path) {
    WashoutConfig w = WashoutConfig::defaults();
    toml::table tbl;
    try { tbl = toml::parse_file(path); } catch (const toml::parse_error&) { return w; }
    if (auto t = tbl["washout"].as_table()) {
        getDouble(*t, "heave_gain", w.heaveGain);
        getDouble(*t, "heave_hp_tau", w.heaveHpTau);
        getDouble(*t, "heave_vel_washout_tau", w.heaveVelWashoutTau);
        getDouble(*t, "heave_pos_washout_tau", w.heavePosWashoutTau);
        getDouble(*t, "heave_limit_mm", w.heaveLimitMm);
        getDouble(*t, "tilt_surge_gain", w.tiltSurgeGain);
        getDouble(*t, "tilt_sway_gain", w.tiltSwayGain);
        getDouble(*t, "tilt_lp_tau", w.tiltLpTau);
        getDouble(*t, "tilt_limit_deg", w.tiltLimitDeg);
        getDouble(*t, "tilt_rate_limit_dps", w.tiltRateLimitDps);
        getDouble(*t, "rot_roll_gain", w.rotRollGain);
        getDouble(*t, "rot_pitch_gain", w.rotPitchGain);
        getDouble(*t, "rot_yaw_gain", w.rotYawGain);
        getDouble(*t, "rot_hp_tau", w.rotHpTau);
        getDouble(*t, "rot_washout_tau", w.rotWashoutTau);
        getDouble(*t, "rot_limit_deg", w.rotLimitDeg);
    }
    return w;
}

EffectsConfig MotionConfig::loadEffects(const std::string& path) {
    EffectsConfig e = EffectsConfig::defaults();
    toml::table tbl;
    try { tbl = toml::parse_file(path); } catch (const toml::parse_error&) { return e; }
    if (auto t = tbl["effects"].as_table()) {
        getDouble(*t, "touchdown_gain", e.touchdownGain);
        getDouble(*t, "touchdown_freq_hz", e.touchdownFreqHz);
        getDouble(*t, "touchdown_decay_tau", e.touchdownDecayTau);
        getDouble(*t, "rumble_gain", e.rumbleGain);
        getDouble(*t, "rumble_freq_hz", e.rumbleFreqHz);
        getDouble(*t, "rumble_speed_ref_mps", e.rumbleSpeedRefMps);
        getDouble(*t, "engine_gain", e.engineGain);
        getDouble(*t, "buffet_gain", e.buffetGain);
    }
    return e;
}

SerialConfig MotionConfig::loadSerial(const std::string& path) {
    SerialConfig s = SerialConfig::defaults();
    toml::table tbl;
    try { tbl = toml::parse_file(path); } catch (const toml::parse_error&) { return s; }
    if (auto t = tbl["serial"].as_table()) {
        getInt(*t, "baud", s.baud);
        getDouble(*t, "output_rate_hz", s.rateHz);
    }
    return s;
}

SafetyConfig MotionConfig::loadSafety(const std::string& path) {
    SafetyConfig s = SafetyConfig::defaults();
    toml::table tbl;
    try { tbl = toml::parse_file(path); } catch (const toml::parse_error&) { return s; }
    if (auto t = tbl["safety"].as_table()) {
        getDouble(*t, "max_velocity_cps", s.maxVelocity);
        getDouble(*t, "max_acceleration_cps2", s.maxAcceleration);
    }
    return s;
}
