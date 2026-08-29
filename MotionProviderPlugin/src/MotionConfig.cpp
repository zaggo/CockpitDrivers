#include "MotionConfig.h"
// toml++ (vendored) trips -Wdeprecated-literal-operator on newer clang/gcc in
// its own headers. Silence it just for this include; don't edit the vendored
// file (it's a local, re-vendorable copy). MSVC ignores the unknown pragma.
#if defined(__clang__) || defined(__GNUC__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wdeprecated-literal-operator"
#endif
#include "toml.hpp"
#if defined(__clang__) || defined(__GNUC__)
#  pragma GCC diagnostic pop
#endif
#include <cstdlib>
#include <fstream>

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
    if (!home) home = std::getenv("USERPROFILE");  // Windows has no HOME
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
        getDouble(*t, "smooth_tau", w.smoothTau);
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
        getDouble(*t, "park_heave_mm", s.parkHeaveMm);
        getDouble(*t, "arm_ramp_sec", s.armRampSec);
        getDouble(*t, "disarm_ramp_sec", s.disarmRampSec);
        getDouble(*t, "runaway_tilt_deg", s.runawayTiltDeg);
        getDouble(*t, "runaway_trans_mm", s.runawayTransMm);
        getDouble(*t, "runaway_hold_sec", s.runawayHoldSec);
        getDouble(*t, "max_dt_sec", s.maxDtSec);
    }
    // The actor firmware clamps a goto move to 30 s; keep the ramp times in
    // the same range so the ArmRamp blend and the goto timer stay in sync.
    if (s.armRampSec < 0.1)     s.armRampSec = 0.1;
    if (s.armRampSec > 30.0)    s.armRampSec = 30.0;
    if (s.disarmRampSec < 0.1)  s.disarmRampSec = 0.1;
    if (s.disarmRampSec > 30.0) s.disarmRampSec = 30.0;
    return s;
}

TelemetryConfig MotionConfig::loadTelemetry(const std::string& path) {
    TelemetryConfig cfg = TelemetryConfig::defaults();
    toml::table tbl;
    try { tbl = toml::parse_file(path); } catch (...) { return cfg; }
    if (auto* t = tbl["telemetry"].as_table()) {
        if (auto v = (*t)["enabled"].value<bool>())        cfg.enabled = *v;
        if (auto v = (*t)["dir"].value<std::string>())     cfg.dir     = *v;
    }
    return cfg;
}

bool MotionConfig::writeDefaults(const std::string& path) {
    std::ofstream f(path);
    if (!f.is_open()) return false;
    f.precision(12);

    const StewartGeometry g = StewartGeometry::defaults();
    const WashoutConfig   w = WashoutConfig::defaults();
    const EffectsConfig   e = EffectsConfig::defaults();
    const SerialConfig    s = SerialConfig::defaults();
    const SafetyConfig   sf = SafetyConfig::defaults();

    auto arrD = [&](const char* key, const double v[6]) {
        f << key << " = [" << v[0];
        for (int i = 1; i < 6; ++i) f << ", " << v[i];
        f << "]\n";
    };
    auto arrI = [&](const char* key, const int v[6]) {
        f << key << " = [" << v[0];
        for (int i = 1; i < 6; ++i) f << ", " << v[i];
        f << "]\n";
    };

    f << "# MotionProviderPlugin configuration (auto-generated with default values).\n";
    f << "# Edit values and click \"Reload config\" in the status window to apply.\n\n";

    f << "[geometry]\n";
    f << "base_radius_mm = "     << g.baseRadius     << "\n";
    f << "platform_radius_mm = " << g.platformRadius << "\n";
    f << "horn_length_mm = "     << g.hornLength     << "\n";
    f << "rod_length_mm = "      << g.rodLength      << "\n";
    arrD("base_angle_deg",   g.phiDeg);
    arrD("anchor_angle_deg", g.psiDeg);
    arrD("horn_azimuth_deg", g.betaDeg);
    arrI("bff_actuator",     g.bff);
    f << "\n[servo]\n";
    f << "angle_at_full_scale_deg = " << g.angleAtFullScale << "\n";
    f << "demand_home = " << g.demandHome << "\n";
    f << "demand_max = "  << g.demandMax  << "\n";

    f << "\n[washout]\n";
    f << "heave_gain = "            << w.heaveGain          << "\n";
    f << "heave_hp_tau = "          << w.heaveHpTau         << "\n";
    f << "heave_vel_washout_tau = " << w.heaveVelWashoutTau << "\n";
    f << "heave_pos_washout_tau = " << w.heavePosWashoutTau << "\n";
    f << "heave_limit_mm = "        << w.heaveLimitMm       << "\n";
    f << "tilt_surge_gain = "       << w.tiltSurgeGain      << "\n";
    f << "tilt_sway_gain = "        << w.tiltSwayGain       << "\n";
    f << "tilt_lp_tau = "           << w.tiltLpTau          << "\n";
    f << "tilt_limit_deg = "        << w.tiltLimitDeg       << "\n";
    f << "tilt_rate_limit_dps = "   << w.tiltRateLimitDps   << "\n";
    f << "rot_roll_gain = "         << w.rotRollGain        << "\n";
    f << "rot_pitch_gain = "        << w.rotPitchGain       << "\n";
    f << "rot_yaw_gain = "          << w.rotYawGain         << "\n";
    f << "rot_hp_tau = "            << w.rotHpTau           << "\n";
    f << "rot_washout_tau = "       << w.rotWashoutTau      << "\n";
    f << "rot_limit_deg = "         << w.rotLimitDeg        << "\n";
    f << "smooth_tau = "            << w.smoothTau          << "\n";

    f << "\n[effects]\n";
    f << "touchdown_gain = "       << e.touchdownGain     << "\n";
    f << "touchdown_freq_hz = "    << e.touchdownFreqHz   << "\n";
    f << "touchdown_decay_tau = "  << e.touchdownDecayTau << "\n";
    f << "rumble_gain = "          << e.rumbleGain        << "\n";
    f << "rumble_freq_hz = "       << e.rumbleFreqHz      << "\n";
    f << "rumble_speed_ref_mps = " << e.rumbleSpeedRefMps << "\n";
    f << "engine_gain = "          << e.engineGain        << "\n";
    f << "buffet_gain = "          << e.buffetGain        << "\n";

    f << "\n[serial]\n";
    f << "baud = "           << s.baud   << "\n";
    f << "output_rate_hz = " << s.rateHz << "\n";

    f << "\n[safety]\n";
    f << "max_velocity_cps = "      << sf.maxVelocity     << "\n";
    f << "max_acceleration_cps2 = " << sf.maxAcceleration << "\n";
    f << "park_heave_mm = "         << sf.parkHeaveMm     << "\n";
    f << "arm_ramp_sec = "          << sf.armRampSec      << "\n";
    f << "disarm_ramp_sec = "       << sf.disarmRampSec   << "\n";
    f << "runaway_tilt_deg = "      << sf.runawayTiltDeg  << "\n";
    f << "runaway_trans_mm = "      << sf.runawayTransMm  << "\n";
    f << "runaway_hold_sec = "      << sf.runawayHoldSec  << "\n";
    f << "max_dt_sec = "            << sf.maxDtSec        << "\n";

    f << "\n[telemetry]\n";
    f << "enabled = false # set true to auto-start CSV recording on plugin load\n";
    f << "dir = \"\" # output directory; empty = the plugin directory\n";

    return f.good();
}
