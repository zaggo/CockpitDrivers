#include "ConfigUtils.h"
#include <fstream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <map>

std::string getConfigFilePath() {
    const char* home = std::getenv("HOME");
    if (!home) return ".dcuprovider.cfg";
    return std::string(home) + "/.dcuprovider.cfg";
}

// Helper: Load all config values into a map
static std::map<std::string, std::string> loadConfig() {
    std::map<std::string, std::string> config;
    std::ifstream f(getConfigFilePath());
    if (f.is_open()) {
        std::string line;
        while (std::getline(f, line)) {
            size_t pos = line.find('=');
            if (pos != std::string::npos) {
                std::string key = line.substr(0, pos);
                std::string value = line.substr(pos + 1);
                config[key] = value;
            }
        }
    }
    return config;
}

// Helper: Save all config values from a map
static void saveConfig(const std::map<std::string, std::string>& config) {
    std::ofstream f(getConfigFilePath());
    if (f.is_open()) {
        for (const auto& kv : config) {
            f << kv.first << "=" << kv.second << std::endl;
        }
    }
}

std::string loadLastUsedPort() {
    auto config = loadConfig();
    auto it = config.find("port");
    return (it != config.end()) ? it->second : "";
}

void saveLastUsedPort(const std::string& port) {
    auto config = loadConfig();
    config["port"] = port;
    saveConfig(config);
}

bool loadStatusWindowVisible() {
    auto config = loadConfig();
    auto it = config.find("window_visible");
    // Default: true (sichtbar) wenn nicht gesetzt
    if (it == config.end()) return true;
    return it->second == "1" || it->second == "true";
}

void saveStatusWindowVisible(bool visible) {
    auto config = loadConfig();
    config["window_visible"] = visible ? "1" : "0";
    saveConfig(config);
}

namespace {

const char* const kCurveParams[] = { "sens_pos", "sens_neg", "neutral", "dead_zone", "extremity" };

float* curveField(AxisCurve& c, int i) {
    switch (i) {
        case 0: return &c.sensPos;
        case 1: return &c.sensNeg;
        case 2: return &c.neutral;
        case 3: return &c.deadZone;
        default: return &c.extremityDeadZone;
    }
}

// strtof accepts "nan"/"inf" and returns 0 on garbage; clampToRange() below
// turns NaN into 0 and clamps the rest, so nothing unsafe reaches the sim.
float parseFloat(const std::string& s) {
    return std::strtof(s.c_str(), nullptr);
}

} // namespace

AxisCurve loadAxisCurve(const std::string& axis) {
    auto config = loadConfig();
    AxisCurve c;
    for (int i = 0; i < 5; ++i) {
        auto it = config.find("curve." + axis + "." + kCurveParams[i]);
        if (it != config.end())
            *curveField(c, i) = parseFloat(it->second);
    }
    c.clampToRange();
    return c;
}

void saveAxisCurve(const std::string& axis, const AxisCurve& curve) {
    auto config = loadConfig();
    AxisCurve c = curve;
    for (int i = 0; i < 5; ++i) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.4f", *curveField(c, i));
        config["curve." + axis + "." + kCurveParams[i]] = buf;
    }
    saveConfig(config);
}

bool loadCurveWindowVisible() {
    auto config = loadConfig();
    auto it = config.find("curve_window_visible");
    if (it == config.end()) return false;
    return it->second == "1" || it->second == "true";
}

void saveCurveWindowVisible(bool visible) {
    auto config = loadConfig();
    config["curve_window_visible"] = visible ? "1" : "0";
    saveConfig(config);
}
