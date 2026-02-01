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
