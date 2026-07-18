#include "ConfigUtils.h"
#include <fstream>
#include <cstdlib>
#include <string>
#include <map>

std::string getConfigFilePath() {
    const char* home = std::getenv("HOME");
    if (!home) return ".motionprovider.cfg";
    return std::string(home) + "/.motionprovider.cfg";
}

static std::map<std::string, std::string> loadConfig() {
    std::map<std::string, std::string> config;
    std::ifstream f(getConfigFilePath());
    if (f.is_open()) {
        std::string line;
        while (std::getline(f, line)) {
            size_t pos = line.find('=');
            if (pos != std::string::npos) {
                config[line.substr(0, pos)] = line.substr(pos + 1);
            }
        }
    }
    return config;
}

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
    if (it == config.end()) return true;
    return it->second == "1" || it->second == "true";
}

void saveStatusWindowVisible(bool visible) {
    auto config = loadConfig();
    config["window_visible"] = visible ? "1" : "0";
    saveConfig(config);
}
