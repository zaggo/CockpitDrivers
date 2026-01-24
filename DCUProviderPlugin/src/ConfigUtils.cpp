#include "ConfigUtils.h"
#include <fstream>
#include <cstdlib>
#include <string>

std::string getConfigFilePath() {
    const char* home = std::getenv("HOME");
    if (!home) return ".dcuprovider.cfg";
    return std::string(home) + "/.dcuprovider.cfg";
}

std::string loadLastUsedPort() {
    std::ifstream f(getConfigFilePath());
    std::string port;
    if (f.is_open()) {
        std::getline(f, port);
    }
    return port;
}

void saveLastUsedPort(const std::string& port) {
    std::ofstream f(getConfigFilePath());
    if (f.is_open()) {
        f << port << std::endl;
    }
}
