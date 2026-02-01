#pragma once
#include <string>

// Returns the path to the config file for the plugin (e.g., ~/.dcuprovider.cfg)
std::string getConfigFilePath();

// Loads the last used port from config file. Returns empty string if not set.
std::string loadLastUsedPort();

// Saves the last used port to config file.
void saveLastUsedPort(const std::string& port);

// Loads the last status window visibility state. Returns true if no saved state (default: visible).
bool loadStatusWindowVisible();

// Saves the status window visibility state.
void saveStatusWindowVisible(bool visible);
