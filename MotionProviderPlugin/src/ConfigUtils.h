#pragma once
#include <string>

std::string getConfigFilePath();
std::string loadLastUsedPort();
void saveLastUsedPort(const std::string& port);
bool loadStatusWindowVisible();
void saveStatusWindowVisible(bool visible);
