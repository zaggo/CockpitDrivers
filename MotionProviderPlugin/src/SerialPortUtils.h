#pragma once
#include <vector>
#include <string>

// Returns a list of available serial ports (e.g., /dev/cu.*) on macOS
std::vector<std::string> enumerateSerialPorts();
