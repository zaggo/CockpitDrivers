#pragma once
#include <vector>
#include <string>

// Returns the serial ports present on the system: COMx from the registry on
// Windows (open ports included), /dev/cu.* on macOS.
std::vector<std::string> enumerateSerialPorts();
