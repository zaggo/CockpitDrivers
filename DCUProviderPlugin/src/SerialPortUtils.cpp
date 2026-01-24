#include "SerialPortUtils.h"

#ifdef _WIN32
#include <windows.h>
#include <setupapi.h>
#include <devguid.h>
#include <regstr.h>
#else
#include <glob.h>
#endif

std::vector<std::string> enumerateSerialPorts() {
    std::vector<std::string> ports;
    
#ifdef _WIN32
    // Windows: enumerate COM ports using registry
    // Simple approach: try COM1 through COM256
    for (int i = 1; i <= 256; i++) {
        std::string portName = "COM" + std::to_string(i);
        std::string fullPath = "\\\\.\\" + portName;
        
        HANDLE hPort = CreateFileA(
            fullPath.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0, NULL, OPEN_EXISTING, 0, NULL
        );
        
        if (hPort != INVALID_HANDLE_VALUE) {
            CloseHandle(hPort);
            ports.push_back(portName);
        }
    }
#else
    // macOS/Linux: use glob to find /dev/cu.* or /dev/ttyUSB*
    glob_t glob_result;
    // Look for /dev/cu.* devices (macOS)
    if (glob("/dev/cu.*", 0, nullptr, &glob_result) == 0) {
        for (size_t i = 0; i < glob_result.gl_pathc; ++i) {
            ports.emplace_back(glob_result.gl_pathv[i]);
        }
        globfree(&glob_result);
    }
#endif
    
    return ports;
}
